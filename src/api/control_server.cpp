// See control_server.h. cpp-httplib blocking server on a dedicated thread; JSON is emitted
// with snprintf and parsed with strstr — the API surface is tiny and we keep zero JSON deps.
#include "api/control_server.h"

#include <httplib.h>

#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>

#include "core/geo.h"
#include "radio/json.h"
#include "radio/radio_spec.h"

namespace skysim::api {

namespace {

// Two long-lived camera streams per vehicle, plus headroom for the control
// plane, which must stay answerable while they run.
constexpr size_t kWorkerThreads = 64;

// How long an MJPEG stream blocks waiting for the next frame before it gives the server a turn
// (a shutdown is noticed between waits). A frame goes out the moment it is published, so this
// does not set the rate or the latency — the 5 ms poll it replaces did neither either, and it
// copied the whole JPEG out of the slot two hundred times a second per viewer to find that out.
constexpr auto kFrameWait = std::chrono::milliseconds(250);

// When the wait came back with nothing new straight away — the vehicle is not in the fleet, or
// the camera does not block — the stream sleeps this long before asking again.
constexpr auto kFrameRetry = std::chrono::milliseconds(20);

// How long camera.jpg waits for a first frame. Rendering is on demand, so the first request for
// a vehicle nobody is watching finds no frame yet: it is what starts the drawing, and one frame
// is tens of milliseconds away. Bounded so a camera that cannot keep up still answers.
constexpr auto kFirstFrameWait = std::chrono::milliseconds(2000);

// One multipart part: headers plus the JPEG.
//
// Only ever called for a frame the caller has not sent before. Re-sending whatever is in the
// slot on a fixed cadence — which is what this did — puts duplicates on the wire at the poll
// rate rather than the render rate: measured at 24.7 frames/s carrying 10 frames/s of content,
// 59% repeats and 37 KB/s of them. Worse than the waste, it feeds a consumer faster than the
// content arrives, and the excess lands in socket and queue buffers as latency the viewer sees
// as lag. FrameStore drops rather than queues for exactly this reason; this route was undoing it.
//
// Each part says when its picture was taken: X-Skysim-Time-Usec is the sim time of the pose (as
// on a survey still), X-Skysim-Capture-Unix-Usec the wall clock when it was drawn or re-sent.
// Multipart readers (GStreamer's multipartdemux, ffmpeg's mpjpeg) skip headers they do not know.
void write_mjpeg_part(httplib::DataSink &sink, const skysim::render::FrameStore::Frame &frame) {
    char head[256];
    const int n =
        std::snprintf(head, sizeof(head),
                      "--skysimframe\r\nContent-Type: image/jpeg\r\n"
                      "Content-Length: %zu\r\nX-Skysim-Time-Usec: %lld\r\n"
                      "X-Skysim-Capture-Unix-Usec: %lld\r\n\r\n",
                      frame.jpeg.size(), static_cast<long long>(std::llround(frame.sim_time_s * 1e6)),
                      static_cast<long long>(frame.captured_unix_us));
    sink.write(head, static_cast<size_t>(n));
    sink.write(reinterpret_cast<const char *>(frame.jpeg.data()), frame.jpeg.size());
    sink.write("\r\n", 2);
}

// One turn of an MJPEG stream: wait for the vehicle's next frame and write it, once. The wait is
// also the stream's standing request for frames — the camera draws only what is being asked for.
void stream_next_frame(const ControlServer::Snapshots &snapshots, uint32_t id, uint64_t &last_seq,
                       httplib::DataSink &sink) {
    const auto asked = std::chrono::steady_clock::now();
    const auto frame =
        id != 0 ? snapshots.camera_frame(id, last_seq, kFrameWait) : skysim::render::FrameStore::Frame{};
    if (!frame.jpeg.empty() && frame.seq != last_seq) {
        last_seq = frame.seq;
        write_mjpeg_part(sink, frame);
        return;
    }
    const auto waited = std::chrono::steady_clock::now() - asked;
    if (waited < kFrameRetry) {
        std::this_thread::sleep_for(kFrameRetry - waited);
    }
}

// {"launch_process":true} — absent key means false.
bool parse_launch_process(const std::string &body) {
    const char *p = std::strstr(body.c_str(), "\"launch_process\"");
    if (p == nullptr) {
        return false;
    }
    p += std::strlen("\"launch_process\"");
    while (*p == ':' || *p == ' ' || *p == '\t') {
        ++p;
    }
    return std::strncmp(p, "true", 4) == 0 || *p == '1';
}

// {"instance":N} — absent key means -1 (skysim picks the lowest free slot).
int parse_instance(const std::string &body) {
    const char *p = std::strstr(body.c_str(), "\"instance\"");
    if (p == nullptr) {
        return -1;
    }
    p += std::strlen("\"instance\"");
    while (*p == ':' || *p == ' ' || *p == '\t') {
        ++p;
    }
    return std::atoi(p);
}

// One numeric field out of a flat JSON body, or `fallback` when it is absent.
//
// Same hand-rolled shape as the parsers around it: the control plane takes small, known
// bodies from one caller, so a JSON dependency has never been worth it here. A missing
// field must fall back rather than read as zero — a pack of zero capacity is empty from
// the first tick, which would ground a fleet rather than misconfigure it.
double parse_double_field(const std::string &body, const char *field, double fallback) {
    const std::string key = std::string("\"") + field + "\"";
    const char *p = std::strstr(body.c_str(), key.c_str());
    if (p == nullptr) {
        return fallback;
    }
    p += key.size();
    while (*p == ':' || *p == ' ' || *p == '\t') {
        ++p;
    }
    const double value = std::atof(p);
    return value > 0.0 ? value : fallback;
}

// The pack block of a spawn request. Every field is independently optional, so a caller
// that only knows its capacity gets a sane voltage curve for free.
skysim::vehicle::BatteryPack parse_battery(const std::string &body) {
    skysim::vehicle::BatteryPack pack{};
    pack.capacity_mah = parse_double_field(body, "battery_capacity_mah", pack.capacity_mah);
    pack.full_v = parse_double_field(body, "battery_full_v", pack.full_v);
    pack.empty_v = parse_double_field(body, "battery_empty_v", pack.empty_v);
    pack.idle_a = parse_double_field(body, "battery_idle_a", pack.idle_a);
    pack.hover_a = parse_double_field(body, "battery_hover_a", pack.hover_a);
    // An empty voltage at or above full would invert the sag and make a draining pack
    // report a rising charge. Cheaper to reject here than to debug in flight.
    if (pack.empty_v >= pack.full_v) {
        pack.empty_v = pack.full_v * 0.8;
    }
    return pack;
}

// {"clearance_m":1.5} — absent means a conservative default airframe half-width.
double parse_clearance(const std::string &body) { return parse_double_field(body, "clearance_m", 1.0); }

// {"waypoints_ned":[[n,e,d],[n,e,d],...]} — flat scan for numbers in triples, matching the
// deliberately dependency-free parsing style of this file.
std::vector<core::Vec3> parse_waypoints(const std::string &body) {
    std::vector<core::Vec3> out;
    const char *p = std::strstr(body.c_str(), "\"waypoints_ned\"");
    if (p == nullptr) {
        return out;
    }
    p = std::strchr(p, '[');
    if (p == nullptr) {
        return out;
    }
    const char *end = body.c_str() + body.size();

    while (p < end) {
        p = std::strchr(p + 1, '[');
        if (p == nullptr) {
            break;
        }
        core::Vec3 wp{};
        bool ok = true;
        const char *cursor = p + 1;
        for (int axis = 0; axis < 3; ++axis) {
            while (cursor < end && (*cursor == ' ' || *cursor == ',' || *cursor == '\t')) {
                ++cursor;
            }
            if (cursor >= end || (*cursor != '-' && *cursor != '+' && *cursor != '.' && std::isdigit(static_cast<unsigned char>(*cursor)) == 0)) {
                ok = false;
                break;
            }
            char *next = nullptr;
            wp[static_cast<size_t>(axis)] = std::strtod(cursor, &next);
            if (next == cursor) {
                ok = false;
                break;
            }
            cursor = next;
        }
        if (!ok) {
            break;
        }
        out.push_back(wp);
        p = cursor;
    }
    return out;
}

std::string vehicle_json(const VehicleInfo &v) {
    char buf[512];
    std::snprintf(buf, sizeof(buf),
                  "{\"id\":%u,\"instance\":%d,\"json_port\":%d,\"mavlink_tcp\":%d,"
                  "\"connected\":%s,\"frozen\":%s,\"silent_ticks\":%llu,"
                  "\"midair_collisions\":%llu,\"static_contacts\":%llu,"
                  "\"building_contacts\":%llu,\"pos_ned\":[%.3f,%.3f,%.3f]}",
                  v.id, v.instance, v.json_port, v.mavlink_tcp_port, v.connected ? "true" : "false",
                  v.frozen ? "true" : "false", static_cast<unsigned long long>(v.silent_ticks),
                  static_cast<unsigned long long>(v.midair_collisions),
                  static_cast<unsigned long long>(v.static_contacts),
                  static_cast<unsigned long long>(v.building_contacts), v.pos_ned[0], v.pos_ned[1],
                  v.pos_ned[2]);
    return buf;
}

// --- /radio helpers ---------------------------------------------------------------------------

void reply_error(httplib::Response &res, int status, const std::string &message) {
    std::string body = "{\"error\":";
    radio::json::append_string(body, message);
    body += '}';
    res.status = status;
    res.set_content(body, "application/json");
}

// A query parameter as a finite number in [lo, hi]. Absent leaves `out` empty and succeeds;
// present but malformed or out of range fails with a message naming it.
bool query_number(const httplib::Request &req, const char *name, double lo, double hi,
                  std::optional<double> &out, std::string &error) {
    out.reset();
    if (!req.has_param(name)) {
        return true;
    }
    const std::string text = req.get_param_value(name);
    double v = 0.0;
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), v);
    if (text.empty() || parsed.ec != std::errc() || parsed.ptr != text.data() + text.size() ||
        !std::isfinite(v)) {
        error = std::string(name) + " must be a number";
        return false;
    }
    if (v < lo || v > hi) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%s must be between %g and %g", name, lo, hi);
        error = buf;
        return false;
    }
    out = v;
    return true;
}

constexpr double kMinAltitudeM = -1000.0;
constexpr double kMaxAltitudeM = 100000.0;

// The vertical position of a request. h (alt_amsl) is what towers are compared against;
// alt_rel sets the path-loss exponent. Given alt_amsl, alt_rel defaults to alt_amsl minus
// ground_amsl; given only alt_rel, alt_amsl = alt_rel + ground_amsl. ground_amsl defaults to
// 0 m, where the two coincide.
bool query_altitude(const httplib::Request &req, double &alt_rel, double &alt_amsl, std::string &error) {
    std::optional<double> amsl;
    std::optional<double> rel;
    std::optional<double> ground;
    if (!query_number(req, "alt_amsl", kMinAltitudeM, kMaxAltitudeM, amsl, error) ||
        !query_number(req, "alt_rel", kMinAltitudeM, kMaxAltitudeM, rel, error) ||
        !query_number(req, "ground_amsl", kMinAltitudeM, kMaxAltitudeM, ground, error)) {
        return false;
    }
    if (!amsl && !rel) {
        error = "alt_amsl or alt_rel (with ground_amsl) is required";
        return false;
    }
    const double g = ground.value_or(0.0);
    alt_amsl = amsl ? *amsl : *rel + g;
    alt_rel = rel ? *rel : *amsl - g;
    return true;
}

// "minLon,minLat,maxLon,maxLat".
bool query_bbox(const httplib::Request &req, radio::GridRequest &grid, std::string &error) {
    const std::string text = req.get_param_value("bbox");
    double v[4] = {0.0, 0.0, 0.0, 0.0};
    const char *p = text.data();
    const char *end = p + text.size();
    for (int i = 0; i < 4; ++i) {
        const auto parsed = std::from_chars(p, end, v[i]);
        const bool last = i == 3;
        if (parsed.ec != std::errc() || !std::isfinite(v[i]) ||
            (last ? parsed.ptr != end : *parsed.ptr != ',')) {
            error = "bbox must be minLon,minLat,maxLon,maxLat";
            return false;
        }
        p = parsed.ptr + 1;
    }
    grid.min_lon_deg = v[0];
    grid.min_lat_deg = v[1];
    grid.max_lon_deg = v[2];
    grid.max_lat_deg = v[3];
    if (v[0] < -180.0 || v[2] > 180.0 || v[1] < -90.0 || v[3] > 90.0 || v[0] > v[2] || v[1] > v[3]) {
        error = "bbox must lie within ±180 lon / ±90 lat with min <= max";
        return false;
    }
    return true;
}

std::string radio_put_summary(const radio::RadioSpec &spec) {
    size_t towers = 0;
    for (const auto &c : spec.carriers) {
        towers += c.towers.size();
    }
    std::string out = "{\"ok\":true,\"carriers\":" + std::to_string(spec.carriers.size()) +
                      ",\"towers\":" + std::to_string(towers) +
                      ",\"holes\":" + std::to_string(spec.holes.size()) + ",\"modems\":[";
    for (size_t i = 0; i < spec.modems.size(); ++i) {
        out += i ? "," : "";
        radio::json::append_string(out, spec.modems[i]);
    }
    out += "]}";
    return out;
}

// --- survey still camera ----------------------------------------------------------------------

// A query parameter as a whole number in [lo, hi]; absent leaves `out` alone and succeeds.
bool query_int(const httplib::Request &req, const char *name, int lo, int hi, int &out, std::string &error) {
    if (!req.has_param(name)) {
        return true;
    }
    const std::string text = req.get_param_value(name);
    long v = 0;
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), v);
    if (text.empty() || parsed.ec != std::errc() || parsed.ptr != text.data() + text.size() || v < lo ||
        v > hi) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%s must be an integer between %d and %d", name, lo, hi);
        error = buf;
        return false;
    }
    out = static_cast<int>(v);
    return true;
}

// The still's query: w, h, hfov, pitch, yaw, roll_stab, gimbal, range, q — every one optional,
// every one bounded, defaults as in render::StillRequest.
bool parse_still_query(const httplib::Request &req, render::StillRequest &r, std::string &error) {
    using R = render::StillRequest;
    std::optional<double> hfov;
    std::optional<double> pitch;
    std::optional<double> yaw;
    std::optional<double> range;
    if (!query_int(req, "w", R::kMinSize, R::kMaxSize, r.width, error) ||
        !query_int(req, "h", R::kMinSize, R::kMaxSize, r.height, error) ||
        !query_number(req, "hfov", R::kMinHfovDeg, R::kMaxHfovDeg, hfov, error) ||
        !query_number(req, "pitch", R::kMinPitchDeg, R::kMaxPitchDeg, pitch, error) ||
        !query_number(req, "yaw", -R::kMaxYawDeg, R::kMaxYawDeg, yaw, error) ||
        !query_number(req, "range", R::kMinRangeM, R::kMaxRangeM, range, error) ||
        !query_int(req, "q", R::kMinQuality, R::kMaxQuality, r.quality, error)) {
        return false;
    }
    r.hfov_deg = hfov.value_or(r.hfov_deg);
    r.pitch_deg = pitch.value_or(r.pitch_deg);
    r.yaw_deg = yaw.value_or(r.yaw_deg);
    r.range_m = range.value_or(r.range_m);
    if (req.has_param("roll_stab")) {
        const std::string v = req.get_param_value("roll_stab");
        if (v == "1" || v == "true") {
            r.roll_stab = true;
        } else if (v == "0" || v == "false") {
            r.roll_stab = false;
        } else {
            error = "roll_stab must be 0 or 1";
            return false;
        }
    }
    if (req.has_param("gimbal")) {
        const std::string v = req.get_param_value("gimbal");
        if (v == "1" || v == "true") {
            r.follow_gimbal = true;
        } else if (v == "0" || v == "false") {
            r.follow_gimbal = false;
        } else {
            error = "gimbal must be 0 or 1";
            return false;
        }
    }
    return true;
}

void set_header(httplib::Response &res, const char *name, const char *format, double value) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), format, value);
    res.set_header(name, buf);
}

// One still for one vehicle: render from the snapshot it came with, stamp the pose it was
// rendered from, or say plainly why not.
void reply_still(const ControlServer::Snapshots &snapshots, const std::optional<radio::GeoAnchor> &home,
                 const VehicleInfo &v, const render::StillRequest &request, httplib::Response &res) {
    const render::StillFrame frame = snapshots.still(v, request);
    switch (frame.status) {
    case render::StillFrame::Status::kBusy:
        // Rendering takes a fraction of a second, so a second is long enough to wait.
        res.set_header("Retry-After", "1");
        reply_error(res, 503, frame.error);
        return;
    case render::StillFrame::Status::kInvalid:
        reply_error(res, 400, frame.error);
        return;
    case render::StillFrame::Status::kDisabled:
        reply_error(res, 404, frame.error.empty() ? "still camera disabled" : frame.error);
        return;
    case render::StillFrame::Status::kFailed:
        reply_error(res, 500, frame.error);
        return;
    case render::StillFrame::Status::kOk:
        break;
    }

    res.set_header("X-Skysim-Time-Usec", std::to_string(std::llround(v.sim_time_s * 1e6)));
    if (home) {
        const core::GeoPoint geo =
            core::geo_from_ned(core::GeoOrigin{home->lat_deg, home->lon_deg, home->ground_amsl_m},
                               v.pos_ned[0], v.pos_ned[1], v.pos_ned[2]);
        set_header(res, "X-Skysim-Lat", "%.9f", geo.lat_deg);
        set_header(res, "X-Skysim-Lon", "%.9f", geo.lon_deg);
        set_header(res, "X-Skysim-Alt-Amsl", "%.3f", geo.alt_amsl_m);
    } else {
        // --spawn-home did not parse (skysim said so at startup): there is no earth position
        // to give, and an invented one would geotag a survey somewhere it was not flown.
        res.set_header("X-Skysim-Warning",
                       "no geo anchor: --spawn-home is not lat,lon[,alt,yaw]; no lat/lon");
    }
    set_header(res, "X-Skysim-Alt-Rel", "%.3f", -v.pos_ned[2]);
    set_header(res, "X-Skysim-Yaw", "%.4f", frame.camera.yaw_deg);
    set_header(res, "X-Skysim-Pitch", "%.4f", frame.camera.pitch_deg);
    set_header(res, "X-Skysim-Roll", "%.4f", frame.camera.roll_deg);
    // Whether those angles are the vehicle's gimbal's (gimbal=1 and it has one) or the request's.
    res.set_header("X-Skysim-Gimbal", frame.gimbal ? "1" : "0");
    set_header(res, "X-Skysim-Hfov", "%.4f", request.hfov_deg);
    char ned[96];
    std::snprintf(ned, sizeof(ned), "%.3f,%.3f,%.3f", v.pos_ned[0], v.pos_ned[1], v.pos_ned[2]);
    res.set_header("X-Skysim-Ned", ned);
    res.set_header("X-Skysim-Vehicle", std::to_string(v.id));
    res.set_header("X-Skysim-Instance", std::to_string(v.instance));
    res.set_header("Cache-Control", "no-store");
    res.set_content(reinterpret_cast<const char *>(frame.jpeg.data()), frame.jpeg.size(), "image/jpeg");
}

// The vehicle at an instance named in a route, or null: the whole number in the path, as the
// still route reads it (no wrap onto another vehicle for a number too big for the type).
const VehicleInfo *find_instance(const std::vector<VehicleInfo> &vehicles, const std::string &key) {
    uint64_t number = 0;
    const auto parsed = std::from_chars(key.data(), key.data() + key.size(), number);
    if (parsed.ec != std::errc() || parsed.ptr != key.data() + key.size() || number > 0x7FFFFFFFULL) {
        return nullptr;
    }
    for (const auto &v : vehicles) {
        if (static_cast<uint64_t>(v.instance) == number) {
            return &v;
        }
    }
    return nullptr;
}

// A POST /camera/encoder body: a JSON object whose bitrate_kbps, fps and keyframe_interval_s,
// when present, are numbers. Other members are ignored, so a client can send back what GET
// gave it with one field changed. An empty body changes nothing.
bool parse_encoder_patch(const std::string &body, EncoderPatch &patch, std::string &error) {
    if (body.find_first_not_of(" \t\r\n") == std::string::npos) {
        return true;
    }
    std::string parse_error;
    const auto doc = radio::json::parse(body, parse_error);
    if (!doc) {
        error = "invalid JSON: " + parse_error;
        return false;
    }
    if (!doc->is_object()) {
        error = "body must be a JSON object";
        return false;
    }
    const std::pair<const char *, std::optional<double> *> fields[] = {
        {"bitrate_kbps", &patch.bitrate_kbps},
        {"fps", &patch.fps},
        {"keyframe_interval_s", &patch.keyframe_interval_s}};
    for (const auto &[name, out] : fields) {
        const radio::json::Value *v = doc->find(name);
        if (v == nullptr) {
            continue;
        }
        if (!v->is_number() || !std::isfinite(v->number)) {
            error = std::string(name) + " must be a number";
            return false;
        }
        *out = v->number;
    }
    return true;
}

std::string encoder_json(const EncoderState &e) {
    std::string out = "{\"bitrate_kbps\":" + std::to_string(e.bitrate_kbps) + ",\"fps\":";
    radio::json::append_number(out, e.fps);
    out += ",\"keyframe_interval_s\":";
    radio::json::append_number(out, e.keyframe_interval_s);
    out +=
        ",\"width\":" + std::to_string(e.width) + ",\"height\":" + std::to_string(e.height) + ",\"max_fps\":";
    radio::json::append_number(out, e.max_fps);
    out += ",\"clients\":" + std::to_string(e.clients) + "}";
    return out;
}

} // namespace

struct ControlServer::Impl {
    httplib::Server server;
    std::thread thread;
};

ControlServer::ControlServer(const std::string &bind_addr, int port, CommandQueue &queue, Snapshots snapshots,
                             RadioConfig radio_config)
    : impl_(std::make_unique<Impl>()) {
    auto &s = impl_->server;
    std::shared_ptr<radio::RadioField> radio_field =
        radio_config.field ? radio_config.field : std::make_shared<radio::RadioField>();
    const std::optional<radio::GeoAnchor> home = radio_config.home;

    // Camera streams hold a worker for their whole life.
    //
    // Each MJPEG response sits in its chunked provider until the client goes
    // away, so it occupies one thread the entire time — and every simulated
    // vehicle opens two (the core container's encoder and the recorder). At
    // httplib's default pool of max(8, ncpu-1), four vehicles on a 2-vCPU task
    // would consume every worker and /vehicles, /metrics and spawn would simply
    // stop answering. Sized for streams rather than for cores.
    s.new_task_queue = [] { return new httplib::ThreadPool(kWorkerThreads); };

    s.Post("/vehicles", [&queue](const httplib::Request &req, httplib::Response &res) {
        SpawnCommand cmd;
        cmd.request.launch_process = parse_launch_process(req.body);
        cmd.request.instance = parse_instance(req.body);
        cmd.request.battery = parse_battery(req.body);
        auto future = cmd.done.get_future();
        queue.push(Command{std::move(cmd)});
        if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
            res.status = 504;
            res.set_content("{\"error\":\"tick thread did not answer\"}", "application/json");
            return;
        }
        const auto result = future.get();
        if (!result.has_value()) {
            res.status = 503;
            res.set_content("{\"error\":\"spawn failed\"}", "application/json");
            return;
        }
        char buf[256];
        std::snprintf(buf, sizeof(buf), "{\"id\":%u,\"instance\":%d,\"json_port\":%d,\"mavlink_tcp\":%d}",
                      result->id, result->instance, result->json_port, result->mavlink_tcp_port);
        res.set_content(buf, "application/json");
    });

    // Sweep a planned mission path through the building geometry. Answers the pre-flight
    // question directly, without flying anything.
    s.Post("/mission/check", [&queue](const httplib::Request &req, httplib::Response &res) {
        PathCheckCommand cmd;
        cmd.waypoints_ned = parse_waypoints(req.body);
        cmd.clearance_m = parse_clearance(req.body);

        if (cmd.waypoints_ned.size() < 2) {
            res.status = 400;
            res.set_content(R"({"error":"at least two waypoints are required"})", "application/json");
            return;
        }

        auto future = cmd.done.get_future();
        queue.push(Command{std::move(cmd)});
        if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
            res.status = 504;
            res.set_content("{\"error\":\"tick thread did not answer\"}", "application/json");
            return;
        }

        const auto result = future.get();
        const auto &hits = result.hits;
        // 'checked' distinguishes "swept and found nothing" from "had nothing to sweep".
        std::string body = std::string("{\"checked\":") + (result.geometry_available ? "true" : "false") +
                           ",\"clear\":" + (result.geometry_available && hits.empty() ? "true" : "false") +
                           ",\"hits\":[";
        for (size_t i = 0; i < hits.size(); ++i) {
            char entry[256];
            std::snprintf(entry, sizeof(entry),
                          "%s{\"leg\":%zu,\"distance_m\":%.2f,\"position_ned\":[%.2f,%.2f,%.2f]}",
                          i ? "," : "", hits[i].leg, hits[i].distance_m, hits[i].position_ned[0],
                          hits[i].position_ned[1], hits[i].position_ned[2]);
            body += entry;
        }
        body += "]}";
        res.set_content(body, "application/json");
    });

    s.Delete(R"(/vehicles/(\d+))", [&queue](const httplib::Request &req, httplib::Response &res) {
        DespawnCommand cmd;
        cmd.id = static_cast<uint32_t>(std::strtoul(req.matches[1].str().c_str(), nullptr, 10));
        auto future = cmd.done.get_future();
        queue.push(Command{std::move(cmd)});
        if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
            res.status = 504;
            return;
        }
        if (!future.get()) {
            res.status = 404;
            res.set_content("{\"error\":\"no such vehicle\"}", "application/json");
            return;
        }
        res.set_content("{\"ok\":true}", "application/json");
    });

    s.Post(R"(/vehicles/(\d+)/battery/reset)", [&queue](const httplib::Request &req, httplib::Response &res) {
        BatteryResetCommand cmd;
        cmd.id = static_cast<uint32_t>(std::strtoul(req.matches[1].str().c_str(), nullptr, 10));
        auto future = cmd.done.get_future();
        queue.push(Command{std::move(cmd)});
        if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
            res.status = 504;
            return;
        }
        if (!future.get()) {
            res.status = 404;
            res.set_content("{\"error\":\"no such vehicle\"}", "application/json");
            return;
        }
        res.set_content("{\"ok\":true}", "application/json");
    });

    s.Get("/vehicles", [snapshots, radio_field, home](const httplib::Request &, httplib::Response &res) {
        // With a radio spec set, each vehicle carries the field's values at its truth
        // position — what a modem on board would report, without the modem.
        const auto model = home ? radio_field->get() : nullptr;
        std::string out = "[";
        bool first = true;
        for (const auto &v : snapshots.vehicles()) {
            std::string entry = vehicle_json(v);
            if (model) {
                entry.pop_back(); // the closing brace
                entry +=
                    ",\"radio\":" + radio::sample_json(*model, radio::position_from_ned(*home, v.pos_ned)) +
                    "}";
            }
            out += (first ? "" : ",") + entry;
            first = false;
        }
        out += "]";
        res.set_content(out, "application/json");
    });

    // --- simulated radio field (docs/RADIO.md) ---------------------------------------------
    //
    // A spec in, ground truth out: the gateway forwards the spec its simulated modems use,
    // and a survey's measurements can then be checked against the field that produced them.
    s.Put("/radio", [radio_field](const httplib::Request &req, httplib::Response &res) {
        radio::SpecResult parsed = radio::parse_radio_spec(req.body);
        if (!parsed.spec) {
            reply_error(res, 400, parsed.error);
            return;
        }
        // Built before the swap, so readers never wait on parsing or see a half-made model.
        auto model = std::make_shared<const radio::RadioModel>(std::move(*parsed.spec));
        const std::string summary = radio_put_summary(model->spec());
        radio_field->set(std::move(model));
        res.set_content(summary, "application/json");
    });

    s.Get("/radio", [radio_field](const httplib::Request &, httplib::Response &res) {
        const auto model = radio_field->get();
        res.set_content(model ? "{\"radio\":" + radio::radio_spec_json(model->spec()) + "}"
                              : "{\"radio\":null}",
                        "application/json");
    });

    s.Delete("/radio", [radio_field](const httplib::Request &, httplib::Response &res) {
        res.set_content(radio_field->clear() ? "{\"ok\":true,\"cleared\":true}"
                                             : "{\"ok\":true,\"cleared\":false}",
                        "application/json");
    });

    s.Get("/radio/sample", [radio_field](const httplib::Request &req, httplib::Response &res) {
        const auto model = radio_field->get();
        if (!model) {
            reply_error(res, 404, "no radio spec set (PUT /radio first)");
            return;
        }
        std::string error;
        std::optional<double> lat;
        std::optional<double> lon;
        radio::Position p;
        if (!query_number(req, "lat", -90.0, 90.0, lat, error) ||
            !query_number(req, "lon", -180.0, 180.0, lon, error) ||
            !query_altitude(req, p.alt_rel_m, p.alt_amsl_m, error)) {
            reply_error(res, 400, error);
            return;
        }
        if (!lat || !lon) {
            reply_error(res, 400, "lat and lon are required");
            return;
        }
        p.lat_deg = *lat;
        p.lon_deg = *lon;
        res.set_content(radio::sample_json(*model, p), "application/json");
    });

    auto grids_in_flight = std::make_shared<std::atomic<int>>(0);
    s.Get("/radio/grid", [radio_field, grids_in_flight](const httplib::Request &req, httplib::Response &res) {
        const auto model = radio_field->get();
        if (!model) {
            reply_error(res, 404, "no radio spec set (PUT /radio first)");
            return;
        }
        radio::GridRequest grid;
        std::string error;
        std::optional<double> step;
        if (!query_bbox(req, grid, error) || !query_altitude(req, grid.alt_rel_m, grid.alt_amsl_m, error) ||
            !query_number(req, "step_m", 1.0, 100000.0, step, error)) {
            reply_error(res, 400, error);
            return;
        }
        grid.step_m = step.value_or(grid.step_m);
        if (req.has_param("carrier")) {
            const auto index = model->carrier_index(req.get_param_value("carrier"));
            if (!index) {
                reply_error(res, 400,
                            "carrier \"" + req.get_param_value("carrier") + "\" is not in the radio spec");
                return;
            }
            grid.carriers.push_back(*index);
        }
        const size_t layers = grid.carriers.empty() ? model->spec().carriers.size() : 1;
        const auto shape = radio::grid_shape(grid, layers, kMaxGridCells);
        if (!shape) {
            reply_error(
                res, 400,
                "grid too large: more than " + std::to_string(kMaxGridCells) +
                    " cells (rows x columns x carriers); raise step_m, shrink bbox or pick a carrier");
            return;
        }
        // Released on every way out, a throw included.
        struct Slot {
            std::atomic<int> &count;
            ~Slot() { count.fetch_sub(1); }
        };
        const int computing = grids_in_flight->fetch_add(1);
        const Slot slot{*grids_in_flight};
        if (computing >= kMaxGridsInFlight) {
            reply_error(res, 503,
                        "busy: " + std::to_string(kMaxGridsInFlight) + " grids already computing; retry");
            return;
        }
        res.set_content(radio::grid_json(*model, grid, *shape), "application/json");
    });

    // The same two feeds addressed by ArduPilot instance.
    //
    // skysim's own vehicle ids are allocated internally and change when it
    // restarts; the instance is what the gateway assigns and everything else
    // already agrees on. A consumer that can only bake a URL into a config —
    // an ffmpeg command line, say — needs one that stays true.
    auto resolve_instance = [snapshots](const std::string &instance) -> uint32_t {
        if (!snapshots.vehicles) {
            return 0;
        }
        for (const auto &v : snapshots.vehicles()) {
            if (std::to_string(v.instance) == instance) {
                return v.id;
            }
        }
        return 0;
    };

    s.Get(R"(/instances/(\d+)/camera.jpg)", [snapshots, resolve_instance](const httplib::Request &req,
                                                                           httplib::Response &res) {
        const uint32_t id = resolve_instance(req.matches[1].str());
        if (id == 0 || !snapshots.camera_frame) {
            res.status = 404;
            res.set_content("{\"error\":\"no such instance\"}", "application/json");
            return;
        }
        std::vector<uint8_t> jpeg = snapshots.camera_frame(id, 0, kFirstFrameWait).jpeg;
        if (jpeg.empty()) {
            res.status = 404;
            res.set_content("{\"error\":\"no frame\"}", "application/json");
            return;
        }
        res.set_content(reinterpret_cast<const char *>(jpeg.data()), jpeg.size(), "image/jpeg");
    });

    s.Get(R"(/instances/(\d+)/camera.mjpg)", [snapshots, resolve_instance](const httplib::Request &req,
                                                                            httplib::Response &res) {
        const std::string instance = req.matches[1].str();
        if (!snapshots.camera_frame) {
            res.status = 404;
            res.set_content("{\"error\":\"camera disabled\"}", "application/json");
            return;
        }
        if (!snapshots.mjpeg) {
            reply_error(res, 404,
                        "mjpeg disabled (--camera-mjpeg 0): the camera is on RTSP at /instances/" + instance +
                            "/main.264");
            return;
        }
        res.set_chunked_content_provider(
            "multipart/x-mixed-replace; boundary=skysimframe",
            [snapshots, resolve_instance, instance, last_seq = uint64_t{0}](size_t,
                                                                            httplib::DataSink &sink) mutable {
                // Re-resolved per frame, so a skysim restart mid-recording picks
                // the vehicle back up instead of streaming nothing forever.
                stream_next_frame(snapshots, resolve_instance(instance), last_seq, sink);
                return true;
            });
    });

    // One frame, for a poll-based viewer or a quick look with curl.
    s.Get(R"(/vehicles/(\d+)/camera.jpg)", [snapshots](const httplib::Request &req, httplib::Response &res) {
        if (!snapshots.camera_frame) {
            res.status = 404;
            res.set_content("{\"error\":\"camera disabled\"}", "application/json");
            return;
        }
        // strtoul, not stoul: the route regex accepts any number of digits, and
        // stoul throws on a value too big for the type. Saturating is the same
        // answer as "no such vehicle" without the exception.
        const auto id = static_cast<uint32_t>(std::strtoul(req.matches[1].str().c_str(), nullptr, 10));
        std::vector<uint8_t> jpeg = snapshots.camera_frame(id, 0, kFirstFrameWait).jpeg;
        if (jpeg.empty()) {
            res.status = 404;
            res.set_content("{\"error\":\"no frame\"}", "application/json");
            return;
        }
        res.set_content(reinterpret_cast<const char *>(jpeg.data()), jpeg.size(), "image/jpeg");
    });

    // MJPEG, which is what the video pipeline consumes.
    //
    // A chunked multipart stream rather than a socket of raw frames because it
    // crosses a container boundary and GStreamer, ffmpeg and a plain browser tab
    // can all open it without agreeing on anything first.
    s.Get(R"(/vehicles/(\d+)/camera.mjpg)", [snapshots](const httplib::Request &req, httplib::Response &res) {
        if (!snapshots.camera_frame) {
            res.status = 404;
            res.set_content("{\"error\":\"camera disabled\"}", "application/json");
            return;
        }
        if (!snapshots.mjpeg) {
            reply_error(
                res, 404,
                "mjpeg disabled (--camera-mjpeg 0): the camera is on RTSP at /instances/<n>/main.264");
            return;
        }
        const auto id = static_cast<uint32_t>(std::strtoul(req.matches[1].str().c_str(), nullptr, 10));
        res.set_chunked_content_provider(
            "multipart/x-mixed-replace; boundary=skysimframe",
            [snapshots, id, last_seq = uint64_t{0}](size_t /*offset*/, httplib::DataSink &sink) mutable {
                stream_next_frame(snapshots, id, last_seq, sink);
                return true;
            });
    });

    // The H.264 encoder behind rtsp://<host>:8554/instances/<n>/main.264 (TASK-2561): read its
    // settings, change some of them (clamped; the reply is what was applied), or force an IDR.
    // By instance only, like the RTSP URL.
    const auto encoder_route = [snapshots](const httplib::Request &req, httplib::Response &res, bool post) {
        if (!snapshots.camera_encoder || !snapshots.vehicles) {
            reply_error(res, 404, "rtsp video disabled");
            return;
        }
        EncoderPatch patch;
        std::string error;
        if (post && !parse_encoder_patch(req.body, patch, error)) {
            reply_error(res, 400, error);
            return;
        }
        const std::vector<VehicleInfo> vehicles = snapshots.vehicles();
        const VehicleInfo *v = find_instance(vehicles, req.matches[1].str());
        if (v == nullptr) {
            reply_error(res, 404, "no such instance");
            return;
        }
        res.set_content(encoder_json(snapshots.camera_encoder(*v, post ? &patch : nullptr)),
                        "application/json");
    };
    s.Get(R"(/instances/(\d+)/camera/encoder)",
          [encoder_route](const httplib::Request &req, httplib::Response &res) {
              encoder_route(req, res, false);
          });
    s.Post(R"(/instances/(\d+)/camera/encoder)",
           [encoder_route](const httplib::Request &req, httplib::Response &res) {
               encoder_route(req, res, true);
           });
    s.Post(R"(/instances/(\d+)/camera/keyframe)",
           [snapshots](const httplib::Request &req, httplib::Response &res) {
               if (!snapshots.camera_keyframe || !snapshots.vehicles) {
                   reply_error(res, 404, "rtsp video disabled");
                   return;
               }
               const std::vector<VehicleInfo> vehicles = snapshots.vehicles();
               const VehicleInfo *v = find_instance(vehicles, req.matches[1].str());
               if (v == nullptr) {
                   reply_error(res, 404, "no such instance");
                   return;
               }
               const size_t clients = snapshots.camera_keyframe(*v);
               res.set_content("{\"keyframe\":true,\"clients\":" + std::to_string(clients) + "}",
                               "application/json");
           });

    // Survey still camera (docs/SKYHUB_INTEGRATION.md, "Survey stills"): one frame rendered on
    // demand, at the size, field of view and gimbal angle asked for, from the vehicle's pose in
    // the latest snapshot. The same frame by instance or by skysim id.
    //
    // The vehicle is found in one snapshot copy and handed over whole, so the picture and its
    // pose headers come from the same tick; nothing here waits on the tick thread.
    const auto still_route = [snapshots, home](const httplib::Request &req, httplib::Response &res,
                                               bool by_instance) {
        if (!snapshots.still || !snapshots.vehicles) {
            reply_error(res, 404, "still camera disabled");
            return;
        }
        render::StillRequest request;
        std::string error;
        if (!parse_still_query(req, request, error)) {
            reply_error(res, 400, error);
            return;
        }
        // The whole number in the path, or no match at all: the regex takes any number of
        // digits, and one too big for the id type must be "no such", not wrap onto another
        // vehicle. Leading zeros name the same vehicle.
        const std::string key = req.matches[1].str();
        uint64_t number = 0;
        const auto parsed = std::from_chars(key.data(), key.data() + key.size(), number);
        const bool valid = parsed.ec == std::errc() && parsed.ptr == key.data() + key.size() &&
                           number <= (by_instance ? 0x7FFFFFFFULL : 0xFFFFFFFFULL);
        for (const auto &v : valid ? snapshots.vehicles() : std::vector<VehicleInfo>{}) {
            if (by_instance ? static_cast<uint64_t>(v.instance) == number : v.id == number) {
                reply_still(snapshots, home, v, request, res);
                return;
            }
        }
        reply_error(res, 404, by_instance ? "no such instance" : "no such vehicle");
    };
    s.Get(
        R"(/instances/(\d+)/still.jpg)",
        [still_route](const httplib::Request &req, httplib::Response &res) { still_route(req, res, true); });
    s.Get(R"(/vehicles/(\d+)/still.jpg)", [still_route](const httplib::Request &req, httplib::Response &res) {
        still_route(req, res, false);
    });

    s.Get("/metrics", [snapshots](const httplib::Request &, httplib::Response &res) {
        const MetricsInfo m = snapshots.metrics();
        // The pilot camera's achieved rate and per-frame cost, or null when it is off.
        char camera[384] = "null";
        if (m.camera.enabled) {
            const CameraMetrics &c = m.camera;
            std::snprintf(camera, sizeof(camera),
                          "{\"target_fps\":%.1f,\"fps\":%.2f,\"watched\":%zu,\"render_ms\":%.2f,"
                          "\"render_ms_max\":%.2f,\"encode_ms\":%.2f,\"threads\":%d,\"frames_drawn\":%llu,"
                          "\"frames_reused\":%llu}",
                          c.target_fps, c.fps, c.watched, c.render_ms, c.render_ms_max, c.encode_ms, c.threads,
                          static_cast<unsigned long long>(c.frames_drawn),
                          static_cast<unsigned long long>(c.frames_reused));
        }
        // The H.264/RTSP streams, or null when RTSP video is off.
        char video[384] = "null";
        if (m.video.enabled) {
            const VideoMetrics &v = m.video;
            std::snprintf(
                video, sizeof(video),
                "{\"rtsp_port\":%d,\"streams\":%zu,\"clients\":%zu,\"threads\":%d,\"encode_ms\":%.2f,"
                "\"frames_encoded\":%llu,\"frames_dropped\":%llu,\"keyframes\":%llu,\"bytes_sent\":%llu}",
                v.rtsp_port, v.streams, v.clients, v.threads, v.encode_ms,
                static_cast<unsigned long long>(v.frames_encoded),
                static_cast<unsigned long long>(v.frames_dropped),
                static_cast<unsigned long long>(v.keyframes), static_cast<unsigned long long>(v.bytes_sent));
        }
        char buf[1536];
        std::snprintf(buf, sizeof(buf),
                      "{\"tick\":%llu,\"sim_time_s\":%.3f,\"tick_p50_us\":%.1f,"
                      "\"tick_p99_us\":%.1f,\"straggler_events\":%llu,\"freezes\":%llu,"
                      "\"vehicles\":%zu,\"resident_tiles\":%zu,\"camera\":%s,\"video\":%s}",
                      static_cast<unsigned long long>(m.tick), m.sim_time_s, m.tick_p50_us, m.tick_p99_us,
                      static_cast<unsigned long long>(m.straggler_events),
                      static_cast<unsigned long long>(m.freezes), m.vehicles, m.resident_tiles, camera,
                      video);
        res.set_content(buf, "application/json");
    });

    if (!s.bind_to_port(bind_addr, port)) {
        throw std::runtime_error("ControlServer: cannot bind " + bind_addr + ":" + std::to_string(port));
    }
    impl_->thread = std::thread([this] { impl_->server.listen_after_bind(); });
    // Wait for the listen loop to actually start: stop() delivered before is_running()
    // would be lost and ~ControlServer would join() forever (httplib stop/listen race).
    for (int i = 0; i < 1000 && !s.is_running(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::printf("skysim: control plane on http://%s:%d\n", bind_addr.c_str(), port);
}

ControlServer::~ControlServer() {
    impl_->server.stop();
    if (impl_->thread.joinable()) {
        impl_->thread.join();
    }
}

} // namespace skysim::api
