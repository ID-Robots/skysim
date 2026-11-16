// The dock marker's control routes (TASK-2562) — GET/POST/DELETE /instances/<n>/camera/dock and
// GET /instances/<n>/camera/intrinsics — over the real HTTP stack, against canned snapshots and a
// real DockBoard wired the way main.cpp wires it.
#include <httplib.h>

#include <cmath>
#include <cstdio>
#include <optional>
#include <string>

#include "api/control_server.h"
#include "core/geo.h"
#include "radio/json.h"
#include "render/dock_marker.h"

namespace {

int g_failures = 0;

#define CHECK(cond)                                                                                          \
    do {                                                                                                     \
        if (!(cond)) {                                                                                       \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                      \
            ++g_failures;                                                                                    \
        }                                                                                                    \
    } while (0)

constexpr int kPort = 18670;     // camera, stills and docks on, a geo anchor
constexpr int kBarePort = 18671; // none of them, no geo anchor

using namespace skysim::api;
namespace json = skysim::radio::json;

VehicleInfo vehicle() {
    VehicleInfo v;
    v.id = 3;
    v.instance = 7;
    v.pos_ned[0] = 0.25;
    v.pos_ned[1] = 10.0;
    v.pos_ned[2] = -5.0;
    v.sim_time_s = 12.5;
    v.gimbal = true;
    v.gimbal_pitch_rad = -M_PI / 2;
    return v;
}

std::optional<json::Value> body(const httplib::Result &r, int status) {
    if (!r || r->status != status) {
        std::printf("  got %d: %s\n", r ? r->status : -1, r ? r->body.c_str() : "(no reply)");
        return std::nullopt;
    }
    std::string error;
    return json::parse(r->body, error);
}

double num(const json::Value &v, const char *a, const char *b = nullptr, int index = -1) {
    const json::Value *x = v.find(a);
    if (x != nullptr && b != nullptr) {
        x = x->find(b);
    }
    if (x != nullptr && index >= 0) {
        x = x->is_array() && static_cast<size_t>(index) < x->items.size()
                ? &x->items[static_cast<size_t>(index)]
                : nullptr;
    }
    return x != nullptr && x->is_number() ? x->number : NAN;
}

std::string str(const json::Value &v, const char *key) {
    const json::Value *x = v.find(key);
    return x != nullptr && x->is_string() ? x->text : "";
}

// Corner k of square `which` ("outer"/"inner"), axis 0 north, 1 east.
double corner(const json::Value &v, const char *which, int k, int axis) {
    const json::Value *sq = v.find(which);
    const json::Value *corners = sq != nullptr ? sq->find("corners_ned") : nullptr;
    if (corners == nullptr || !corners->is_array() || corners->items.size() != 4) {
        return NAN;
    }
    const json::Value &c = corners->items[static_cast<size_t>(k)];
    return c.is_array() && c.items.size() == 3 ? c.items[static_cast<size_t>(axis)].number : NAN;
}

bool near(double a, double b, double tol = 1e-6) { return std::fabs(a - b) <= tol; }

} // namespace

int main() {
    CommandQueue queue;
    auto board = std::make_shared<skysim::render::DockBoard>(skysim::render::DockMarkerSpec{});
    board->add(3, 0.0, 10.0); // vehicle 3 spawned 10 m east of the origin

    ControlServer::Snapshots snaps;
    snaps.vehicles = [] { return std::vector<VehicleInfo>{vehicle()}; };
    snaps.metrics = [] { return MetricsInfo{}; };
    snaps.still = [](const VehicleInfo &, const skysim::render::StillRequest &) {
        return skysim::render::StillFrame{};
    };
    snaps.camera_intrinsics.enabled = true;
    snaps.camera_intrinsics.width = 1920;
    snaps.camera_intrinsics.height = 1080;
    snaps.camera_intrinsics.hfov_deg = 81.0;
    snaps.camera_intrinsics.fps = 10.0;
    snaps.camera_intrinsics.stream_width = 1920;
    snaps.camera_intrinsics.stream_height = 1080;
    bool drop = false; // the vehicle's dock gone (it despawned between the lookup and the call)
    snaps.dock = [&board, &drop](const VehicleInfo &v, const skysim::render::DockPatch *patch,
                                 bool reset) -> std::optional<DockState> {
        const auto tex = skysim::render::dock_marker_texture(board->spec());
        const auto e =
            drop ? std::nullopt
                 : (reset ? board->reset(v.id) : (patch ? board->update(v.id, *patch) : board->get(v.id)));
        if (!tex || !e) {
            return std::nullopt;
        }
        DockState s;
        s.pose = e->pose;
        s.home = e->home;
        s.at_home = e->at_home;
        s.layout = tex->layout();
        return s;
    };
    ControlServer::RadioConfig radio;
    radio.home = skysim::radio::GeoAnchor{42.140389, 24.764549, 160.0};
    ControlServer server("127.0.0.1", kPort, queue, snaps, radio);
    httplib::Client c("127.0.0.1", kPort);

    // --- intrinsics --------------------------------------------------------------------------
    {
        const auto v = body(c.Get("/instances/7/camera/intrinsics"), 200);
        CHECK(v.has_value());
        if (v) {
            const double f = 960.0 / std::tan(40.5 * M_PI / 180.0);
            CHECK(str(*v, "camera") == "pilot");
            CHECK(num(*v, "width") == 1920 && num(*v, "height") == 1080 && num(*v, "hfov_deg") == 81.0);
            CHECK(near(num(*v, "fx"), f, 1e-9) && near(num(*v, "fy"), f, 1e-9));
            CHECK(num(*v, "cx") == 959.5 && num(*v, "cy") == 539.5);
            CHECK(near(num(*v, "vfov_deg"), 2.0 * std::atan(540.0 / f) * 180.0 / M_PI, 1e-9));
            const json::Value *m = v->find("camera_matrix");
            CHECK(m && m->items.size() == 3 && m->items[0].items.size() == 3 &&
                  near(m->items[0].items[0].number, f) && m->items[0].items[2].number == 959.5 &&
                  m->items[1].items[2].number == 539.5 && m->items[2].items[2].number == 1.0);
            const json::Value *d = v->find("distortion");
            CHECK(d && d->items.size() == 5 && d->items[0].number == 0.0);
            CHECK(num(*v, "fps") == 10.0);
            CHECK(num(*v, "stream", "width") == 1920);
            const json::Value *stream = v->find("stream");
            CHECK(stream && str(*stream, "path") == "/instances/7/main.264");
            CHECK(near(num(*v, "gimbal", "pitch_deg"), -90.0, 1e-9));
        }
        const auto s = body(c.Get("/instances/7/camera/intrinsics?w=640&h=480&hfov=60"), 200);
        CHECK(s && str(*s, "camera") == "still" && near(num(*s, "fx"), 320.0 / std::tan(M_PI / 6), 1e-9) &&
              num(*s, "cx") == 319.5 && num(*s, "cy") == 239.5 && !s->find("stream"));
        const auto sd = body(c.Get("/instances/7/camera/intrinsics?w=800"), 200); // hfov: the still default
        CHECK(sd && num(*sd, "width") == 800 && num(*sd, "height") == 1200 && num(*sd, "hfov_deg") == 73.7);
        CHECK(body(c.Get("/instances/7/camera/intrinsics?w=10"), 400));
        CHECK(body(c.Get("/instances/7/camera/intrinsics?hfov=abc"), 400));
        CHECK(body(c.Get("/instances/8/camera/intrinsics"), 404));
    }

    // --- dock: as spawned --------------------------------------------------------------------
    {
        const auto v = body(c.Get("/instances/7/camera/dock"), 200);
        CHECK(v.has_value());
        if (v) {
            CHECK(str(*v, "source") == "home" && num(*v, "instance") == 7 && num(*v, "vehicle") == 3);
            const json::Value *m = v->find("marker");
            CHECK(m && str(*m, "dictionary") == "DICT_7X7_250" && num(*m, "outer_id") == 228 &&
                  num(*m, "inner_id") == 11 && num(*m, "outer_size_m") == 1.0 &&
                  num(*m, "inner_size_m") == 0.09);
            CHECK(near(num(*v, "marker", "cell_size_m"), 437.0 / 3937.0, 1e-12));
            CHECK(near(num(*v, "marker", "plate_size_m", 0), 4093.0 / 3937.0, 1e-12));
            CHECK(num(*v, "pose", "north_m") == 0.0 && num(*v, "pose", "east_m") == 10.0 &&
                  num(*v, "pose", "yaw_deg") == 0.0);
            const auto geo = skysim::core::geo_from_ned({42.140389, 24.764549, 160.0}, 0.0, 10.0, 0.0);
            CHECK(near(num(*v, "pose", "lat"), geo.lat_deg, 1e-9) &&
                  near(num(*v, "pose", "lon"), geo.lon_deg, 1e-9));
            CHECK(num(*v, "pose", "alt_amsl_m") == 160.0);
            CHECK(num(*v, "home", "east_m") == 10.0);
            // Upright, north up: the outer marker's top-left corner is its north-west one.
            CHECK(near(corner(*v, "outer", 0, 0), 0.5) && near(corner(*v, "outer", 0, 1), 9.5));
            CHECK(near(corner(*v, "outer", 2, 0), -0.5) && near(corner(*v, "outer", 2, 1), 10.5));
            CHECK(near(num(*v, "inner", "size_m"), 0.09));
            CHECK(near(num(*v, "inner", "centre_ned", 0), 2.0 / 3937.0, 1e-12) &&
                  near(num(*v, "inner", "centre_ned", 1), 10.0 - 2.0 / 3937.0, 1e-12));
            // The vehicle 0.25 m north of it, 5 m up.
            CHECK(near(num(*v, "vehicle_pose", "horizontal_error_m"), 0.25) &&
                  num(*v, "vehicle_pose", "height_above_marker_m") == 5.0 &&
                  near(num(*v, "vehicle_pose", "offset_ned", 0), 0.25) &&
                  num(*v, "vehicle_pose", "sim_time_s") == 12.5);
        }
        CHECK(body(c.Get("/instances/8/camera/dock"), 404));
    }

    // --- dock: moved, by NED and by lat/lon, then back home -----------------------------------
    {
        const auto v = body(c.Post("/instances/7/camera/dock", R"({"north_m":3,"east_m":-2,"yaw_deg":90})",
                                   "application/json"),
                            200);
        CHECK(v && str(*v, "source") == "set" && num(*v, "pose", "north_m") == 3.0 &&
              num(*v, "pose", "east_m") == -2.0 && near(num(*v, "pose", "yaw_deg"), 90.0, 1e-9));
        // Turned to face east: its top-left corner is now the north-east one.
        CHECK(v && near(corner(*v, "outer", 0, 0), 3.5) && near(corner(*v, "outer", 0, 1), -1.5));
        CHECK(board->get(3)->pose.east_m == -2.0); // what the cameras draw moved too

        const auto yaw_only =
            body(c.Post("/instances/7/camera/dock", R"({"yaw_deg":-45})", "application/json"), 200);
        CHECK(yaw_only && num(*yaw_only, "pose", "north_m") == 3.0 &&
              near(num(*yaw_only, "pose", "yaw_deg"), -45.0));

        const auto geo = skysim::core::geo_from_ned({42.140389, 24.764549, 160.0}, 12.0, -34.0, 0.0);
        char by_geo[160];
        std::snprintf(by_geo, sizeof(by_geo), R"({"lat":%.10f,"lon":%.10f})", geo.lat_deg, geo.lon_deg);
        const auto g = body(c.Post("/instances/7/camera/dock", by_geo, "application/json"), 200);
        CHECK(g && near(num(*g, "pose", "north_m"), 12.0, 1e-3) &&
              near(num(*g, "pose", "east_m"), -34.0, 1e-3) && near(num(*g, "pose", "yaw_deg"), -45.0));

        // Nothing named, nothing changed.
        const auto same = body(c.Post("/instances/7/camera/dock", "{}", "application/json"), 200);
        CHECK(same && near(num(*same, "pose", "north_m"), 12.0, 1e-3) && str(*same, "source") == "set");
        CHECK(body(c.Post("/instances/7/camera/dock", "", "application/json"), 200));

        // A GET's whole pose sent back with only its yaw changed: lat/lon and north/east agree.
        const auto got = c.Get("/instances/7/camera/dock");
        std::string error;
        const auto now = got ? json::parse(got->body, error) : std::nullopt;
        CHECK(now.has_value());
        if (now) {
            char round_trip[256];
            std::snprintf(round_trip, sizeof(round_trip),
                          R"({"north_m":%.9f,"east_m":%.9f,"down_m":0,"yaw_deg":30,"lat":%.9f,"lon":%.9f})",
                          num(*now, "pose", "north_m"), num(*now, "pose", "east_m"), num(*now, "pose", "lat"),
                          num(*now, "pose", "lon"));
            const auto rt = body(c.Post("/instances/7/camera/dock", round_trip, "application/json"), 200);
            CHECK(rt && near(num(*rt, "pose", "north_m"), num(*now, "pose", "north_m"), 1e-6) &&
                  near(num(*rt, "pose", "east_m"), num(*now, "pose", "east_m"), 1e-6) &&
                  near(num(*rt, "pose", "yaw_deg"), 30.0, 1e-9));
            std::snprintf(round_trip, sizeof(round_trip), R"({"north_m":%.9f,"lat":%.9f,"lon":%.9f})",
                          num(*now, "pose", "north_m") + 1.0, num(*now, "pose", "lat"),
                          num(*now, "pose", "lon"));
            const auto apart = c.Post("/instances/7/camera/dock", round_trip, "application/json");
            CHECK(apart && apart->status == 400 && apart->body.find("apart") != std::string::npos);
        }

        // Refused bodies change nothing: the whole pose is as it was, after each one.
        const skysim::render::DockPose before = board->get(3)->pose;
        for (const char *bad :
             {R"({"lat":42.1})", R"({"lat":42.1,"lon":24.7,"north_m":1})", R"({"yaw_deg":400})",
              R"({"north_m":30000})", R"({"east_m":-30000})", R"({"lat":95,"lon":24})",
              R"({"lat":42,"lon":200})", R"({"north_m":"3"})", R"([1,2])", R"({"north_m":)"}) {
            CHECK(body(c.Post("/instances/7/camera/dock", bad, "application/json"), 400));
            const skysim::render::DockPose after = board->get(3)->pose;
            CHECK(after.north_m == before.north_m && after.east_m == before.east_m &&
                  after.yaw_rad == before.yaw_rad);
        }

        const auto home = body(c.Delete("/instances/7/camera/dock"), 200);
        CHECK(home && str(*home, "source") == "home" && num(*home, "pose", "north_m") == 0.0 &&
              num(*home, "pose", "east_m") == 10.0 && num(*home, "pose", "yaw_deg") == 0.0);
        CHECK(body(c.Post("/instances/8/camera/dock", "{}", "application/json"), 404));
        CHECK(body(c.Delete("/instances/8/camera/dock"), 404));

        drop = true; // the vehicle is in the snapshot but its dock has gone with it
        CHECK(body(c.Get("/instances/7/camera/dock"), 404));
        CHECK(body(c.Post("/instances/7/camera/dock", R"({"north_m":1})", "application/json"), 404));
        drop = false;
    }

    // --- a server with no camera, no stills, no docks and no geo anchor -----------------------
    {
        ControlServer::Snapshots bare;
        bare.vehicles = [] { return std::vector<VehicleInfo>{vehicle()}; };
        bare.metrics = [] { return MetricsInfo{}; };
        ControlServer off("127.0.0.1", kBarePort, queue, bare);
        httplib::Client b("127.0.0.1", kBarePort);
        const auto r = b.Get("/instances/7/camera/intrinsics");
        CHECK(r && r->status == 404 && r->body.find("camera disabled") != std::string::npos);
        CHECK(body(b.Get("/instances/7/camera/intrinsics?w=640"), 404)); // no still camera
        const auto d = b.Get("/instances/7/camera/dock");
        CHECK(d && d->status == 404 && d->body.find("dock markers disabled") != std::string::npos);

        // No fleet to find the instance in.
        ControlServer::Snapshots none;
        none.metrics = [] { return MetricsInfo{}; };
        ControlServer blind("127.0.0.1", kBarePort + 1, queue, none);
        httplib::Client n("127.0.0.1", kBarePort + 1);
        CHECK(body(n.Get("/instances/7/camera/intrinsics"), 404)); // no vehicles at all
    }
    {
        // Docks without a geo anchor: no lat/lon either way.
        ControlServer::Snapshots s = snaps;
        ControlServer unanchored("127.0.0.1", kBarePort + 2, queue, s);
        httplib::Client u("127.0.0.1", kBarePort + 2);
        const auto v = body(u.Get("/instances/7/camera/dock"), 200);
        CHECK(v && !v->find("pose")->find("lat"));
        CHECK(
            body(u.Post("/instances/7/camera/dock", R"({"lat":42.1,"lon":24.7})", "application/json"), 400));
        CHECK(body(u.Post("/instances/7/camera/dock", R"({"north_m":1.5})", "application/json"), 200));
    }

    {
        // Near a pole: a dock past it has no lat/lon to give (the latitude clamps at 90), so its
        // pose carries none, says why, and still posts back.
        ControlServer::RadioConfig polar;
        polar.home = skysim::radio::GeoAnchor{89.99, 0.0, 0.0};
        ControlServer arctic("127.0.0.1", kBarePort + 3, queue, snaps, polar);
        httplib::Client a("127.0.0.1", kBarePort + 3);
        const auto far = body(
            a.Post("/instances/7/camera/dock", R"({"north_m":2000,"east_m":0})", "application/json"), 200);
        CHECK(far && !far->find("pose")->find("lat") &&
              str(*far->find("pose"), "geo").find("pole") != std::string::npos);
        CHECK(body(a.Post("/instances/7/camera/dock", R"({"north_m":2000,"east_m":0,"yaw_deg":10})",
                          "application/json"),
                   200));
        const auto near_pole = body(
            a.Post("/instances/7/camera/dock", R"({"north_m":100,"east_m":0})", "application/json"), 200);
        CHECK(near_pole &&
              near_pole->find("pose")->find("lat") != nullptr); // short of the pole: lat/lon as usual
        CHECK(body(a.Delete("/instances/7/camera/dock"), 200));
    }

    if (g_failures != 0) {
        std::printf("%d failure(s)\n", g_failures);
        return 1;
    }
    std::printf("test_dock_api: all passed\n");
    return 0;
}
