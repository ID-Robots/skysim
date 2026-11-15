// The camera's H.264 control routes (TASK-2561) — GET/POST /instances/<n>/camera/encoder and
// POST /instances/<n>/camera/keyframe — the video block of /metrics, the --camera-mjpeg switch and
// the MJPEG part timestamps, over the real HTTP stack against canned snapshots.
#include <httplib.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <string>

#include "api/control_server.h"
#include "video/video_service.h"

namespace {

int g_failures = 0;

#define CHECK(cond)                                                                                          \
    do {                                                                                                     \
        if (!(cond)) {                                                                                       \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                      \
            ++g_failures;                                                                                    \
        }                                                                                                    \
    } while (0)

constexpr int kPort = 18660;         // RTSP video on, MJPEG off
constexpr int kDisabledPort = 18661; // no RTSP video, MJPEG on

using namespace skysim::api;

std::vector<VehicleInfo> fleet() {
    VehicleInfo a;
    a.id = 1;
    a.instance = 3;
    VehicleInfo b;
    b.id = 2;
    b.instance = 4;
    return {a, b};
}

bool has(const httplib::Result &r, const std::string &text) {
    return r && r->body.find(text) != std::string::npos;
}

} // namespace

int main() {
    CommandQueue queue;

    // The control routes in front of a real VideoService, wired the way main.cpp wires it.
    skysim::video::VideoService::Config vcfg;
    vcfg.width = 256;
    vcfg.height = 144;
    vcfg.max_fps = 10.0;
    vcfg.defaults.fps = 10.0;
    vcfg.rtsp.bind = "127.0.0.1";
    vcfg.rtsp.port = 0;
    skysim::video::VideoService::Hooks vhooks;
    vhooks.resolve = [](int instance) -> uint32_t { return instance == 3 ? 1 : instance == 4 ? 2 : 0; };
    skysim::video::VideoService video(vcfg, vhooks);

    std::atomic<int> keyframes{0};
    ControlServer::Snapshots snaps;
    snaps.vehicles = [] { return fleet(); };
    snaps.metrics = [&video] {
        MetricsInfo m;
        const auto v = video.stats();
        m.video.enabled = true;
        m.video.rtsp_port = v.port;
        m.video.threads = v.threads;
        m.video.frames_encoded = 12;
        m.video.keyframes = 2;
        m.video.bytes_sent = 3456;
        return m;
    };
    snaps.camera_frame = [](uint32_t id, uint64_t, std::chrono::milliseconds) {
        return id == 1 ? skysim::render::FrameStore::Frame{{0xFF, 0xD8, 0xFF, 0xD9}, 1.5, 1, 1700000000123456}
                       : skysim::render::FrameStore::Frame{};
    };
    snaps.camera_encoder = [&video](const VehicleInfo &v, const EncoderPatch *patch) {
        skysim::video::EncoderPatch p;
        if (patch != nullptr) {
            p.bitrate_kbps = patch->bitrate_kbps;
            p.fps = patch->fps;
            p.keyframe_interval_s = patch->keyframe_interval_s;
        }
        const auto st = video.encoder(v.instance, v.id, patch != nullptr ? &p : nullptr);
        EncoderState out;
        out.bitrate_kbps = st.settings.bitrate_kbps;
        out.fps = st.settings.fps;
        out.keyframe_interval_s = st.settings.keyframe_interval_s;
        out.width = st.width;
        out.height = st.height;
        out.max_fps = st.max_fps;
        out.clients = st.clients;
        return out;
    };
    snaps.camera_keyframe = [&video, &keyframes](const VehicleInfo &v) {
        ++keyframes;
        return video.request_keyframe(v.instance, v.id);
    };
    snaps.mjpeg = false;

    {
        ControlServer server("127.0.0.1", kPort, queue, snaps);
        httplib::Client http("127.0.0.1", kPort);
        http.set_read_timeout(5, 0);

        auto r = http.Get("/instances/3/camera/encoder");
        CHECK(r && r->status == 200);
        CHECK(
            r &&
            r->body ==
                R"({"bitrate_kbps":1200,"fps":10,"keyframe_interval_s":1,"width":256,"height":144,"max_fps":10,"clients":0})");
        CHECK(r && r->get_header_value("Content-Type") == "application/json");
        r = http.Get("/instances/9/camera/encoder");
        CHECK(r && r->status == 404 && has(r, "no such instance"));
        r = http.Get("/instances/99999999999999999999/camera/encoder");
        CHECK(r && r->status == 404);

        // POST changes only what it names, clamps, and answers with what was applied.
        r = http.Post("/instances/3/camera/encoder", R"({"bitrate_kbps":800})", "application/json");
        CHECK(r && r->status == 200 && has(r, R"("bitrate_kbps":800,"fps":10,"keyframe_interval_s":1,)"));
        r = http.Post("/instances/3/camera/encoder", R"({"fps":60,"keyframe_interval_s":0.5})",
                      "application/json");
        CHECK(r && r->status == 200 && has(r, R"("bitrate_kbps":800,"fps":10,"keyframe_interval_s":0.5,)"));
        r = http.Post("/instances/3/camera/encoder", R"({"bitrate_kbps":1,"fps":0,"keyframe_interval_s":99})",
                      "application/json");
        CHECK(r && r->status == 200 && has(r, R"("bitrate_kbps":64,"fps":1,"keyframe_interval_s":10,)"));
        // What GET returned, sent back with one field changed: the read-only fields are ignored.
        r = http.Post(
            "/instances/3/camera/encoder",
            R"({"bitrate_kbps":2500.4,"fps":7.5,"keyframe_interval_s":2,"width":1,"height":1,"max_fps":99,"clients":5})",
            "application/json");
        CHECK(
            r && r->status == 200 &&
            r->body ==
                R"({"bitrate_kbps":2500,"fps":7.5,"keyframe_interval_s":2,"width":256,"height":144,"max_fps":10,"clients":0})");
        r = http.Post("/instances/3/camera/encoder", "", "application/json");
        CHECK(r && r->status == 200 && has(r, R"("bitrate_kbps":2500,"fps":7.5)"));
        r = http.Get("/instances/3/camera/encoder");
        CHECK(r && has(r, R"("bitrate_kbps":2500,"fps":7.5,"keyframe_interval_s":2,)"));
        // Settings are per instance.
        r = http.Get("/instances/4/camera/encoder");
        CHECK(r && has(r, R"("bitrate_kbps":1200,"fps":10,"keyframe_interval_s":1,)"));

        // Bad bodies are 400 with the reason, and change nothing.
        r = http.Post("/instances/3/camera/encoder", "not json", "application/json");
        CHECK(r && r->status == 400 && has(r, "invalid JSON"));
        r = http.Post("/instances/3/camera/encoder", "[1,2]", "application/json");
        CHECK(r && r->status == 400 && has(r, "body must be a JSON object"));
        r = http.Post("/instances/3/camera/encoder", R"({"fps":"fast"})", "application/json");
        CHECK(r && r->status == 400 && has(r, "fps must be a number"));
        r = http.Post("/instances/3/camera/encoder", R"({"bitrate_kbps":null})", "application/json");
        CHECK(r && r->status == 400 && has(r, "bitrate_kbps must be a number"));
        r = http.Post("/instances/9/camera/encoder", R"({"fps":5})", "application/json");
        CHECK(r && r->status == 404);
        r = http.Get("/instances/3/camera/encoder");
        CHECK(r && has(r, R"("bitrate_kbps":2500,"fps":7.5,"keyframe_interval_s":2,)"));

        // Keyframe on demand.
        r = http.Post("/instances/4/camera/keyframe", "", "application/json");
        CHECK(r && r->status == 200 && r->body == R"({"keyframe":true,"clients":0})");
        CHECK(keyframes.load() == 1);
        r = http.Post("/instances/5/camera/keyframe", "", "application/json");
        CHECK(r && r->status == 404 && keyframes.load() == 1);

        // The video block of /metrics.
        r = http.Get("/metrics");
        CHECK(r && r->status == 200);
        CHECK(
            has(r, "\"video\":{\"rtsp_port\":" + std::to_string(video.port()) +
                       ",\"streams\":0,\"clients\":0,\"threads\":1,\"encode_ms\":0.00,\"frames_encoded\":12,"
                       "\"frames_dropped\":0,\"keyframes\":2,\"bytes_sent\":3456}"));

        // MJPEG switched off: both stream routes say where the camera went; camera.jpg stays.
        r = http.Get("/instances/3/camera.mjpg");
        CHECK(r && r->status == 404 && has(r, "mjpeg disabled") && has(r, "/instances/3/main.264"));
        r = http.Get("/vehicles/1/camera.mjpg");
        CHECK(r && r->status == 404 && has(r, "mjpeg disabled"));
        r = http.Get("/instances/3/camera.jpg");
        CHECK(r && r->status == 200 && r->body.size() == 4);
    }

    // No RTSP video: the routes are 404, /metrics says null; MJPEG parts carry their timestamps.
    {
        ControlServer::Snapshots off;
        off.vehicles = [] { return fleet(); };
        off.metrics = [] { return MetricsInfo{}; };
        off.camera_frame = snaps.camera_frame;
        ControlServer server("127.0.0.1", kDisabledPort, queue, off);
        httplib::Client http("127.0.0.1", kDisabledPort);
        http.set_read_timeout(5, 0);
        auto r = http.Get("/instances/3/camera/encoder");
        CHECK(r && r->status == 404 && has(r, "rtsp video disabled"));
        r = http.Post("/instances/3/camera/encoder", R"({"fps":5})", "application/json");
        CHECK(r && r->status == 404 && has(r, "rtsp video disabled"));
        r = http.Post("/instances/3/camera/keyframe", "", "application/json");
        CHECK(r && r->status == 404 && has(r, "rtsp video disabled"));
        r = http.Get("/metrics");
        CHECK(has(r, "\"video\":null"));

        std::string part;
        http.Get("/instances/3/camera.mjpg", [&](const char *data, size_t len) {
            part.append(data, len);
            return part.find("\r\n\r\n") == std::string::npos;
        });
        CHECK(part.find("--skysimframe\r\nContent-Type: image/jpeg\r\nContent-Length: 4\r\n") == 0);
        CHECK(part.find("X-Skysim-Time-Usec: 1500000\r\n") != std::string::npos);
        CHECK(part.find("X-Skysim-Capture-Unix-Usec: 1700000000123456\r\n") != std::string::npos);
    }

    video.stop();
    if (g_failures != 0) {
        std::printf("test_video_api: %d failure(s)\n", g_failures);
        return 1;
    }
    std::printf("test_video_api: OK\n");
    return 0;
}
