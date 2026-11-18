// skysim entry point: config -> World -> UdpEndpoints/ControlServer -> tick loop.
// Time policy (docs/DESIGN.md): both modes barrier on every connected vehicle's next frame;
// strict aborts on a straggler, interactive is wall-clock paced and evicts one (freeze->despawn).
// --canned keeps the M1 kinematic reply path; --replay-servo re-runs a recorded input tape.
#include <algorithm>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <vector>

#include "api/control_server.h"
#include "core/cpus.h"
#include "core/metrics.h"
#include "core/thread_pool.h"
#include "core/world.h"
#include "protocol/packets.h"
#include "radio/radio_model.h"
#include "radio/radio_spec.h"
#include "render/dock_marker.h"
#include "render/render_service.h"
#include "render/scene.h"
#include "render/still.h"
#include "video/video_service.h"
#include "protocol/udp_endpoint.h"
#include "terrain/tile_streamer.h"
#include "vehicle/gimbal.h"
#include "vehicle/manager.h"
#include "vehicle/quad.h"

namespace {

struct Options {
    int vehicles = 1;
    int base_instance = 0;
    // 800 Hz, matching the rate the README/DESIGN document and benchmark. It is also
    // the lowest rate ArduPilot copter will arm at under --model json: pre-arm requires
    // gyro rate >= SCHED_LOOP_RATE * 1.8, i.e. 720 Hz for the default 400 Hz loop, so a
    // 400 Hz world fails with "PreArm: Gyro 0 rate 400Hz < loop rate*1.8 720Hz" and the
    // vehicle can never take off.
    double dt_s = 1.0 / 800.0;
    std::string time_mode = "strict";
    std::string tiles;
    std::string truth_log;
    std::string record_servo;
    std::string replay_servo;
    skysim::core::Vec3 wind_ned{0.0, 0.0, 0.0};
    double gust_sigma = 0.0;
    double gust_tau = 2.0;
    uint64_t seed = 1;
    int rangefinders = 0;     // 0=none, 1=rng_1 down, 2=+rng_2 right (corridor wall tracking)
    bool no_lockstep = false; // emit no_lockstep:1 -> ArduPilot free-runs (loaded fleets)
    bool canned = false;
    double stream_radius_m = 600.0; // tiles resident within this of any vehicle
    int stream_max_resident = 64;   // hard tile memory bound
    int physics_threads = 0;        // Jolt job-system worker threads; 0 = single-threaded
                                    // (best for <~1000 bodies — see tools/bench/world_bench)
    int io_threads = 3;             // reply build+send worker threads (>48 vehicles); 0 = serial
    // M4: control plane + straggler policy + managed SITL processes.
    int api_port = 0;                   // 0 = control plane disabled
    std::string api_bind = "127.0.0.1"; // 0.0.0.0 when the gateway calls from a bridge net
    // interactive: how long a connected vehicle may go without sending a frame before it is
    // declared gone and dropped out of the barrier. A liveness timeout, not a smoothing knob —
    // a late vehicle stalls its own tick rather than flying on the last tick's PWM (see
    // run_interactive). In seconds, not ticks, so its meaning does not quietly change with --dt:
    // the tick-count knob this replaced defaulted to 3, which at 800 Hz wrote a vehicle off
    // after 3.75 ms and froze SITLs that were merely busy booting.
    double straggler_timeout_s = 2.0;
    // Fraction of a tick a frame may be late before it counts as a straggler.
    double frame_grace = 0.7;

    // Camera feed. Off unless asked for: rendering costs rays per pixel, and a
    // headless fleet run has nobody watching.
    double camera_fps = 0.0;  // 0 = no camera
    int camera_width = 256;
    int camera_height = 144;
    double camera_fov_deg = 78.0;
    double camera_pitch_deg = -15.0;
    // The gimbal every vehicle's camera sits on, read from its servo outputs (TASK-2548,
    // vehicle/gimbal.h): the 1-based outputs ArduPilot drives the mount's pitch and yaw on.
    // A vehicle that drives none keeps the fixed camera_pitch_deg. 0 = never follow a gimbal.
    int gimbal_pitch_channel = 10;
    int gimbal_yaw_channel = 11;
    double camera_range_m = 350.0;
    int camera_quality = 70;
    // Threads drawing a camera frame (SKYSIM_CAMERA_THREADS). They are outside the physics
    // tick and run at background priority, so this trades cores for frame rate. 0, the
    // default, is one per CPU this process can really use, at most 4 (core/cpus.h): the old
    // fixed 4 was five threads on a 2-vCPU task, past its quota.
    int camera_threads = 0;
    // Texture of the rendered world (SKYSIM_TEXTURE_SEED): the same seed draws the same world
    // in every run, the pilot camera and the survey stills alike.
    uint32_t texture_seed = 1;
    // Survey still camera (GET .../still.jpg): renders at once at most (SKYSIM_STILL_CONCURRENCY,
    // 0 = off), and worker threads per render. Independent of the camera feed above.
    int still_concurrency = 2;
    int still_threads = 2;
    // The camera as H.264 over RTSP (TASK-2561): rtsp://<host>:<rtsp_port>/instances/<n>/main.264,
    // the SIYI ZR30's URL layout. On with the camera (--camera-fps) and the control plane, which
    // resolves instances; 0 = off. rtsp_bind defaults to --api-bind.
    int rtsp_port = 8554;
    std::string rtsp_bind;
    // H.264 encoder threads (SKYSIM_VIDEO_THREADS); 0 = one per four CPUs this process can use,
    // 1 to 4. A stream at the camera's sizes costs a millisecond or two a frame.
    int video_threads = 0;
    // Keep serving camera.mjpg (SKYSIM_CAMERA_MJPEG). On until every consumer reads RTSP.
    bool camera_mjpeg = true;
    // How the pictures are drawn (TASK-2661; render/camera_mode.h). Every picture is
    // camera_width x camera_height; the normal mode draws it at camera_draw_* (0: that size) and
    // scales it up. The landing mode is what a vehicle draws while it is in the air within
    // camera_landing_radius_m of its dock and camera_landing_height_m above it — full size for
    // the dock marker, where a precision landing reads it — on when a landing size or fps is
    // given (SKYSIM_CAMERA_LANDING_SIZE / _FPS; the size defaults to the picture's, the fps to
    // the fleet's, the bitrate, 0, to the stream's own). camera_rest_fps (0: off) is the rate of
    // a vehicle sitting still on the ground.
    int camera_draw_width = 0;
    int camera_draw_height = 0;
    double camera_rest_fps = 0.0;
    bool camera_landing = false;
    int camera_landing_width = 0;
    int camera_landing_height = 0;
    double camera_landing_fps = 0.0;
    int camera_landing_bitrate_kbps = 0;
    double camera_landing_radius_m = 25.0;
    double camera_landing_height_m = 40.0;
    // Lit ground the pilot camera remembers between frames (render/ground_cache.h;
    // SKYSIM_CAMERA_CACHE_MB): 0 computes every pixel, as before. A 1920x1080 view needs 20-45 MB.
    int camera_cache_mb = 512;
    // Observer 1's e-ArUco dock marker on the ground at every vehicle's dock (TASK-2562;
    // render/dock_marker.h), drawn by the pilot camera and the survey stills alike
    // (SKYSIM_DOCK_MARKERS, SKYSIM_DOCK_MARKER_SIZE, SKYSIM_DOCK_INNER_SIZE). The sizes are the
    // black squares' sides, metres.
    bool dock_markers = true;
    double dock_marker_size_m = skysim::render::DockMarkerSpec::kDefaultOuterSizeM;
    double dock_inner_size_m = skysim::render::DockMarkerSpec::kDefaultInnerSizeM;
    // Which docks a vehicle's own cameras draw (TASK-2609; SKYSIM_DOCK_MARKERS_VISIBLE): "own",
    // the default, only its own; "all" every vehicle's, as before (render/dock.h).
    skysim::render::DockVisibility dock_markers_visible = skysim::render::DockVisibility::kOwn;
    double grace_s = 30.0;              // interactive: frozen longer than this => auto-despawn
    double strict_timeout_s = 10.0;     // strict: barrier stalled longer => abort with report
    std::string spawn_binary;           // arducopter for POST /vehicles {"launch_process":true}
    // Hristo Botev Stadium, Plovdiv — matches the gateway SITL_HOME_LOCATION and
    // the anchor used to cook the building world; the old default sat ~1.7 km away.
    std::string spawn_home = "42.1403890,24.7645490,0,0";
    std::string spawn_defaults;
    // Simulated radio field loaded at startup (docs/RADIO.md); PUT /radio can replace it later.
    std::string radio_spec;
    double spacing_m = 5.0; // east spacing between spawn positions (fleet airspace layout)
};

// A whole number in [lo, hi], or exit with the bad-option code: these set thread counts and
// the world's texture, where a silently truncated "2x" or a wrapped "-1" is a different
// deployment from the one that was asked for.
unsigned long long whole_number(const char *text, const char *flag, unsigned long long lo,
                                unsigned long long hi) {
    unsigned long long v = 0;
    const char *end = text + std::strlen(text);
    const auto parsed = std::from_chars(text, end, v);
    if (parsed.ec != std::errc() || parsed.ptr != end || v < lo || v > hi) {
        std::fprintf(stderr, "skysim: %s must be a whole number from %llu to %llu\n", flag, lo, hi);
        std::exit(2);
    }
    return v;
}

// A finite length in metres, or exit with the bad-option code (the dock marker's sizes: atof
// would read "1m" as 1 and "abc" as 0).
double metres(const char *text, const char *flag) {
    double v = 0.0;
    const char *end = text + std::strlen(text);
    const auto parsed = std::from_chars(text, end, v);
    if (parsed.ec != std::errc() || parsed.ptr != end || !std::isfinite(v)) {
        std::fprintf(stderr, "skysim: %s must be a length in metres\n", flag);
        std::exit(2);
    }
    return v;
}

// A finite number, or exit with the bad-option code (atof would read "15fps" as 15 and "abc" as 0).
double finite_number(const char *text, const char *flag) {
    double v = 0.0;
    const char *end = text + std::strlen(text);
    const auto parsed = std::from_chars(text, end, v);
    if (parsed.ec != std::errc() || parsed.ptr != end || !std::isfinite(v)) {
        std::fprintf(stderr, "skysim: %s must be a number\n", flag);
        std::exit(2);
    }
    return v;
}

Options parse_args(int argc, char **argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        auto need_value = [&](const char *flag) -> const char * {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "skysim: %s requires a value\n", flag);
                std::exit(2);
            }
            return argv[++i];
        };
        if (std::strcmp(argv[i], "--vehicles") == 0) {
            o.vehicles = std::atoi(need_value("--vehicles"));
        } else if (std::strcmp(argv[i], "--base-instance") == 0) {
            o.base_instance = std::atoi(need_value("--base-instance"));
        } else if (std::strcmp(argv[i], "--dt") == 0) {
            o.dt_s = std::atof(need_value("--dt"));
        } else if (std::strcmp(argv[i], "--time-mode") == 0) {
            o.time_mode = need_value("--time-mode");
        } else if (std::strcmp(argv[i], "--tiles") == 0) {
            o.tiles = need_value("--tiles");
        } else if (std::strcmp(argv[i], "--truth-log") == 0) {
            o.truth_log = need_value("--truth-log");
        } else if (std::strcmp(argv[i], "--record-servo") == 0) {
            o.record_servo = need_value("--record-servo");
        } else if (std::strcmp(argv[i], "--replay-servo") == 0) {
            o.replay_servo = need_value("--replay-servo");
        } else if (std::strcmp(argv[i], "--wind") == 0) {
            if (std::sscanf(need_value("--wind"), "%lf,%lf,%lf", &o.wind_ned[0], &o.wind_ned[1],
                            &o.wind_ned[2]) != 3) {
                std::fprintf(stderr, "skysim: --wind expects n,e,d (m/s)\n");
                std::exit(2);
            }
        } else if (std::strcmp(argv[i], "--gust") == 0) {
            if (std::sscanf(need_value("--gust"), "%lf,%lf", &o.gust_sigma, &o.gust_tau) < 1) {
                std::fprintf(stderr, "skysim: --gust expects sigma[,tau]\n");
                std::exit(2);
            }
        } else if (std::strcmp(argv[i], "--seed") == 0) {
            o.seed = std::strtoull(need_value("--seed"), nullptr, 10);
        } else if (std::strcmp(argv[i], "--rangefinder") == 0) {
            o.rangefinders = std::max(o.rangefinders, 1);
        } else if (std::strcmp(argv[i], "--rangefinders") == 0) {
            o.rangefinders = std::atoi(need_value("--rangefinders"));
        } else if (std::strcmp(argv[i], "--stream-radius") == 0) {
            o.stream_radius_m = std::atof(need_value("--stream-radius"));
        } else if (std::strcmp(argv[i], "--stream-max") == 0) {
            o.stream_max_resident = std::atoi(need_value("--stream-max"));
        } else if (std::strcmp(argv[i], "--physics-threads") == 0) {
            o.physics_threads = std::atoi(need_value("--physics-threads"));
        } else if (std::strcmp(argv[i], "--io-threads") == 0) {
            o.io_threads = std::atoi(need_value("--io-threads"));
        } else if (std::strcmp(argv[i], "--no-lockstep") == 0) {
            o.no_lockstep = true;
        } else if (std::strcmp(argv[i], "--canned") == 0) {
            o.canned = true;
        } else if (std::strcmp(argv[i], "--api-port") == 0) {
            o.api_port = std::atoi(need_value("--api-port"));
        } else if (std::strcmp(argv[i], "--api-bind") == 0) {
            o.api_bind = need_value("--api-bind");
        } else if (std::strcmp(argv[i], "--camera-fps") == 0) {
            o.camera_fps = std::atof(need_value("--camera-fps"));
        } else if (std::strcmp(argv[i], "--camera-size") == 0) {
            // WxH, e.g. 256x144
            const char *v = need_value("--camera-size");
            const char *x = std::strchr(v, 'x');
            o.camera_width = std::atoi(v);
            o.camera_height = (x != nullptr) ? std::atoi(x + 1) : 0;
        } else if (std::strcmp(argv[i], "--camera-fov") == 0) {
            o.camera_fov_deg = std::atof(need_value("--camera-fov"));
        } else if (std::strcmp(argv[i], "--camera-pitch") == 0) {
            o.camera_pitch_deg = std::atof(need_value("--camera-pitch"));
        } else if (std::strcmp(argv[i], "--gimbal-pitch-channel") == 0) {
            o.gimbal_pitch_channel = static_cast<int>(
                whole_number(need_value("--gimbal-pitch-channel"), "--gimbal-pitch-channel", 0, 32));
        } else if (std::strcmp(argv[i], "--gimbal-yaw-channel") == 0) {
            o.gimbal_yaw_channel = static_cast<int>(
                whole_number(need_value("--gimbal-yaw-channel"), "--gimbal-yaw-channel", 0, 32));
        } else if (std::strcmp(argv[i], "--camera-range") == 0) {
            o.camera_range_m = std::atof(need_value("--camera-range"));
        } else if (std::strcmp(argv[i], "--camera-quality") == 0) {
            o.camera_quality = std::atoi(need_value("--camera-quality"));
        } else if (std::strcmp(argv[i], "--camera-threads") == 0) {
            o.camera_threads =
                static_cast<int>(whole_number(need_value("--camera-threads"), "--camera-threads", 0, 64));
        } else if (std::strcmp(argv[i], "--texture-seed") == 0) {
            o.texture_seed = static_cast<uint32_t>(
                whole_number(need_value("--texture-seed"), "--texture-seed", 0, 4294967295ULL));
        } else if (std::strcmp(argv[i], "--still-concurrency") == 0) {
            o.still_concurrency = static_cast<int>(
                whole_number(need_value("--still-concurrency"), "--still-concurrency", 0, 16));
        } else if (std::strcmp(argv[i], "--still-threads") == 0) {
            o.still_threads =
                static_cast<int>(whole_number(need_value("--still-threads"), "--still-threads", 1, 64));
        } else if (std::strcmp(argv[i], "--rtsp-port") == 0) {
            o.rtsp_port = static_cast<int>(whole_number(need_value("--rtsp-port"), "--rtsp-port", 0, 65535));
        } else if (std::strcmp(argv[i], "--rtsp-bind") == 0) {
            o.rtsp_bind = need_value("--rtsp-bind");
        } else if (std::strcmp(argv[i], "--video-threads") == 0) {
            o.video_threads =
                static_cast<int>(whole_number(need_value("--video-threads"), "--video-threads", 0, 16));
        } else if (std::strcmp(argv[i], "--camera-landing-size") == 0 ||
                   std::strcmp(argv[i], "--camera-draw-size") == 0) {
            const bool landing = std::strcmp(argv[i], "--camera-landing-size") == 0;
            const char *flag = landing ? "--camera-landing-size" : "--camera-draw-size";
            const char *v = need_value(flag);
            int w = 0;
            int h = 0;
            char tail = 0;
            if (std::sscanf(v, "%dx%d%c", &w, &h, &tail) != 2 || w < skysim::render::kMinCameraSide ||
                w > skysim::render::kMaxCameraWidth || h < skysim::render::kMinCameraSide ||
                h > skysim::render::kMaxCameraHeight) {
                std::fprintf(stderr, "skysim: %s must be WxH, 16x16..3840x2160\n", flag);
                std::exit(2);
            }
            (landing ? o.camera_landing_width : o.camera_draw_width) = w;
            (landing ? o.camera_landing_height : o.camera_draw_height) = h;
            o.camera_landing = o.camera_landing || landing;
        } else if (std::strcmp(argv[i], "--camera-rest-fps") == 0) {
            o.camera_rest_fps = finite_number(need_value("--camera-rest-fps"), "--camera-rest-fps");
            if (o.camera_rest_fps != 0.0 && (o.camera_rest_fps < 0.1 || o.camera_rest_fps > 60.0)) {
                std::fprintf(stderr, "skysim: --camera-rest-fps must be 0 (off) or 0.1..60\n");
                std::exit(2);
            }
        } else if (std::strcmp(argv[i], "--camera-landing-fps") == 0) {
            o.camera_landing_fps = finite_number(need_value("--camera-landing-fps"), "--camera-landing-fps");
            if (o.camera_landing_fps < skysim::render::kMinCameraFps ||
                o.camera_landing_fps > skysim::render::kMaxCameraFps) {
                std::fprintf(stderr, "skysim: --camera-landing-fps must be 0.5..60\n");
                std::exit(2);
            }
            o.camera_landing = true;
        } else if (std::strcmp(argv[i], "--camera-landing-bitrate") == 0) {
            o.camera_landing_bitrate_kbps = static_cast<int>(
                whole_number(need_value("--camera-landing-bitrate"), "--camera-landing-bitrate", 0, 8000));
            if (o.camera_landing_bitrate_kbps != 0 &&
                o.camera_landing_bitrate_kbps < skysim::render::kMinCameraBitrateKbps) {
                std::fprintf(stderr,
                             "skysim: --camera-landing-bitrate must be 0 (the stream's own) or 64..8000\n");
                std::exit(2);
            }
        } else if (std::strcmp(argv[i], "--camera-landing-radius") == 0) {
            o.camera_landing_radius_m =
                metres(need_value("--camera-landing-radius"), "--camera-landing-radius");
        } else if (std::strcmp(argv[i], "--camera-landing-height") == 0) {
            o.camera_landing_height_m =
                metres(need_value("--camera-landing-height"), "--camera-landing-height");
        } else if (std::strcmp(argv[i], "--camera-cache-mb") == 0) {
            o.camera_cache_mb = static_cast<int>(
                whole_number(need_value("--camera-cache-mb"), "--camera-cache-mb", 0, 65536));
        } else if (std::strcmp(argv[i], "--camera-mjpeg") == 0) {
            o.camera_mjpeg = whole_number(need_value("--camera-mjpeg"), "--camera-mjpeg", 0, 1) == 1;
        } else if (std::strcmp(argv[i], "--dock-markers") == 0) {
            o.dock_markers = whole_number(need_value("--dock-markers"), "--dock-markers", 0, 1) == 1;
        } else if (std::strcmp(argv[i], "--dock-marker-size") == 0) {
            o.dock_marker_size_m = metres(need_value("--dock-marker-size"), "--dock-marker-size");
        } else if (std::strcmp(argv[i], "--dock-inner-size") == 0) {
            o.dock_inner_size_m = metres(need_value("--dock-inner-size"), "--dock-inner-size");
        } else if (std::strcmp(argv[i], "--dock-markers-visible") == 0) {
            const auto visible = skysim::render::parse_dock_visibility(need_value("--dock-markers-visible"));
            if (!visible) {
                std::fprintf(stderr, "skysim: --dock-markers-visible must be own or all\n");
                std::exit(2);
            }
            o.dock_markers_visible = *visible;
        } else if (std::strcmp(argv[i], "--frame-grace") == 0) {
            o.frame_grace = std::atof(need_value("--frame-grace"));
        } else if (std::strcmp(argv[i], "--straggler-timeout") == 0) {
            // Checked here rather than where it is used: run_interactive turns this into a
            // tick count by dividing by dt, and atof happily returns nan or inf for "abc" or
            // "1e400". Converting either to int is undefined behaviour, so the freeze
            // threshold would be whatever that produced — silently, on a flag about how long
            // to tolerate a silent vehicle.
            const double v = std::atof(need_value("--straggler-timeout"));
            if (!std::isfinite(v) || v < 0.0 || v > 86400.0) {
                std::fprintf(stderr, "skysim: --straggler-timeout must be between 0 and 86400 seconds\n");
                std::exit(2);
            }
            o.straggler_timeout_s = v;
        } else if (std::strcmp(argv[i], "--grace") == 0) {
            o.grace_s = std::atof(need_value("--grace"));
        } else if (std::strcmp(argv[i], "--strict-timeout") == 0) {
            o.strict_timeout_s = std::atof(need_value("--strict-timeout"));
        } else if (std::strcmp(argv[i], "--spawn-binary") == 0) {
            o.spawn_binary = need_value("--spawn-binary");
        } else if (std::strcmp(argv[i], "--spawn-home") == 0) {
            o.spawn_home = need_value("--spawn-home");
        } else if (std::strcmp(argv[i], "--spawn-defaults") == 0) {
            o.spawn_defaults = need_value("--spawn-defaults");
        } else if (std::strcmp(argv[i], "--radio-spec") == 0) {
            o.radio_spec = need_value("--radio-spec");
        } else if (std::strcmp(argv[i], "--spacing") == 0) {
            o.spacing_m = std::atof(need_value("--spacing"));
        } else {
            std::fprintf(stderr, "skysim: unknown arg %s\n", argv[i]);
            std::exit(2);
        }
    }
    if (o.vehicles < 0 || o.dt_s <= 0.0 || (o.time_mode != "strict" && o.time_mode != "interactive")) {
        std::fprintf(stderr, "skysim: invalid options (--time-mode strict|interactive)\n");
        std::exit(2);
    }
    // These three change timing or thread counts rather than failing outright, so
    // an out-of-range value does not announce itself: a negative thread count is
    // an undefined pool size, a grace above 1 stretches every tick past its
    // period, and stb quietly reads quality 0 as 90.
    if (o.camera_quality < 1 || o.camera_quality > 100) {
        std::fprintf(stderr, "skysim: --camera-quality must be 1-100\n");
        std::exit(2);
    }
    if (o.camera_landing_radius_m < 0.0 || o.camera_landing_radius_m > 10000.0 ||
        o.camera_landing_height_m < 0.0 || o.camera_landing_height_m > 10000.0) {
        std::fprintf(stderr,
                     "skysim: --camera-landing-radius / --camera-landing-height must be 0..10000 m\n");
        std::exit(2);
    }
    if (o.frame_grace < 0.0 || o.frame_grace > 1.0) {
        std::fprintf(stderr, "skysim: --frame-grace must be 0.0-1.0 (a fraction of the tick period)\n");
        std::exit(2);
    }
    skysim::render::DockMarkerSpec dock;
    dock.outer_size_m = o.dock_marker_size_m;
    dock.inner_size_m = o.dock_inner_size_m;
    if (const std::string why = dock.invalid(); !why.empty()) {
        std::fprintf(stderr, "skysim: --dock-marker-size / --dock-inner-size: %s\n", why.c_str());
        std::exit(2);
    }
    return o;
}

std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }

// --radio-spec: the radio field a deployment starts with. Any problem stops startup with the
// bad-option exit code: a survey flown against a field that silently failed to load would
// read as "no service" everywhere rather than as a configuration error.
bool load_radio_spec(const std::string &path, skysim::radio::RadioField &field) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        std::fprintf(stderr, "skysim: cannot read --radio-spec %s\n", path.c_str());
        return false;
    }
    std::stringstream text;
    text << in.rdbuf();
    skysim::radio::SpecResult parsed = skysim::radio::parse_radio_spec(text.str());
    if (!parsed.spec) {
        std::fprintf(stderr, "skysim: --radio-spec %s: %s\n", path.c_str(), parsed.error.c_str());
        return false;
    }
    size_t towers = 0;
    for (const auto &c : parsed.spec->carriers) {
        towers += c.towers.size();
    }
    std::printf("skysim: radio field from %s: %zu carrier(s), %zu tower(s), %zu hole(s)\n", path.c_str(),
                parsed.spec->carriers.size(), towers, parsed.spec->holes.size());
    field.set(std::make_shared<const skysim::radio::RadioModel>(std::move(*parsed.spec)));
    return true;
}

// Where the world's NED origin is on the earth, for placing vehicles in the radio field:
// --spawn-home's lat,lon, with its altitude as the ground's height AMSL (0 when omitted).
// It is the same point every managed SITL is given as --home.
//
// Every field must be a whole number (spaces around it allowed): a prefix parse would read
// "47,8x,500" as 47,8 at ground 0 and place every vehicle with no warning. Values are
// range-checked too, since from_chars reads "nan" and "inf" as numbers.
std::optional<skysim::radio::GeoAnchor> spawn_anchor(const std::string &home) {
    std::vector<double> fields; // lat, lon[, alt[, yaw]]
    size_t start = 0;
    while (true) {
        const size_t comma = home.find(',', start);
        std::string_view field(home);
        field = field.substr(start, comma == std::string::npos ? std::string_view::npos : comma - start);
        while (!field.empty() && field.front() == ' ') {
            field.remove_prefix(1);
        }
        while (!field.empty() && field.back() == ' ') {
            field.remove_suffix(1);
        }
        double v = 0.0;
        const auto parsed = std::from_chars(field.data(), field.data() + field.size(), v);
        if (field.empty() || parsed.ec != std::errc() || parsed.ptr != field.data() + field.size() ||
            !std::isfinite(v) || fields.size() == 4) {
            return std::nullopt;
        }
        fields.push_back(v);
        if (comma == std::string::npos) {
            break;
        }
        start = comma + 1;
    }
    if (fields.size() < 2) {
        return std::nullopt;
    }
    const skysim::radio::GeoAnchor anchor{fields[0], fields[1], fields.size() > 2 ? fields[2] : 0.0};
    if (std::fabs(anchor.lat_deg) > 90.0 || std::fabs(anchor.lon_deg) > 180.0 ||
        std::fabs(anchor.ground_amsl_m) > 10000.0) {
        return std::nullopt;
    }
    return anchor;
}

// One vehicle: endpoint (null when replaying) + body + motors + straggler bookkeeping.
struct VehicleSlot {
    std::unique_ptr<skysim::protocol::UdpEndpoint> endpoint;
    uint32_t vehicle_id = 0; // app-level id (control plane)
    int instance = 0;
    std::optional<pid_t> pid; // set when this slot's arducopter is managed by us
    uint32_t body_id = 0;
    // Where it spawned (NED): the dock the landing camera measures from without dock markers.
    skysim::core::Vec3 home_ned{};
    skysim::vehicle::QuadParams params;
    std::array<skysim::vehicle::MotorState, 4> motors{};
    skysim::core::BodyState state;
    char last_json[1024];
    size_t last_json_len = 0;
    bool has_pending_input = false; // fresh frame consumed, reply owed after this tick
    skysim::protocol::ParsedServos pending{};
    bool connected = false; // ever received a frame
    int silent_ticks = 0;     // consecutive deadline misses (interactive)
    bool frozen = false;
    uint64_t frozen_at_tick = 0;
    uint64_t gaps = 0, reboots = 0, bad = 0, ticks = 0;

    // --- battery -------------------------------------------------------------------
    // A coarse but honest pack model: current scales with total motor demand, and
    // voltage sags as charge is used. It exists so endurance is a real constraint —
    // without it a SITL hovers indefinitely and nothing that plans around battery
    // (an RTL margin, a relay handover) can be exercised at all.
    double battery_consumed_mah = 0.0;
    double battery_voltage_v = 0.0;
    double battery_current_a = 0.0;
    // The pack this vehicle carries, chosen at spawn. Per-vehicle because a world can hold
    // a mixed fleet, and because the pack is what sets endurance — the constant that used
    // to be compiled in here quietly made every vehicle a 3S hobby quad regardless of what
    // airframe it was standing in for.
    skysim::vehicle::BatteryPack battery{};
    // The gimbal as the frame last stepped drove it (TASK-2548), latched with the step so the
    // pose and the gimbal a picture is drawn from are always one tick's: `pending` itself can
    // already hold the next, unstepped frame while the world waits for the rest of the fleet.
    skysim::vehicle::GimbalState gimbal{};

    VehicleSlot() = default;
    explicit VehicleSlot(int inst)
        : endpoint(std::make_unique<skysim::protocol::UdpEndpoint>(inst)), instance(inst) {}
};

using Fleet = std::vector<std::unique_ptr<VehicleSlot>>;

skysim::protocol::VehicleTruth truth_from_state(const skysim::core::BodyState &s, double t) {
    skysim::protocol::VehicleTruth out{};
    out.timestamp_s = t;
    for (int k = 0; k < 3; ++k) {
        out.gyro_rps[k] = s.gyro_frd[k];
        out.accel_body[k] = s.accel_body_frd[k];
        out.pos_ned_m[k] = s.pos_ned[k];
        out.vel_ned_mps[k] = s.vel_ned[k];
    }
    for (int k = 0; k < 4; ++k) {
        out.quat_wxyz[k] = s.quat_ned_frd[k];
    }
    return out;
}

// Integrate one tick of discharge from the commanded motor demand.
//
// The pack comes from the vehicle (see VehicleSlot::battery), not from a constant: the
// same world routinely holds an Observer-class aircraft and a small quad, and their
// endurance differs by the better part of an hour.
//
// ArduPilot only reads the battery block when both voltage and current are present; with
// neither it synthesises voltage from instantaneous throttle, which never sags and so can
// never trip a failsafe. Current is the half that matters most: ArduPilot integrates it
// into consumed_mah, and capacity_remaining_pct — the number every planner reads — is
// derived from that.
void step_battery(VehicleSlot &v, const std::array<uint16_t, 16> &pwm, double dt_s) {
    // Mean normalised throttle across the four motors, from standard 1100-1900 PWM.
    double demand = 0.0;
    for (int k = 0; k < 4; ++k) {
        const double norm = (static_cast<double>(pwm[k]) - 1100.0) / 800.0;
        demand += std::clamp(norm, 0.0, 1.0);
    }
    demand /= 4.0;

    // Quadratic in throttle: drawing twice the thrust costs appreciably more than twice
    // the current, which is why an aggressive climb eats a pack so much faster than a hover.
    v.battery_current_a = v.battery.idle_a + v.battery.hover_a * demand * demand;
    v.battery_consumed_mah += v.battery_current_a * (dt_s / 3600.0) * 1000.0;

    const double used = std::clamp(v.battery_consumed_mah / v.battery.capacity_mah, 0.0, 1.0);
    // Linear sag is not the real curve, but it is monotonic and ordered, which is all a
    // voltage failsafe needs. Consumed charge is the signal that should be trusted.
    v.battery_voltage_v = v.battery.full_v - (v.battery.full_v - v.battery.empty_v) * used;
}

// Apply wrenches for every active vehicle and advance the world one tick.
//
// Invariant, and both loops are built to hold it: a participating slot's `pending` PWM is
// always a frame the vehicle actually sent for this tick. Slots that are frozen or have never
// connected sit out (gravity + contacts only); nobody is ever stepped on a replay of last
// tick's PWM. The autopilot reads its own loop rate off the timestamps we return, so a step it
// did not contribute to is not an approximation — it desynchronises its controller.
void step_world(skysim::core::World &world, Fleet &fleet, double dt_s, FILE *truth_log, FILE *record_log) {
    const auto wind = world.wind_ned();
    for (size_t i = 0; i < fleet.size(); ++i) {
        auto &v = *fleet[i];
        if (v.frozen || !v.connected) {
            continue; // kinematic hold / never-armed slot: gravity + contacts only
        }
        if (record_log != nullptr && v.has_pending_input) {
            std::fprintf(record_log, "%llu,%zu,%u", static_cast<unsigned long long>(world.tick_index()), i,
                         v.pending.frame_rate_hz);
            for (int k = 0; k < 16; ++k) {
                std::fprintf(record_log, ",%u", v.pending.pwm[k]);
            }
            std::fputc('\n', record_log);
        }
        const skysim::core::Vec3 v_air_ned{v.state.vel_ned[0] - wind[0], v.state.vel_ned[1] - wind[1],
                                           v.state.vel_ned[2] - wind[2]};
        const auto vel_frd = skysim::core::quat_rotate_inverse(v.state.quat_ned_frd, v_air_ned);
        std::array<uint16_t, 16> pwm{};
        std::copy_n(v.pending.pwm.begin(), 16, pwm.begin());
        const double vel_air_frd[3] = {vel_frd[0], vel_frd[1], vel_frd[2]};
        const auto wrench = skysim::vehicle::compute_wrench(v.params, v.motors, pwm, vel_air_frd, dt_s);
        step_battery(v, pwm, dt_s);
        world.apply_body_wrench(v.body_id, {wrench.force_frd[0], wrench.force_frd[1], wrench.force_frd[2]},
                                {wrench.torque_frd[0], wrench.torque_frd[1], wrench.torque_frd[2]});
    }
    world.step();
    const double now_s = world.now();
    for (size_t i = 0; i < fleet.size(); ++i) {
        fleet[i]->state = world.get_state(fleet[i]->body_id);
        if (truth_log != nullptr) {
            const auto &pos = fleet[i]->state.pos_ned;
            std::fprintf(truth_log, "%.4f,%zu,%.4f,%.4f,%.4f\n", now_s, i, pos[0], pos[1], pos[2]);
        }
    }
}

// Poll one endpoint; returns true if a fresh steppable frame was staged.
bool poll_endpoint(VehicleSlot &v) {
    using skysim::protocol::FrameStatus;
    if (v.has_pending_input || !v.endpoint) {
        return v.has_pending_input;
    }
    if (auto p = v.endpoint->take_latest()) {
        switch (p->status) {
        case FrameStatus::kOk:
        case FrameStatus::kGap:
        case FrameStatus::kReboot:
            if (p->status == FrameStatus::kGap) {
                ++v.gaps;
            }
            if (p->status == FrameStatus::kReboot) {
                ++v.reboots;
                std::printf("skysim: vehicle %u reboot detected (frame_count %u); pose kept\n", v.vehicle_id,
                            p->frame_count);
            }
            v.pending = *p;
            v.has_pending_input = true;
            v.connected = true;
            break;
        case FrameStatus::kDuplicate:
            // Handshake duplicates must be re-answered, already-answered ones must not
            // (lockstep desync — see udp_endpoint.h).
            if (v.last_json_len > 0 && v.endpoint->last_taken_unanswered()) {
                v.endpoint->send_reply(v.last_json, v.last_json_len);
            }
            break;
        case FrameStatus::kBadMagic:
        case FrameStatus::kBadSize:
            ++v.bad;
            break;
        }
    }
    return v.has_pending_input;
}

// Build + send one vehicle's reply (truth -> JSON -> UDP). Touches only this slot and its own
// socket, so it is safe to run concurrently for different vehicles. Returns false on JSON
// buffer overflow. Rangefinder raycasts read the Jolt world read-only (safe after step()).
bool reply_one(VehicleSlot &v, skysim::core::World *world, const Options &opt, double now_s) {
    if (!v.has_pending_input) {
        return true;
    }
    skysim::protocol::VehicleTruth truth;
    if (world != nullptr) {
        truth = truth_from_state(v.state, now_s);
        if (opt.rangefinders > 0) {
            // rng_1: down (body +z), rng_2: right (body +y) — corridor wall tracking.
            const skysim::core::Vec3 body_dirs[2] = {{0.0, 0.0, 1.0}, {0.0, 1.0, 0.0}};
            const int n = std::min(opt.rangefinders, 2);
            for (int r = 0; r < n; ++r) {
                const auto dir_world = skysim::core::quat_rotate(v.state.quat_ned_frd, body_dirs[r]);
                const double d = world->raycast(v.state.pos_ned, dir_world, 40.0, v.body_id);
                truth.rangefinder_m[r] = d < 0.0 ? 40.0 : d;
            }
            truth.rangefinder_count = static_cast<uint8_t>(n);
        }
        truth.no_lockstep = opt.no_lockstep;
        // Only once a frame has actually been stepped: reporting a full pack before the
        // vehicle has drawn anything would be a guess, and the block is optional
        // precisely so it can be withheld.
        truth.battery_valid = v.battery_voltage_v > 0.0;
        truth.battery_voltage_v = v.battery_voltage_v;
        truth.battery_current_a = v.battery_current_a;
    } else {
        truth = {};
        truth.timestamp_s = now_s;
        truth.accel_body[2] = -9.81;
        truth.quat_wxyz[0] = 1.0;
    }
    v.last_json_len = skysim::protocol::build_state_json(truth, v.last_json, sizeof(v.last_json));
    if (v.last_json_len == 0) {
        return false;
    }
    if (v.endpoint) {
        v.endpoint->send_reply(v.last_json, v.last_json_len);
    }
    v.has_pending_input = false;
    ++v.ticks;
    return true;
}

// Build + send the reply for every vehicle whose fresh input was consumed this tick. The
// per-vehicle reply (JSON format + sendto syscall) is kernel-bound and embarrassingly
// parallel; above kReplyParallelThreshold vehicles it is fanned across io_pool (each syscall
// runs on its own core). Below that, the serial path avoids dispatch overhead (same lesson as
// the physics threading — see core/world.cpp). io_pool == nullptr => always serial.
constexpr size_t kReplyParallelThreshold = 48;

int send_replies(skysim::core::World *world, Fleet &fleet, const Options &opt, double now_s,
                 skysim::core::ThreadPool *io_pool) {
    if (io_pool != nullptr && fleet.size() > kReplyParallelThreshold) {
        std::atomic<bool> overflow{false};
        io_pool->parallel_for(fleet.size(), [&](size_t begin, size_t end) {
            for (size_t i = begin; i < end; ++i) {
                if (!reply_one(*fleet[i], world, opt, now_s)) {
                    overflow.store(true, std::memory_order_relaxed);
                }
            }
        });
        if (overflow.load()) {
            std::fprintf(stderr, "skysim: build_state_json overflow\n");
            return 1;
        }
        return 0;
    }
    for (auto &v : fleet) {
        if (!reply_one(*v, world, opt, now_s)) {
            std::fprintf(stderr, "skysim: build_state_json overflow\n");
            return 1;
        }
    }
    return 0;
}

int run_replay(const Options &opt, skysim::core::World &world, Fleet &fleet, FILE *truth_log) {
    FILE *in = std::fopen(opt.replay_servo.c_str(), "r");
    if (in == nullptr) {
        std::fprintf(stderr, "skysim: cannot open --replay-servo %s\n", opt.replay_servo.c_str());
        return 1;
    }
    char line[512];
    size_t staged = 0, steps = 0;
    while (std::fgets(line, sizeof(line), in) != nullptr) {
        unsigned long long tick;
        size_t vi;
        unsigned rate;
        int consumed = 0;
        if (std::sscanf(line, "%llu,%zu,%u%n", &tick, &vi, &rate, &consumed) != 3 || vi >= fleet.size()) {
            std::fprintf(stderr, "skysim: bad replay line: %s", line);
            std::fclose(in);
            return 1;
        }
        auto &v = *fleet[vi];
        v.pending.frame_rate_hz = static_cast<uint16_t>(rate);
        const char *cursor = line + consumed;
        for (int k = 0; k < 16; ++k) {
            unsigned pwm_val = 0;
            int used = 0;
            if (std::sscanf(cursor, ",%u%n", &pwm_val, &used) != 1) {
                std::fprintf(stderr, "skysim: bad replay pwm at line: %s", line);
                std::fclose(in);
                return 1;
            }
            v.pending.pwm[k] = static_cast<uint16_t>(pwm_val);
            cursor += used;
        }
        v.has_pending_input = true;
        v.connected = true;
        if (++staged == fleet.size()) {
            step_world(world, fleet, opt.dt_s, truth_log, nullptr);
            for (auto &s : fleet) {
                s->has_pending_input = false;
            }
            staged = 0;
            ++steps;
        }
    }
    std::fclose(in);
    std::printf("skysim: replay done, %zu steps\n", steps);
    return 0;
}

// The whole mutable app state the control plane interacts with (tick thread owns it all).
struct App {
    Options opt;
    std::unique_ptr<skysim::core::World> world;
    std::unique_ptr<skysim::vehicle::VehicleManager> manager;
    std::unique_ptr<skysim::terrain::TileStreamer> streamer;
    std::unordered_map<size_t, uint32_t> tile_world_ids; // streamer index -> world tile id
    Fleet fleet;
    skysim::api::CommandQueue queue;
    skysim::core::TickMetrics metrics;
    std::unique_ptr<skysim::core::ThreadPool> io_pool;
    uint32_t next_vehicle_id = 1;

    // Apply one streaming plan to the world. Returns true if anything actually changed.
    //
    // Both callers must go through this: the broadphase bookkeeping below is not optional,
    // and having it in one place is what stops the next caller from forgetting it.
    bool apply_tile_plan(const skysim::terrain::TileStreamer::Update &plan) {
        for (size_t idx : plan.remove) {
            auto it = tile_world_ids.find(idx);
            if (it != tile_world_ids.end()) {
                world->remove_static_tile(it->second);
                tile_world_ids.erase(it);
            }
        }
        for (size_t idx : plan.add) {
            const uint32_t id = world->add_static_tile(streamer->tile_path(idx));
            if (id != 0) {
                tile_world_ids.emplace(idx, id);
            }
        }
        if (plan.add.empty() && plan.remove.empty()) {
            return false;
        }
        // Reclaim the broadphase nodes those removals freed. Normally PhysicsSystem::Update
        // does this, but a world with no vehicles never steps (the strict barrier is never
        // satisfied), so nothing would ever reclaim them and the node pool would run dry —
        // "QuadTree: Out of nodes!" then std::abort(). See World::optimize_broadphase.
        world->optimize_broadphase();
        return true;
    }

    // Which tick stream_tiles() last acted on, so a stalled loop cannot rescan
    // repeatedly on the same tick. Empty until the first evaluation.
    std::optional<uint64_t> last_stream_tick;

    // Re-evaluate tile residency every ~0.125 s of sim time (tick boundaries only).
    void stream_tiles() {
        if (!streamer) {
            return;
        }
        // With no vehicles there is nothing to stream around. Recomputing the desired set
        // here would find it empty and evict every tile a path check had just loaded — and
        // the next path check would load them straight back. That churn is what exhausted
        // the broadphase; leaving residency alone is both cheaper and correct, since the
        // resident set is already bounded by max_resident.
        if (fleet.empty()) {
            return;
        }
        // Throttle on the tick actually evaluated, not on the tick number alone.
        //
        // `tick_index() % 100` looks like "every 100 ticks" but only advances when the
        // world steps, and the strict loop calls this every ~50 us whether it stepped or
        // not. Any stall that parks tick_index on a multiple of 100 — a fleet whose
        // barrier is waiting on a vehicle that has not sent a frame yet, which is the
        // normal state between spawn and SITL connect — turns the throttle off entirely
        // and rescans at ~20 kHz. Remembering which tick was last evaluated is what makes
        // it a throttle rather than a coincidence.
        const uint64_t tick = world->tick_index();
        if (tick % 100 != 0 || (last_stream_tick && *last_stream_tick == tick)) {
            return;
        }
        last_stream_tick = tick;
        std::vector<skysim::core::Vec3> positions;
        positions.reserve(fleet.size());
        for (const auto &v : fleet) {
            positions.push_back(v->state.pos_ned);
        }
        const auto plan = streamer->update(positions);
        if (apply_tile_plan(plan)) {
            std::printf("skysim: tiles +%zu -%zu (resident %zu)\n", plan.add.size(), plan.remove.size(),
                        streamer->resident_count());
        }
    }

    std::mutex snapshot_mutex;
    std::vector<skysim::api::VehicleInfo> vehicle_snapshot;
    skysim::api::MetricsInfo metrics_snapshot;

    // The camera's H.264/RTSP streams. Declared before render_service so it is destroyed after
    // it: the render thread hands it pictures until it stops. ~App stops it first, because its
    // threads call back into render_service (through render_ptr).
    std::unique_ptr<skysim::video::VideoService> video_service;
    std::atomic<skysim::render::RenderService *> render_ptr{nullptr};

    // Camera feed. Rendered on this thread because raycasting a stepping world
    // is not safe; published for the HTTP thread exactly like the snapshots above.
    std::unique_ptr<skysim::render::RenderService> render_service;
    // Survey stills, rendered on HTTP request from the same snapshot /vehicles serves.
    std::unique_ptr<skysim::render::StillService> still_service;
    // Every vehicle's dock marker (TASK-2562), drawn by both cameras — a vehicle's own cameras draw
    // the docks seen_by() gives them, by default its own alone (TASK-2609); null when neither
    // camera is on or --dock-markers 0. A vehicle's dock starts where it spawns and goes when it does.
    std::shared_ptr<skysim::render::DockBoard> docks;

    ~App() {
        if (video_service) {
            video_service->stop();
        }
    }
    App() = default;
    App(const App &) = delete;
    App &operator=(const App &) = delete;

    // The vehicle id now at an ArduPilot instance, 0 when none (any thread).
    uint32_t vehicle_at(int instance) {
        std::lock_guard<std::mutex> lock(snapshot_mutex);
        for (const auto &v : vehicle_snapshot) {
            if (v.instance == instance) {
                return v.id;
            }
        }
        return 0;
    }

    std::unique_ptr<VehicleSlot> make_slot(int instance) {
        auto slot = std::make_unique<VehicleSlot>(instance);
        slot->vehicle_id = next_vehicle_id++;
        skysim::core::VehicleBodyParams bp;
        bp.mass_kg = slot->params.mass_kg;
        bp.inertia_diag_frd = {slot->params.inertia_diag[0], slot->params.inertia_diag[1],
                               slot->params.inertia_diag[2]};
        // Spawn on a 5 m east-spaced line by instance offset (deterministic, collision-free).
        bp.start_pos_ned = {0.0, opt.spacing_m * (instance - opt.base_instance),
                            -(bp.half_extents_frd[2] + 0.005)};
        slot->body_id = world->add_vehicle(bp);
        slot->home_ned = bp.start_pos_ned;
        slot->state = world->get_state(slot->body_id);
        if (docks) {
            // Its home, heading north: where ArduPilot will set home when it arms, so an RTL comes
            // back to the marker unless the control plane moves it.
            docks->add(slot->vehicle_id, bp.start_pos_ned[0], bp.start_pos_ned[1]);
        }
        return slot;
    }

    void despawn(size_t idx) {
        auto &v = *fleet[idx];
        std::printf("skysim: despawning vehicle %u (instance %d)\n", v.vehicle_id, v.instance);
        if (v.pid.has_value()) {
            manager->stop_sitl(*v.pid);
        }
        world->remove_vehicle(v.body_id);
        if (docks) {
            docks->remove(v.vehicle_id);
        }
        if (render_service) {
            render_service->forget(v.vehicle_id); // its own camera settings go with it
        }
        manager->release_instance(v.instance);
        fleet.erase(fleet.begin() + static_cast<long>(idx));
    }

    // Drain control-plane commands at the tick boundary (DESIGN.md threading step 1).
    void drain_commands() {
        for (auto &cmd : queue.drain()) {
            if (auto *spawn = std::get_if<skysim::api::SpawnCommand>(&cmd)) {
                std::optional<skysim::vehicle::SpawnResult> result;
                int instance = -1;
                if (spawn->request.instance >= 0) {
                    // External allocator (SkyHub gateway) is authoritative for numbering.
                    if (manager->allocate_instance_at(spawn->request.instance)) {
                        instance = spawn->request.instance;
                    } else {
                        std::fprintf(stderr, "skysim: spawn refused — instance %d busy/invalid\n",
                                     spawn->request.instance);
                        spawn->done.set_value(result);
                        continue;
                    }
                } else {
                    instance = manager->allocate_instance();
                }
                try {
                    auto slot = make_slot(instance);
                    slot->battery = spawn->request.battery;
                    if (spawn->request.launch_process) {
                        slot->pid = manager->launch_sitl(instance);
                        if (!slot->pid.has_value()) {
                            world->remove_vehicle(slot->body_id);
                            if (docks) {
                                docks->remove(slot->vehicle_id); // make_slot gave it one
                            }
                            throw std::runtime_error("launch failed");
                        }
                    }
                    result = skysim::vehicle::SpawnResult{slot->vehicle_id, instance, 9002 + 10 * instance,
                                                          5760 + 10 * instance};
                    std::printf("skysim: spawned vehicle %u on instance %d\n", slot->vehicle_id, instance);
                    // Its own camera (TASK-2661), in place before its first frame is drawn. Checked
                    // against the bounds by the control plane already; with the camera off there is
                    // nothing to apply it to.
                    if (spawn->camera && render_service) {
                        std::string error;
                        const auto cam = render_service->configure(slot->vehicle_id, *spawn->camera, &error);
                        if (cam) {
                            std::printf("skysim: vehicle %u camera %dx%d, drawn at %dx%d @ %.1f fps%s\n",
                                        slot->vehicle_id, cam->picture_width, cam->picture_height,
                                        cam->normal.width, cam->normal.height, cam->normal.fps,
                                        cam->landing_available ? " (landing mode on)" : "");
                        } else {
                            std::fprintf(stderr, "skysim: vehicle %u camera not applied: %s\n",
                                         slot->vehicle_id, error.c_str());
                        }
                    }
                    fleet.push_back(std::move(slot));
                } catch (const std::exception &e) {
                    std::fprintf(stderr, "skysim: spawn failed: %s\n", e.what());
                    manager->release_instance(instance);
                }
                spawn->done.set_value(result);
            } else if (auto *despawn_cmd = std::get_if<skysim::api::DespawnCommand>(&cmd)) {
                bool found = false;
                for (size_t i = 0; i < fleet.size(); ++i) {
                    if (fleet[i]->vehicle_id == despawn_cmd->id) {
                        despawn(i);
                        found = true;
                        break;
                    }
                }
                despawn_cmd->done.set_value(found);
            } else if (auto *batt = std::get_if<skysim::api::BatteryResetCommand>(&cmd)) {
                // A swapped pack, not a recharge: the discharge counter starts again from
                // zero, which is what the aircraft's own coulomb count will do once the
                // ground crew tells it a new battery is fitted.
                bool found = false;
                for (auto &slot : fleet) {
                    if (slot->vehicle_id == batt->id) {
                        slot->battery_consumed_mah = 0.0;
                        // This vehicle's own full voltage: a swapped pack is the same pack
                        // type it was flying, not whatever a compiled-in constant says.
                        slot->battery_voltage_v = slot->battery.full_v;
                        found = true;
                        break;
                    }
                }
                batt->done.set_value(found);
            } else if (auto *path = std::get_if<skysim::api::PathCheckCommand>(&cmd)) {
                // A read of world geometry, so it belongs on the tick thread like the
                // mutations above — Jolt queries must not race PhysicsSystem::Update.
                skysim::api::PathCheckResult result;

                // Tile residency follows the vehicles, so a mission the fleet is not
                // near yet has no geometry loaded and would sweep clean through a city.
                // Stream the path's own tiles in first, passing the fleet positions too
                // so nothing a flying vehicle needs gets evicted.
                if (streamer) {
                    std::vector<skysim::core::Vec3> interest = path->waypoints_ned;
                    for (const auto &v : fleet) {
                        interest.push_back(v->state.pos_ned);
                    }
                    apply_tile_plan(streamer->update(interest));
                    result.geometry_available = streamer->resident_count() > 0;
                }

                if (world) {
                    result.hits = world->sweep_path(path->waypoints_ned, path->clearance_m);
                }
                path->done.set_value(std::move(result));
            }
        }
        manager->reap();
    }

    // Hand the render thread the poses it should draw from.
    //
    // Deliberately all this tick does for the camera: a few doubles under a
    // mutex. Rendering itself happens on RenderService's own thread against its
    // own copy of the static world, because a single row of raycasting costs
    // about as much as the entire tick budget.
    void publish_frames() {
        if (!render_service) {
            return;
        }
        const double now = world ? world->now() : 0.0;
        std::vector<skysim::render::RenderService::Pose> poses;
        poses.reserve(fleet.size());
        for (const auto &v : fleet) {
            skysim::render::RenderService::Pose pose{v->vehicle_id, v->state.pos_ned, v->state.quat_ned_frd,
                                                     now};
            const auto g = gimbal_of(*v);
            pose.gimbal = g.present;
            pose.gimbal_pitch_rad = g.pitch_rad;
            pose.gimbal_yaw_rad = g.yaw_rad;
            pose.home_north_m = v->home_ned[0];
            pose.home_east_m = v->home_ned[1];
            poses.push_back(pose);
        }
        render_service->publish_poses(std::move(poses));
    }

    // The gimbal as the frame just stepped drives it; called right after step_world, before
    // any endpoint is polled for the next tick.
    void latch_gimbals() {
        skysim::vehicle::GimbalConfig cfg;
        cfg.pitch_channel = opt.gimbal_pitch_channel;
        cfg.yaw_channel = opt.gimbal_yaw_channel;
        for (auto &v : fleet) {
            v->gimbal = v->connected ? skysim::vehicle::gimbal_from_servos(cfg, v->pending.pwm,
                                                                           v->pending.channel_count)
                                     : skysim::vehicle::GimbalState{};
        }
    }

    static const skysim::vehicle::GimbalState &gimbal_of(const VehicleSlot &v) { return v.gimbal; }

    void publish_snapshot() {
        std::vector<skysim::api::VehicleInfo> infos;
        infos.reserve(fleet.size());
        for (const auto &v : fleet) {
            skysim::api::VehicleInfo info;
            info.id = v->vehicle_id;
            info.instance = v->instance;
            info.json_port = 9002 + 10 * v->instance;
            info.mavlink_tcp_port = 5760 + 10 * v->instance;
            info.connected = v->connected;
            info.frozen = v->frozen;
            info.silent_ticks = static_cast<uint64_t>(v->silent_ticks);
            info.midair_collisions = v->state.midair_collisions;
            info.static_contacts = v->state.static_contacts;
            info.building_contacts = v->state.building_contacts;
            for (int k = 0; k < 3; ++k) {
                info.pos_ned[k] = v->state.pos_ned[k];
            }
            for (int k = 0; k < 4; ++k) {
                info.quat_ned_frd[k] = v->state.quat_ned_frd[k];
            }
            info.sim_time_s = world ? world->now() : 0.0;
            const auto g = gimbal_of(*v);
            info.gimbal = g.present;
            info.gimbal_pitch_rad = g.pitch_rad;
            info.gimbal_yaw_rad = g.yaw_rad;
            infos.push_back(info);
        }
        skysim::api::MetricsInfo m;
        const auto snap = metrics.snapshot();
        m.tick_p50_us = snap.p50_us;
        m.tick_p99_us = snap.p99_us;
        m.straggler_events = snap.straggler_events;
        m.freezes = snap.freezes;
        m.tick = world ? world->tick_index() : 0;
        m.sim_time_s = world ? world->now() : 0.0;
        m.vehicles = fleet.size();
        m.resident_tiles = streamer ? streamer->resident_count() : 0;
        std::lock_guard<std::mutex> lock(snapshot_mutex);
        vehicle_snapshot = std::move(infos);
        metrics_snapshot = m;
    }
};

// Per-tick work that is neither physics nor waiting, timed so it can be charged to the tick.
//
// It spends the same budget the vehicle's round trip needs, and leaving it out of the metric is
// how a box reporting a 73 us p99 could be missing a hundred deadlines a minute — the cost was
// real, it was just all in tile streaming. Measured fresh per iteration rather than accumulated:
// an iteration that does not go on to step is time spent waiting, and charging it to whichever
// tick eventually lands would put a whole barrier stall into one sample.
double timed_housekeeping(App &app) {
    const auto t0 = std::chrono::steady_clock::now();
    app.drain_commands();
    app.stream_tiles();
    return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
}

// Strict: barrier on every CONNECTED vehicle's next frame; not-yet-connected vehicles don't
// block (they sit on the ground until their SITL shows up). Abort on stalled barrier.
int run_strict(App &app, FILE *truth_log, FILE *record_log) {
    using clock = std::chrono::steady_clock;
    auto barrier_stalled_since = clock::now();
    bool barrier_was_complete = true;
    while (!g_stop.load(std::memory_order_relaxed)) {
        const double house_us = timed_housekeeping(app);

        bool all_fresh = true;
        bool any_fresh = false;
        bool any_connected = false;
        for (auto &v : app.fleet) {
            const bool fresh = poll_endpoint(*v);
            any_fresh = any_fresh || fresh;
            if (v->connected) {
                any_connected = true;
                if (!fresh) {
                    all_fresh = false;
                }
            }
        }
        const bool ready = all_fresh && any_fresh;
        if (!ready) {
            if (barrier_was_complete) {
                barrier_was_complete = false;
                barrier_stalled_since = clock::now();
            } else if (app.opt.strict_timeout_s > 0.0 && any_connected) {
                // Connected, not fresh: a genuine stall. Gated on anyone being connected
                // rather than on anyone being fresh, so that the case this abort exists for —
                // every SITL in the fleet going silent at once — is not the one case it sat
                // through in silence. Waiting indefinitely is still right before the first
                // vehicle arrives; a world with nothing in it is not stalled.
                const double stalled =
                    std::chrono::duration<double>(clock::now() - barrier_stalled_since).count();
                if (stalled > app.opt.strict_timeout_s) {
                    std::fprintf(stderr, "skysim: STRICT ABORT after %.1f s — stalled vehicles:\n", stalled);
                    for (const auto &v : app.fleet) {
                        if (v->connected && !v->has_pending_input) {
                            std::fprintf(stderr, "  vehicle %u (instance %d, udp %d): no frame\n",
                                         v->vehicle_id, v->instance, 9002 + 10 * v->instance);
                        }
                    }
                    return 3;
                }
            }
            std::this_thread::sleep_for(std::chrono::microseconds(50));
            continue;
        }
        barrier_was_complete = true;

        const auto t0 = clock::now();
        step_world(*app.world, app.fleet, app.opt.dt_s, truth_log, record_log);
        app.latch_gimbals();
        app.metrics.record_us(house_us + std::chrono::duration<double, std::micro>(clock::now() - t0).count());
        if (const int rc =
                send_replies(app.world.get(), app.fleet, app.opt, app.world->now(), app.io_pool.get())) {
            return rc;
        }
        app.publish_snapshot();
        app.publish_frames();
    }
    return 0;
}

// Poll every connected vehicle until all have a frame staged or the deadline passes.
//
// Frames are staged into the slot by poll_endpoint, so a vehicle that answers
// early is not polled again and the loop exits as soon as the slowest one lands.
void wait_for_frames(Fleet &fleet, std::chrono::steady_clock::time_point deadline) {
    for (;;) {
        bool all_ready = true;
        for (auto &v : fleet) {
            // Frozen slots stay `connected` — they are still ours, and still polled below so
            // they can thaw. But waiting on one here spends the whole grace period every
            // single tick on a vehicle the barrier has already agreed to step without, which
            // is the fleet-wide stall that freezing it was supposed to end.
            if (v->connected && !v->frozen && !poll_endpoint(*v)) {
                all_ready = false;
            }
        }
        if (all_ready || std::chrono::steady_clock::now() >= deadline) {
            return;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
}

// Interactive: pace to the wall clock, but never advance a vehicle without its frame.
//
// The tick is a barrier here exactly as it is in strict mode. What interactive adds is an
// exit from that barrier: a vehicle that stops answering for a real length of time is
// declared gone, frozen, and despawned after the grace, instead of stalling the fleet. That
// is what a long-lived server needs, where aircraft join and leave and one that dies must
// not take everybody else down with it.
//
// It used to add a second thing, and that second thing was the bug. A vehicle which missed its
// deadline had its last PWM replayed for up to k ticks so the world could keep to the wall
// clock. ArduPilot's JSON backend takes its entire scheduler rate from the timestamp we send it
// — SIM_JSON.cpp runs `adjust_frame_time(1.0 / deltat)` on every frame — so a tick stepped
// without the vehicle comes back as a doubled deltat and silently re-rates its control loop,
// describing motion it never commanded. It crashed every takeoff. Freezing a vehicle that was
// merely late is the same lie told louder.
//
// So a late vehicle now stalls its own tick. Lateness costs wall-clock pacing, which is
// recoverable and shows up in the metrics; it must never cost flight fidelity.
int run_interactive(App &app, FILE *truth_log, FILE *record_log) {
    using clock = std::chrono::steady_clock;
    const auto dt = std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(app.opt.dt_s));
    auto next_tick = clock::now() + dt;
    const uint64_t grace_ticks = static_cast<uint64_t>(app.opt.grace_s / app.opt.dt_s);
    // Ticks of silence before a vehicle is written off as gone.
    const int freeze_after = std::max(1, static_cast<int>(app.opt.straggler_timeout_s / app.opt.dt_s));
    // How often the API snapshot is refreshed while the world is held waiting for a straggler.
    constexpr auto kStallPublishPeriod = std::chrono::milliseconds(50);
    auto last_stall_publish = clock::now();
    // How much of a tick a late frame may use before it counts as late. Kept
    // under the period so a persistently slow vehicle still shows up as one
    // rather than silently stretching every tick.
    const auto frame_grace =
        std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(app.opt.dt_s * app.opt.frame_grace));

    while (!g_stop.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_until(next_tick);
        next_tick += dt;
        if (clock::now() > next_tick + 100 * dt) {
            next_tick = clock::now() + dt; // fell far behind (debugger, suspend): resync
        }

        const double house_us = timed_housekeeping(app);

        // Give a late frame a bounded moment to land before calling it late.
        //
        // take_latest() is non-blocking, so polling it once at the wall-clock
        // instant asks "has the frame arrived YET", when lockstep only cares
        // whether it arrives WITHIN the tick. A round trip a hair longer than
        // the period then reads as a straggler even though the frame lands
        // microseconds later — measured at 19% of ticks on an idle 16-core box,
        // with the vehicle holding stale PWM for every one of them.
        //
        // Waiting a fraction of the period costs nothing when frames are early
        // (the loop breaks out immediately) and converts most of those false
        // stragglers into ordinary on-time ticks.
        wait_for_frames(app.fleet, clock::now() + frame_grace);

        // A vehicle that has never connected, or that has already been written off, does not
        // block the tick. Anyone else we are still waiting on does.
        bool all_ready = true;
        for (auto &v : app.fleet) {
            if (poll_endpoint(*v)) {
                if (v->frozen) {
                    std::printf("skysim: vehicle %u thawed (frames resumed)\n", v->vehicle_id);
                    v->frozen = false;
                    app.world->set_frozen(v->body_id, false);
                }
                v->silent_ticks = 0;
            } else if (v->connected && !v->frozen) {
                ++v->silent_ticks;
                ++app.metrics.straggler_events;
                if (v->silent_ticks > freeze_after) {
                    std::printf("skysim: vehicle %u FROZEN after %.2f s without a frame\n",
                                v->vehicle_id, v->silent_ticks * app.opt.dt_s);
                    v->frozen = true;
                    v->frozen_at_tick = app.world->tick_index();
                    ++app.metrics.freezes;
                    app.world->set_frozen(v->body_id, true);
                } else {
                    all_ready = false; // still ours: wait for it rather than step without it
                }
            }
        }

        if (!all_ready) {
            // Hold the entire world at this tick. Time is shared — one Jolt world, one clock —
            // so there is no advancing the rest of the fleet without also advancing the vehicle
            // we are waiting on, and advancing that one without its frame is the thing that
            // crashes it.
            //
            // Keep publishing so /vehicles and /metrics stay live through the stall, but at a
            // polling rate rather than a tick rate: publish_snapshot sorts the whole tick-time
            // ring to get its quantiles, and nothing it reports moves while the world is held.
            // At the default 2 s timeout and 800 Hz that would be 1600 identical sorts.
            if (clock::now() - last_stall_publish > kStallPublishPeriod) {
                last_stall_publish = clock::now();
                app.publish_snapshot();
            }
            continue;
        }

        const auto t0 = clock::now();
        step_world(*app.world, app.fleet, app.opt.dt_s, truth_log, record_log);
        app.latch_gimbals();
        app.metrics.record_us(house_us + std::chrono::duration<double, std::micro>(clock::now() - t0).count());
        if (const int rc =
                send_replies(app.world.get(), app.fleet, app.opt, app.world->now(), app.io_pool.get())) {
            return rc;
        }

        // Grace expiry (reverse order: despawn erases by index).
        for (size_t i = app.fleet.size(); i-- > 0;) {
            auto &v = *app.fleet[i];
            if (v.frozen && app.world->tick_index() - v.frozen_at_tick > grace_ticks) {
                std::printf("skysim: vehicle %u exceeded grace (%.0f s frozen) — despawning\n", v.vehicle_id,
                            app.opt.grace_s);
                app.despawn(i);
            }
        }
        app.publish_snapshot();
        app.publish_frames();
    }
    return 0;
}

// M1 canned mode: kinematic on-ground reply, no world. Kept for protocol debugging.
int run_canned(App &app) {
    double canned_time = 0.0;
    while (!g_stop.load(std::memory_order_relaxed)) {
        bool any = false;
        for (auto &v : app.fleet) {
            if (poll_endpoint(*v)) {
                any = true;
            }
        }
        if (!any) {
            std::this_thread::sleep_for(std::chrono::microseconds(50));
            continue;
        }
        canned_time += app.opt.dt_s;
        if (const int rc = send_replies(nullptr, app.fleet, app.opt, canned_time, app.io_pool.get())) {
            return rc;
        }
    }
    return 0;
}

} // namespace

int main(int argc, char **argv) {
    App app;
    app.opt = parse_args(argc, argv);
    const Options &opt = app.opt;
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    std::signal(SIGPIPE, SIG_IGN);

    // Before anything heavier starts, so a bad spec fails the launch at once.
    auto radio_field = std::make_shared<skysim::radio::RadioField>();
    if (!opt.radio_spec.empty() && !load_radio_spec(opt.radio_spec, *radio_field)) {
        return 2;
    }

    skysim::core::WorldConfig wcfg;
    wcfg.dt_s = opt.dt_s;
    wcfg.wind_steady_ned = opt.wind_ned;
    wcfg.gust_sigma_mps = opt.gust_sigma;
    wcfg.gust_tau_s = opt.gust_tau;
    wcfg.rng_seed = opt.seed;
    wcfg.worker_threads = opt.physics_threads;
    const bool replaying = !opt.replay_servo.empty();
    if (!opt.canned) {
        app.world = std::make_unique<skysim::core::World>(wcfg);
        app.world->add_ground_plane();
        if (!opt.tiles.empty() && replaying) {
            // Replay: all tiles resident up front (streaming decisions are not on the tape).
            const size_t n = app.world->load_tiles(opt.tiles);
            std::printf("skysim: loaded %zu tile(s) from %s\n", n, opt.tiles.c_str());
            if (n == 0) {
                std::fprintf(stderr, "skysim: --tiles %s yielded no tiles\n", opt.tiles.c_str());
                return 1;
            }
        } else if (!opt.tiles.empty()) {
            try {
                skysim::terrain::StreamerConfig scfg;
                scfg.tile_dir = opt.tiles;
                scfg.keep_radius_m = opt.stream_radius_m;
                scfg.max_resident = static_cast<size_t>(opt.stream_max_resident);
                app.streamer = std::make_unique<skysim::terrain::TileStreamer>(scfg);
                std::printf("skysim: streaming %zu tile(s) from %s (radius %.0f m, max %d)\n",
                            app.streamer->tiles().size(), opt.tiles.c_str(), opt.stream_radius_m,
                            opt.stream_max_resident);
            } catch (const std::exception &e) {
                std::fprintf(stderr, "skysim: %s\n", e.what());
                return 1;
            }
        }
    }

    skysim::vehicle::ProcessConfig pcfg;
    pcfg.binary = opt.spawn_binary;
    pcfg.home = opt.spawn_home;
    pcfg.defaults = opt.spawn_defaults;
    pcfg.run_dir = "/tmp/skysim_managed";
    app.manager = std::make_unique<skysim::vehicle::VehicleManager>(opt.base_instance, pcfg);
    if (opt.io_threads > 0) {
        app.io_pool = std::make_unique<skysim::core::ThreadPool>(opt.io_threads);
    }

    // One render scene — tiles, textures, shadow grid — shared by the camera feed and the
    // survey still camera, built when either is on. Stills need the control plane to be asked
    // for, so a headless replay or canned run never pays for loading it.
    const bool stills_on = opt.api_port > 0 && !opt.canned && !replaying && opt.still_concurrency > 0;
    std::shared_ptr<const skysim::render::RasterScene> render_scene;
    size_t render_tiles = 0;
    if (opt.camera_fps > 0.0 || stills_on) {
        render_scene = skysim::render::load_scene(opt.tiles, opt.texture_seed, &render_tiles);
        std::printf(
            "skysim: render scene has %zu tile(s), %zu triangle(s), %zu building(s), texture seed %u\n",
            render_tiles, render_scene->triangle_count(), render_scene->building_count(), opt.texture_seed);
    }

    if (opt.dock_markers && render_scene) {
        skysim::render::DockMarkerSpec spec;
        spec.outer_size_m = opt.dock_marker_size_m;
        spec.inner_size_m = opt.dock_inner_size_m;
        app.docks = std::make_shared<skysim::render::DockBoard>(spec, opt.dock_markers_visible);
        const bool own = opt.dock_markers_visible == skysim::render::DockVisibility::kOwn;
        std::printf(
            "skysim: e-ArUco dock marker (DICT_7X7_250 %d in %d) %.3f m / %.3f m at every vehicle's dock, "
            "drawn by the cameras only: %s (--dock-markers-visible %s)\n",
            skysim::render::DockMarkerSpec::kInnerId, skysim::render::DockMarkerSpec::kOuterId,
            spec.outer_size_m, spec.inner_size_m,
            own ? "a vehicle's cameras draw its own dock alone" : "every vehicle's cameras draw every dock",
            skysim::render::dock_visibility_name(opt.dock_markers_visible));
    }

    if (opt.camera_fps > 0.0) {
        if (opt.camera_width <= 0 || opt.camera_height <= 0) {
            std::fprintf(stderr, "skysim: --camera-size must be WxH with both > 0\n");
            return 1;
        }
        skysim::render::RenderService::Config rcfg;
        rcfg.camera.width = opt.camera_width;
        rcfg.camera.height = opt.camera_height;
        rcfg.camera.fov_deg = opt.camera_fov_deg;
        rcfg.camera.pitch_deg = opt.camera_pitch_deg;
        rcfg.camera.max_range_m = opt.camera_range_m;
        rcfg.scene = render_scene;
        rcfg.scene_tiles = render_tiles;
        rcfg.docks = app.docks;
        rcfg.fps = opt.camera_fps;
        rcfg.quality = opt.camera_quality;
        // How the pictures are drawn (TASK-2661): the normal mode at --camera-draw-size, scaled up
        // to the picture; the landing mode, full size by default, near the dock only, where a
        // landing reads the marker; the rest rate for a vehicle sitting still on the ground.
        // Every mode keeps the picture's aspect ratio, or its pictures would be stretched.
        const auto aspect_ok = [&opt](int w, int h) {
            if (w == 0) {
                return true;
            }
            const double picture = static_cast<double>(opt.camera_width) / opt.camera_height;
            return std::fabs(static_cast<double>(w) / h - picture) <= 0.01 * picture;
        };
        if (!aspect_ok(opt.camera_draw_width, opt.camera_draw_height) ||
            !aspect_ok(opt.camera_landing_width, opt.camera_landing_height)) {
            std::fprintf(stderr,
                         "skysim: --camera-draw-size and --camera-landing-size must have --camera-size's "
                         "aspect ratio (%dx%d)\n",
                         opt.camera_width, opt.camera_height);
            return 1;
        }
        rcfg.draw_width = opt.camera_draw_width;
        rcfg.draw_height = opt.camera_draw_height;
        rcfg.rest_fps = opt.camera_rest_fps;
        if (opt.camera_landing) {
            rcfg.landing.width = opt.camera_landing_width;
            rcfg.landing.height = opt.camera_landing_height;
            rcfg.landing.fps = opt.camera_landing_fps > 0.0 ? opt.camera_landing_fps : opt.camera_fps;
            rcfg.landing.bitrate_kbps = opt.camera_landing_bitrate_kbps;
        }
        rcfg.landing_radius_m = opt.camera_landing_radius_m;
        rcfg.landing_height_m = opt.camera_landing_height_m;
        rcfg.ground_cache_bytes = static_cast<size_t>(opt.camera_cache_mb) * 1024 * 1024;
        const int cpus = skysim::core::available_cpus();
        rcfg.threads = opt.camera_threads > 0 ? opt.camera_threads : skysim::core::default_render_threads(cpus);
        // A line every ten seconds while frames are being drawn — achieved fps against the
        // target, and what a frame costs — so a laggy stream is diagnosable from the log.
        rcfg.log_every_s = 10.0;

        // H.264 over RTSP, built before the renderer that feeds it. Its hooks reach the renderer
        // through render_ptr, which is null until the renderer exists: a session that starts
        // playing in between just waits for the next refresh.
        if (opt.rtsp_port > 0 && opt.api_port > 0 && !opt.canned && !replaying) {
            skysim::video::VideoService::Config vcfg;
            vcfg.width = opt.camera_width;
            vcfg.height = opt.camera_height;
            vcfg.max_fps = opt.camera_fps;
            vcfg.defaults.fps = opt.camera_fps;
            vcfg.threads = opt.video_threads > 0 ? opt.video_threads : std::clamp((cpus + 3) / 4, 1, 4);
            vcfg.rtsp.bind = opt.rtsp_bind.empty() ? opt.api_bind : opt.rtsp_bind;
            vcfg.rtsp.port = opt.rtsp_port;
            skysim::video::VideoService::Hooks vhooks;
            vhooks.resolve = [&app](int instance) { return app.vehicle_at(instance); };
            vhooks.want_frames = [&app](uint32_t id) {
                if (auto *r = app.render_ptr.load()) {
                    r->want_video(id);
                }
            };
            // The vehicle's stream now (TASK-2661): its picture size, its mode's rate and bitrate.
            vhooks.camera = [&app](uint32_t id) {
                auto *r = app.render_ptr.load();
                return r != nullptr ? r->stream_mode(id) : skysim::render::CameraMode{};
            };
            try {
                app.video_service = std::make_unique<skysim::video::VideoService>(vcfg, std::move(vhooks));
            } catch (const std::exception &e) {
                std::fprintf(stderr, "skysim: %s (--rtsp-port 0 turns RTSP off)\n", e.what());
                return 1;
            }
            rcfg.video_sink =
                [v = app.video_service.get()](uint32_t id, std::shared_ptr<const skysim::render::Image> image,
                                              double sim_time_s, std::chrono::steady_clock::time_point at,
                                              const skysim::render::CameraMode &mode) {
                    v->on_frame(id, std::move(image), sim_time_s, at, mode);
                };
        }
        app.render_service = std::make_unique<skysim::render::RenderService>(rcfg);
        app.render_ptr.store(app.render_service.get());
        std::printf("skysim: camera %dx%d @ %.1f fps on %d render thread(s) (%s; %d CPU(s) available), "
                    "drawn only while somebody is watching; lit-ground cache %d MB\n",
                    opt.camera_width, opt.camera_height, opt.camera_fps, app.render_service->threads(),
                    opt.camera_threads > 0 ? "--camera-threads" : "default", cpus, opt.camera_cache_mb);
        {
            const int draw_w = opt.camera_draw_width > 0 ? opt.camera_draw_width : opt.camera_width;
            const int draw_h = opt.camera_draw_height > 0 ? opt.camera_draw_height : opt.camera_height;
            std::printf(
                "skysim: camera modes: normal drawn at %dx%d%s; landing %s; at rest %s\n", draw_w, draw_h,
                draw_w != opt.camera_width ? " and scaled to the picture" : "",
                opt.camera_landing
                    ? (std::to_string(rcfg.landing.width > 0 ? rcfg.landing.width : opt.camera_width) + "x" +
                       std::to_string(rcfg.landing.height > 0 ? rcfg.landing.height : opt.camera_height) +
                       " @ " + std::to_string(static_cast<int>(rcfg.landing.fps + 0.5)) + " fps" +
                       (rcfg.landing.bitrate_kbps > 0
                            ? ", " + std::to_string(rcfg.landing.bitrate_kbps) + " kbps"
                            : std::string()) +
                       ", within " + std::to_string(static_cast<int>(rcfg.landing_radius_m)) +
                       " m of a vehicle's dock and " +
                       std::to_string(static_cast<int>(rcfg.landing_height_m)) + " m above it, in the air")
                          .c_str()
                    : "off (--camera-landing-size)",
                opt.camera_rest_fps > 0.0 ? (std::to_string(opt.camera_rest_fps) + " fps").c_str()
                                          : "its mode's rate (--camera-rest-fps)");
        }
        if (app.video_service) {
            const auto defaults = skysim::video::apply_patch({}, {}, opt.camera_fps);
            std::printf("skysim: camera H.264 (constrained baseline) on rtsp://%s:%d/instances/<n>/main.264, "
                        "%d kbps, %.0f fps, keyframe every %.1f s, %d encoder thread(s), encoded only while "
                        "somebody plays it%s\n",
                        (opt.rtsp_bind.empty() ? opt.api_bind : opt.rtsp_bind).c_str(),
                        app.video_service->port(), defaults.bitrate_kbps, opt.camera_fps,
                        defaults.keyframe_interval_s, app.video_service->stats().threads,
                        opt.camera_mjpeg ? "; camera.mjpg too" : "; camera.mjpg off");
        }
    }

    if (stills_on) {
        skysim::render::StillService::Config scfg;
        scfg.concurrency = opt.still_concurrency;
        scfg.threads = opt.still_threads;
        app.still_service = std::make_unique<skysim::render::StillService>(render_scene, scfg);
        std::printf("skysim: survey stills on demand, %d at a time on %d background thread(s) each\n",
                    opt.still_concurrency, opt.still_threads);
    }

    if (replaying && (!app.world || opt.time_mode != "strict")) {
        std::fprintf(stderr, "skysim: --replay-servo requires strict mode + jolt physics\n");
        return 1;
    }

    for (int i = 0; i < opt.vehicles; ++i) {
        const int instance = app.manager->allocate_instance();
        if (opt.canned) {
            auto slot = std::make_unique<VehicleSlot>(instance);
            slot->vehicle_id = app.next_vehicle_id++;
            app.fleet.push_back(std::move(slot));
        } else if (replaying) {
            auto slot = std::make_unique<VehicleSlot>();
            slot->vehicle_id = app.next_vehicle_id++;
            slot->instance = instance;
            skysim::core::VehicleBodyParams bp;
            bp.mass_kg = slot->params.mass_kg;
            bp.inertia_diag_frd = {slot->params.inertia_diag[0], slot->params.inertia_diag[1],
                                   slot->params.inertia_diag[2]};
            bp.start_pos_ned = {0.0, opt.spacing_m * i, -(bp.half_extents_frd[2] + 0.005)};
            slot->body_id = app.world->add_vehicle(bp);
            slot->state = app.world->get_state(slot->body_id);
            app.fleet.push_back(std::move(slot));
        } else {
            app.fleet.push_back(app.make_slot(instance));
        }
        if (app.fleet.back()->endpoint) {
            std::printf("skysim: vehicle %u listening on udp %d\n", app.fleet.back()->vehicle_id,
                        app.fleet.back()->endpoint->port());
        }
    }
    std::printf("skysim: %d vehicle(s), dt=%.6f s, %s mode, %s\n", opt.vehicles, opt.dt_s,
                opt.time_mode.c_str(), opt.canned ? "canned physics (M1)" : "jolt physics");
    std::fflush(stdout);

    FILE *truth_log = nullptr;
    if (!opt.truth_log.empty()) {
        truth_log = std::fopen(opt.truth_log.c_str(), "w");
        if (truth_log == nullptr) {
            std::fprintf(stderr, "skysim: cannot open --truth-log %s\n", opt.truth_log.c_str());
            return 1;
        }
        std::fprintf(truth_log, "t,vehicle,north,east,down\n");
    }
    FILE *record_log = nullptr;
    if (!opt.record_servo.empty()) {
        record_log = std::fopen(opt.record_servo.c_str(), "w");
        if (record_log == nullptr) {
            std::fprintf(stderr, "skysim: cannot open --record-servo %s\n", opt.record_servo.c_str());
            return 1;
        }
    }

    std::unique_ptr<skysim::api::ControlServer> api;
    if (opt.api_port > 0 && !opt.canned && !replaying) {
        skysim::api::ControlServer::Snapshots snaps;
        snaps.vehicles = [&app] {
            std::lock_guard<std::mutex> lock(app.snapshot_mutex);
            return app.vehicle_snapshot;
        };
        snaps.metrics = [&app] {
            skysim::api::MetricsInfo m;
            {
                std::lock_guard<std::mutex> lock(app.snapshot_mutex);
                m = app.metrics_snapshot;
            }
            // Read when asked rather than copied every tick: the render thread keeps these.
            if (app.render_service) {
                const auto s = app.render_service->stats();
                m.camera.enabled = true;
                m.camera.target_fps = s.target_fps;
                m.camera.fps = s.fps;
                m.camera.watched = s.watched;
                m.camera.render_ms = s.render_ms;
                m.camera.render_ms_max = s.render_ms_max;
                m.camera.encode_ms = s.encode_ms;
                m.camera.threads = s.threads;
                m.camera.frames_drawn = s.frames_drawn;
                m.camera.frames_reused = s.frames_reused;
                m.camera.landing = s.landing;
                m.camera.ground_cache_mb = static_cast<double>(s.ground.bytes) / (1024.0 * 1024.0);
                m.camera.ground_cache_budget_mb =
                    static_cast<double>(s.ground.budget_bytes) / (1024.0 * 1024.0);
                m.camera.ground_tiles_built = s.ground.tiles_built;
            }
            if (app.video_service) {
                const auto v = app.video_service->stats();
                m.video.enabled = true;
                m.video.rtsp_port = v.port;
                m.video.streams = v.streams;
                m.video.clients = v.clients;
                m.video.threads = v.threads;
                m.video.encode_ms = v.encode_ms;
                m.video.frames_encoded = v.frames_encoded;
                m.video.frames_dropped = v.frames_dropped + v.rtsp.frames_dropped;
                m.video.keyframes = v.keyframes;
                m.video.bytes_sent = v.rtsp.bytes_sent;
            }
            return m;
        };
        snaps.mjpeg = opt.camera_mjpeg;
        if (app.video_service) {
            snaps.camera_encoder = [&app](const skysim::api::VehicleInfo &v,
                                          const skysim::api::EncoderPatch *patch) {
                skysim::video::EncoderPatch p;
                if (patch != nullptr) {
                    p.bitrate_kbps = patch->bitrate_kbps;
                    p.fps = patch->fps;
                    p.keyframe_interval_s = patch->keyframe_interval_s;
                }
                const auto st = app.video_service->encoder(v.instance, v.id, patch != nullptr ? &p : nullptr);
                skysim::api::EncoderState out;
                out.bitrate_kbps = st.settings.bitrate_kbps;
                out.fps = st.settings.fps;
                out.keyframe_interval_s = st.settings.keyframe_interval_s;
                out.width = st.width;
                out.height = st.height;
                out.max_fps = st.max_fps;
                out.clients = st.clients;
                return out;
            };
            snaps.camera_keyframe = [&app](const skysim::api::VehicleInfo &v) {
                return app.video_service->request_keyframe(v.instance, v.id);
            };
        }
        // Asking is what gets a vehicle drawn: RenderService::frame puts it on the draw list.
        snaps.camera_frame = [&app](uint32_t id, uint64_t after_seq, std::chrono::milliseconds wait) {
            return app.render_service ? app.render_service->frame(id, after_seq, wait)
                                      : skysim::render::FrameStore::Frame{};
        };
        if (app.still_service) {
            // The pose comes from the snapshot the route already read: position, attitude and
            // time from one tick, copied out under the snapshot lock. Nothing here touches the
            // tick thread, and the render runs on the still service's own workers.
            snaps.still = [&app](const skysim::api::VehicleInfo &v, const skysim::render::StillRequest &r) {
                skysim::render::StillPose pose;
                pose.position_ned = {v.pos_ned[0], v.pos_ned[1], v.pos_ned[2]};
                pose.quat_ned_frd = {v.quat_ned_frd[0], v.quat_ned_frd[1], v.quat_ned_frd[2],
                                     v.quat_ned_frd[3]};
                pose.sim_time_s = v.sim_time_s;
                pose.gimbal = v.gimbal;
                pose.gimbal_pitch_rad = v.gimbal_pitch_rad;
                pose.gimbal_yaw_rad = v.gimbal_yaw_rad;
                // The docks this vehicle's cameras see: its own alone unless --dock-markers-visible
                // all (TASK-2609), exactly as its pilot camera draws them.
                const auto markers = app.docks ? app.docks->seen_by(v.id) : nullptr;
                return app.still_service->render(pose, r, markers.get());
            };
        }
        if (app.render_service) {
            // A vehicle's own camera (TASK-2661): read, or changed through RenderService, which
            // checks the bounds and takes effect on the vehicle's next frame.
            snaps.camera_settings =
                [&app](const skysim::api::VehicleInfo &v, const skysim::render::CameraSettingsPatch *patch,
                       std::string *error) -> std::optional<skysim::render::VehicleCameraState> {
                if (patch == nullptr) {
                    return app.render_service->camera(v.id);
                }
                return app.render_service->configure(v.id, *patch, error);
            };
            skysim::api::CameraIntrinsics &ci = snaps.camera_intrinsics;
            ci.enabled = true;
            ci.width = opt.camera_width;
            ci.height = opt.camera_height;
            ci.hfov_deg = opt.camera_fov_deg;
            ci.fps = opt.camera_fps;
            ci.mount_pitch_deg = opt.camera_pitch_deg;
            if (app.video_service) {
                ci.stream_width = opt.camera_width & ~1;
                ci.stream_height = opt.camera_height & ~1;
            }
        }
        if (app.docks) {
            // Ground truth for the dock routes. The layout comes off the marker's texture, which
            // the first such request builds if no camera has drawn a marker yet.
            snaps.dock = [&app](const skysim::api::VehicleInfo &v, const skysim::render::DockPatch *patch,
                                bool reset) -> std::optional<skysim::api::DockState> {
                const auto tex = skysim::render::dock_marker_texture(app.docks->spec());
                if (!tex) {
                    return std::nullopt;
                }
                const auto entry =
                    reset ? app.docks->reset(v.id)
                          : (patch != nullptr ? app.docks->update(v.id, *patch) : app.docks->get(v.id));
                if (!entry) {
                    return std::nullopt;
                }
                skysim::api::DockState out;
                out.pose = entry->pose;
                out.home = entry->home;
                out.at_home = entry->at_home;
                out.layout = tex->layout();
                return out;
            };
        }
        skysim::api::ControlServer::RadioConfig radio_cfg;
        radio_cfg.field = radio_field;
        radio_cfg.home = spawn_anchor(opt.spawn_home);
        if (!radio_cfg.home) {
            std::fprintf(stderr,
                         "skysim: --spawn-home %s is not lat,lon[,alt,yaw]: vehicles carry no radio values\n",
                         opt.spawn_home.c_str());
        }
        api = std::make_unique<skysim::api::ControlServer>(opt.api_bind, opt.api_port, app.queue,
                                                           std::move(snaps), std::move(radio_cfg));
    }

    int rc = 0;
    if (replaying) {
        rc = run_replay(opt, *app.world, app.fleet, truth_log);
    } else if (opt.canned) {
        rc = run_canned(app);
    } else if (opt.time_mode == "interactive") {
        rc = run_interactive(app, truth_log, record_log);
    } else {
        rc = run_strict(app, truth_log, record_log);
    }

    api.reset(); // stop HTTP thread before tearing down the fleet it reads
    if (app.video_service) {
        app.video_service->stop(); // its threads read the snapshot and call the renderer
    }
    if (truth_log != nullptr) {
        std::fclose(truth_log);
    }
    if (record_log != nullptr) {
        std::fclose(record_log);
    }
    for (size_t i = 0; i < app.fleet.size(); ++i) {
        std::printf("skysim: vehicle %zu ticks=%llu gaps=%llu reboots=%llu bad=%llu\n", i,
                    static_cast<unsigned long long>(app.fleet[i]->ticks),
                    static_cast<unsigned long long>(app.fleet[i]->gaps),
                    static_cast<unsigned long long>(app.fleet[i]->reboots),
                    static_cast<unsigned long long>(app.fleet[i]->bad));
    }
    return rc;
}
