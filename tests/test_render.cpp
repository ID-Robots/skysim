// Camera render acceptance: put one real cooked building in a world, point a
// camera at it, and check the picture says what the world says — sky above the
// horizon, ground below it, and the building where the building is.
//
// Asserting on pixels rather than on "it produced bytes" is the point: a renderer
// that silently draws a blank frame, or draws the world upside down, still
// produces a valid JPEG.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "core/world.h"
#include "core/thread_pool.h"
#include "render/frame_store.h"
#include "render/ground_cache.h"
#include "render/jpeg.h"
#include "render/raster.h"
#include "render/render_service.h"
#include "render/renderer.h"
#include "render/resize.h"
#include "terrain/cook.h"

namespace {

int g_failures = 0;

#define CHECK(cond)                                                                                          \
    do {                                                                                                     \
        if (!(cond)) {                                                                                       \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                      \
            ++g_failures;                                                                                    \
        }                                                                                                    \
    } while (0)

// Same box the collision test uses: NED north 30..70, east -20..20, roof at z=-30.
void write_test_obj(const std::filesystem::path &path) {
    std::ofstream f(path);
    const double x0 = -20, x1 = 20, y0 = 30, y1 = 70, z0 = 0, z1 = 30;
    f << "v " << x0 << " " << y0 << " " << z0 << "\nv " << x1 << " " << y0 << " " << z0 << "\n"
      << "v " << x1 << " " << y1 << " " << z0 << "\nv " << x0 << " " << y1 << " " << z0 << "\n"
      << "v " << x0 << " " << y0 << " " << z1 << "\nv " << x1 << " " << y0 << " " << z1 << "\n"
      << "v " << x1 << " " << y1 << " " << z1 << "\nv " << x0 << " " << y1 << " " << z1 << "\n";
    const int quads[6][4] = {{1, 2, 6, 5}, {2, 3, 7, 6}, {3, 4, 8, 7},
                             {4, 1, 5, 8}, {5, 6, 7, 8}, {4, 3, 2, 1}};
    for (const auto &q : quads) {
        f << "f " << q[0] << " " << q[1] << " " << q[2] << "\n";
        f << "f " << q[0] << " " << q[2] << " " << q[3] << "\n";
    }
}

skysim::core::WorldConfig cfg() {
    skysim::core::WorldConfig c;
    c.dt_s = 1.0 / 800.0;
    c.worker_threads = 2;
    return c;
}

struct Px {
    int r, g, b;
};

Px pixel(const skysim::render::Image &img, int x, int y) {
    const size_t i = (static_cast<size_t>(y) * img.width + x) * 3;
    return {img.rgb[i], img.rgb[i + 1], img.rgb[i + 2]};
}

// CPU time the calling thread has used, in seconds.
double thread_cpu_s() {
    timespec ts{};
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

// CPU time this process has used, all threads, in seconds.
double process_cpu_s() {
    timespec ts{};
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

// Sky is the only thing in the palette that is markedly more blue than red.
bool looks_like_sky(const Px &p) { return p.b > p.r + 20; }

// Ground is the only thing in the palette with a green cast; building grey has
// its three channels within a few counts of each other.
bool looks_like_ground(const Px &p) { return p.g > p.b + 12; }

// --- TASK-2661: the lit-ground cache and the fast ground path --------------------------------

double mean_abs_diff(const skysim::render::Image &a, const skysim::render::Image &b, int *max_diff) {
    double sum = 0.0;
    int worst = 0;
    for (size_t i = 0; i < a.rgb.size() && i < b.rgb.size(); ++i) {
        const int d = std::abs(static_cast<int>(a.rgb[i]) - static_cast<int>(b.rgb[i]));
        sum += d;
        worst = std::max(worst, d);
    }
    if (max_diff != nullptr) {
        *max_diff = worst;
    }
    return a.rgb.empty() ? 0.0 : sum / static_cast<double>(a.rgb.size());
}

void test_ground_cache(const std::filesystem::path &tiles) {
    using skysim::render::GroundCache;
    const auto scene = skysim::render::load_scene(tiles.string(), 1);

    // Levels: a footprint of exactly 2^k m reads level k alone; between two, the blend follows
    // log2 (to the polynomial's 2e-4); outside the pyramid, the end levels.
    int level = 0;
    double blend = 0.0;
    GroundCache::level_of(0.03125, level, blend);
    CHECK(level == -5 && blend < 1e-9);
    GroundCache::level_of(0.03125 * std::sqrt(2.0), level, blend);
    CHECK(level == -5 && std::fabs(blend - 0.5) < 1e-3);
    GroundCache::level_of(1e-9, level, blend);
    CHECK(level == GroundCache::kMinLevel && blend == 0.0);
    GroundCache::level_of(1e9, level, blend);
    CHECK(level == GroundCache::kMaxLevel && blend == 0.0);
    GroundCache::level_of(std::nan(""), level, blend);
    CHECK(level == GroundCache::kMinLevel && blend == 0.0);
    GroundCache::level_of(-1.0, level, blend);
    CHECK(level == GroundCache::kMinLevel && blend == 0.0);
    double worst = 0.0;
    for (double fp = 1e-3; fp < 100.0; fp *= 1.37) {
        GroundCache::level_of(fp, level, blend);
        worst = std::max(worst, std::fabs(level + blend - std::log2(fp)));
    }
    CHECK(worst < 2e-3);

    // A texel centre, at that level's own footprint, is the direct colour to 16-bit precision:
    // the cache stores what WorldTexture and the light give, nothing else.
    GroundCache cache(scene, 0);
    cache.begin_frame();
    GroundCache::Sampler sampler(cache);
    double max_err = 0.0;
    for (int k = 0; k < 200; ++k) {
        const int lv = -9 + k % 12;
        const double size = std::ldexp(1.0, lv);
        const double n = (std::floor(-37.0 + k * 3.7) + 0.5) * size;
        const double e = (std::floor(51.0 - k * 2.3) + 0.5) * size;
        double cached[3];
        double direct[3];
        sampler.lit(n, e, size, cached);
        scene->lit_colour(skysim::render::kSurfaceGround, {n, e, 0.0}, size, direct);
        for (int c = 0; c < 3; ++c) {
            max_err = std::max(max_err, std::fabs(cached[c] - direct[c]));
        }
    }
    CHECK(max_err < 2e-4);

    // Across a frame's worth of points and footprints the colour is the direct one, softened a
    // little: the mean error is a fraction of a byte, and a byte is 1/255.
    double sum = 0.0;
    int count = 0;
    for (int k = 0; k < 4000; ++k) {
        const double n = -40.0 + 0.0197 * k * 7.0;
        const double e = 25.0 - 0.0131 * k * 5.0;
        const double fp = 0.004 * std::pow(1.0009, k);
        double cached[3];
        double direct[3];
        cache.begin_frame(1 << 30); // all the optional tiles it wants: the full trilinear blend
        GroundCache::Sampler s2(cache);
        s2.lit(n, e, fp, cached);
        scene->lit_colour(skysim::render::kSurfaceGround, {n, e, 0.0}, fp, direct);
        for (int c = 0; c < 3; ++c) {
            sum += std::fabs(cached[c] - direct[c]);
            ++count;
        }
    }
    std::printf("test_render: ground cache mean |cached - direct| = %.4f (%.2f of a byte)\n", sum / count,
                sum / count * 255.0);
    CHECK(sum / count < 0.03);

    // Without a budget for the lighter level, a pixel between two levels reads the heavier alone;
    // with one, both: the two differ only by that level's weight.
    {
        GroundCache lean(scene, 0);
        lean.begin_frame(0);
        GroundCache::Sampler a(lean);
        double heavy_only[3];
        const double fp = 0.03125 * std::pow(2.0, 0.3); // blend 0.3: level -5 heavier
        a.lit(3.3, -7.1, fp, heavy_only);
        GroundCache full(scene, 0);
        full.begin_frame(1 << 30);
        GroundCache::Sampler b(full);
        double both[3];
        b.lit(3.3, -7.1, fp, both);
        double level5[3];
        GroundCache::Sampler c5(full);
        c5.lit(3.3, -7.1, 0.03125, level5); // level -5 alone (blend 0)
        for (int c = 0; c < 3; ++c) {
            CHECK(std::fabs(heavy_only[c] - level5[c]) < 1e-9);
        }
        CHECK(lean.stats().tiles < full.stats().tiles);
        (void)both;
    }

    // The budget holds: past it the least recently used tiles go, and a tile built again is the
    // same tile (texels are a function of where they are, not of when).
    {
        GroundCache small(scene, 20 * GroundCache::kTileBytes);
        double first[3];
        small.begin_frame();
        {
            GroundCache::Sampler a(small);
            a.lit(0.37, 0.41, 0.01, first);
        }
        for (int f = 0; f < 40; ++f) {
            small.begin_frame();
            GroundCache::Sampler a(small);
            double c[3];
            a.lit(100.0 + f * 50.0, -200.0, 0.01, c); // somewhere new each frame
            small.evict();
        }
        CHECK(small.stats().tiles * GroundCache::kTileBytes <= 20 * GroundCache::kTileBytes);
        CHECK(small.stats().tiles_evicted > 0);
        small.begin_frame();
        GroundCache::Sampler a(small);
        double again[3];
        a.lit(0.37, 0.41, 0.01, again);
        for (int c = 0; c < 3; ++c) {
            CHECK(again[c] == first[c]);
        }
    }

    // Four threads filling the same new ground at once agree with one thread doing it alone.
    {
        GroundCache shared(scene, 0);
        shared.begin_frame(1 << 30);
        std::vector<double> results(4 * 300 * 3);
        std::vector<std::thread> threads;
        for (int t = 0; t < 4; ++t) {
            threads.emplace_back([&, t] {
                GroundCache::Sampler s(shared);
                for (int k = 0; k < 300; ++k) {
                    s.lit(-5.0 + k * 0.037, 7.0 - k * 0.029, 0.006 + k * 1e-4, &results[(t * 300 + k) * 3]);
                }
            });
        }
        for (auto &th : threads) {
            th.join();
        }
        GroundCache alone(scene, 0);
        alone.begin_frame(1 << 30);
        GroundCache::Sampler s(alone);
        bool same = true;
        for (int k = 0; k < 300; ++k) {
            double c[3];
            s.lit(-5.0 + k * 0.037, 7.0 - k * 0.029, 0.006 + k * 1e-4, c);
            for (int t = 0; t < 4; ++t) {
                for (int ch = 0; ch < 3; ++ch) {
                    same = same && results[(t * 300 + k) * 3 + ch] == c[ch];
                }
            }
        }
        CHECK(same);
    }

    // prepare(): the tiles a frame reads, gathered ahead (tiles_for) and built across a pool, are
    // the tiles drawing then reads — it builds none of its own — with the same colours as tiles
    // built on demand. Negative coordinates included: the keys round-trip.
    {
        GroundCache pre(scene, 0);
        pre.begin_frame(1 << 30);
        std::vector<uint64_t> required;
        std::vector<uint64_t> optional;
        const auto point = [](int k, double &n, double &e, double &fp) {
            n = -3.0 + k * 0.05;
            e = -7.0 + k * 0.031;
            fp = 0.01 + k * 1e-4;
        };
        for (int k = 0; k < 200; ++k) {
            double n, e, fp;
            point(k, n, e, fp);
            GroundCache::tiles_for(n, e, fp, required, optional);
        }
        skysim::core::ThreadPool helpers(3);
        pre.prepare(required, optional, &helpers);
        const uint64_t built = pre.stats().tiles_built;
        CHECK(built > 0);
        GroundCache lazy(scene, 0);
        lazy.begin_frame(1 << 30);
        GroundCache::Sampler a(pre);
        GroundCache::Sampler b(lazy);
        bool same = true;
        for (int k = 0; k < 200; ++k) {
            double n, e, fp;
            point(k, n, e, fp);
            double ca[3];
            double cb[3];
            a.lit(n, e, fp, ca);
            b.lit(n, e, fp, cb);
            same = same && ca[0] == cb[0] && ca[1] == cb[1] && ca[2] == cb[2];
        }
        CHECK(same);
        CHECK(pre.stats().tiles_built == built);
        CHECK(lazy.stats().tiles_built == built);
    }

    // The rasteriser with the cache draws the picture without it, to within a byte or two: looking
    // straight down at the ground and the building's roof, and obliquely across the city. Once the
    // cache is warm the picture does not depend on the thread count.
    skysim::render::CameraConfig cc;
    cc.width = 320;
    cc.height = 180;
    cc.fov_deg = 81.0;
    cc.pitch_deg = -90.0;
    skysim::core::ThreadPool pool(3);
    for (const auto &[pitch, position] : {std::pair{-90.0, skysim::core::Vec3{20.0, 0.0, -40.0}},
                                          std::pair{-20.0, skysim::core::Vec3{-60.0, 0.0, -25.0}}}) {
        cc.pitch_deg = pitch;
        const skysim::render::Camera camera(cc, position, {1.0, 0.0, 0.0, 0.0});
        const skysim::render::Rasterizer raster(cc);
        std::vector<int32_t> plain_surface;
        std::vector<int32_t> cached_surface;
        const auto plain = raster.render(*scene, camera, nullptr, &plain_surface);
        GroundCache warm(scene, 0);
        skysim::render::Image cached;
        for (int f = 0; f < 30; ++f) { // until the lighter levels are built too
            cached = raster.render(*scene, camera, &pool, &cached_surface, true, nullptr, &warm);
        }
        int max_diff = 0;
        const double mean = mean_abs_diff(plain, cached, &max_diff);
        std::printf("test_render: pitch %.0f: cached vs direct mean %.3f, max %d levels\n", pitch, mean,
                    max_diff);
        CHECK(mean < 1.5);
        CHECK(plain_surface == cached_surface);
        const auto serial = raster.render(*scene, camera, nullptr, nullptr, true, nullptr, &warm);
        CHECK(serial.rgb == cached.rgb);
    }
}

// --- TASK-2661: per-vehicle camera modes and the landing camera ------------------------------

void test_resize() {
    using skysim::render::Image;
    // A 2x2 picture scaled to 4x4: the corners keep their colours, the middle is their blend, and
    // the result does not depend on the pool.
    Image src;
    src.width = 2;
    src.height = 2;
    src.rgb = {0, 0, 0, 200, 0, 0, 0, 200, 0, 200, 200, 0};
    const Image up = skysim::render::resize_bilinear(src, 4, 4);
    CHECK(up.width == 4 && up.height == 4 && up.rgb.size() == 48);
    CHECK(up.rgb[0] == 0 && up.rgb[1] == 0 && up.rgb[2] == 0);                 // top-left: src (0,0)
    CHECK(up.rgb[(0 * 4 + 3) * 3] == 200 && up.rgb[(0 * 4 + 3) * 3 + 1] == 0); // top-right: src (1,0)
    CHECK(up.rgb[(3 * 4 + 3) * 3] == 200 && up.rgb[(3 * 4 + 3) * 3 + 1] == 200);
    // Pixel (1,1) samples src at (0.25, 0.25): 9/16 of (0,0), 3/16 each of (1,0) and (0,1), 1/16 of (1,1).
    CHECK(std::abs(up.rgb[(1 * 4 + 1) * 3] - (3 * 200 + 200) / 16) <= 1);
    skysim::core::ThreadPool pool(3);
    Image big;
    big.width = 64;
    big.height = 36;
    big.rgb.resize(64 * 36 * 3);
    for (size_t i = 0; i < big.rgb.size(); ++i) {
        big.rgb[i] = static_cast<uint8_t>((i * 37) % 251);
    }
    const Image a = skysim::render::resize_bilinear(big, 192, 108);
    const Image b = skysim::render::resize_bilinear(big, 192, 108, &pool);
    CHECK(a.rgb == b.rgb && a.width == 192 && a.height == 108);
    const Image same = skysim::render::resize_bilinear(big, 64, 36);
    CHECK(same.rgb == big.rgb); // its own size: unchanged
    CHECK(skysim::render::resize_bilinear(Image{}, 10, 10).empty());
}

void test_camera_modes() {
    using namespace std::chrono_literals;
    using skysim::render::CameraMode;
    using skysim::render::RenderService;
    struct Seen {
        uint32_t id;
        int width;
        int height;
        CameraMode mode;
    };
    std::mutex mutex;
    std::vector<Seen> seen;
    const auto make = [&](double rest_fps) {
        RenderService::Config cfg;
        cfg.camera.width = 128; // the picture: every frame of every vehicle
        cfg.camera.height = 72;
        cfg.camera.fov_deg = 81.0;
        cfg.draw_width = 64; // the normal mode draws half that and scales it up
        cfg.draw_height = 36;
        cfg.scene = skysim::render::load_scene("", 1);
        cfg.fps = 20.0;
        cfg.rest_fps = rest_fps;
        cfg.threads = 2;
        cfg.background_priority = false;
        cfg.demand_window_s = 1.0;
        cfg.landing = {0, 0, 10.0, 3000}; // the picture's own size
        cfg.landing_radius_m = 20.0;
        cfg.landing_height_m = 30.0;
        cfg.settle_s = 0.4;
        cfg.ground_cache_bytes = 16u << 20;
        cfg.video_sink = [&](uint32_t id, std::shared_ptr<const skysim::render::Image> image, double,
                             std::chrono::steady_clock::time_point, const CameraMode &mode) {
            std::lock_guard<std::mutex> lock(mutex);
            seen.push_back({id, image->width, image->height, mode});
        };
        return cfg;
    };
    RenderService service(make(0.0));
    const auto cfg = service.config();

    // Settings: the fleet's until given, bounds and aspect ratio checked (and nothing changed on a
    // refusal).
    auto cam = service.camera(1);
    CHECK(cam.picture_width == 128 && cam.picture_height == 72 && !cam.picture_own);
    CHECK(cam.normal == (CameraMode{64, 36, 20.0, 0}) && !cam.normal_own);
    CHECK(cam.landing_available && cam.landing == (CameraMode{128, 72, 10.0, 3000}) && !cam.landing_active);
    std::string error;
    skysim::render::CameraSettingsPatch bad;
    bad.normal.width = 8;
    CHECK(!service.configure(1, bad, &error) && error.find("width") != std::string::npos);
    bad = {};
    bad.landing.fps = 0.1;
    CHECK(!service.configure(1, bad, &error) && error.find("fps") != std::string::npos);
    bad = {};
    bad.normal.bitrate_kbps = 10;
    CHECK(!service.configure(1, bad, &error) && error.find("bitrate") != std::string::npos);
    bad = {};
    bad.picture_width = 100; // 100x72 against a 64x36 draw size: stretched, so refused
    CHECK(!service.configure(1, bad, &error) && error.find("aspect") != std::string::npos);
    CHECK(!service.camera(1).normal_own && !service.camera(1).picture_own);
    skysim::render::CameraSettingsPatch slow;
    slow.normal.fps = 5.0;
    cam = *service.configure(2, slow, &error);
    CHECK(cam.normal == (CameraMode{64, 36, 5.0, 0}) && cam.normal_own);

    // Two vehicles parked on their docks (home), both watched: their own rates, 20 and 5 fps,
    // each on its own schedule; every picture 128x72, the stream told the mode's rate.
    RenderService::Pose a;
    a.vehicle_id = 1;
    a.position_ned = {0.0, 0.0, -0.1};
    RenderService::Pose b = a;
    b.vehicle_id = 2;
    b.position_ned = {0.0, 5.0, -0.1};
    b.home_east_m = 5.0;
    service.publish_poses({a, b});
    const auto watch = [&](RenderService &svc, std::chrono::milliseconds span) {
        const auto until = std::chrono::steady_clock::now() + span;
        while (std::chrono::steady_clock::now() < until) {
            svc.want_video(1);
            svc.want_video(2);
            std::this_thread::sleep_for(50ms);
        }
    };
    watch(service, 300ms);
    {
        std::lock_guard<std::mutex> lock(mutex);
        seen.clear();
    }
    watch(service, 2000ms);
    {
        std::lock_guard<std::mutex> lock(mutex);
        const auto n1 = std::count_if(seen.begin(), seen.end(), [](const Seen &s) { return s.id == 1; });
        const auto n2 = std::count_if(seen.begin(), seen.end(), [](const Seen &s) { return s.id == 2; });
        std::printf("test_render: 2 s at 20 and 5 fps: %ld and %ld pictures\n", static_cast<long>(n1),
                    static_cast<long>(n2));
        CHECK(n1 >= 30 && n1 <= 45);
        CHECK(n2 >= 8 && n2 <= 12);
        CHECK(std::all_of(seen.begin(), seen.end(), [](const Seen &s) {
            return s.width == 128 && s.height == 72 && s.mode.width == 128 && s.mode.height == 72;
        }));
        CHECK(std::all_of(seen.begin(), seen.end(),
                          [](const Seen &s) { return s.mode.fps == (s.id == 1 ? 20.0 : 5.0); }));
    }
    CHECK(service.camera(1).watched && service.camera(1).fps > 15.0 && service.camera(2).fps < 7.0);

    // Vehicle 1 lifts off its dock and climbs: the landing mode — drawn full size, at its rate and
    // bitrate — while it is in the zone; its normal mode again above it (30 m + the 5 m margin).
    // The picture stays 128x72 throughout.
    double z = -0.1;
    const auto climb_to = [&](double target, std::chrono::milliseconds span) {
        const auto start = std::chrono::steady_clock::now();
        const double from = z;
        while (std::chrono::steady_clock::now() - start < span) {
            const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start) / span;
            z = from + (target - from) * std::min(t, 1.0);
            a.position_ned = {0.0, 0.0, z};
            service.publish_poses({a, b});
            service.want_video(1);
            service.want_video(2);
            std::this_thread::sleep_for(20ms);
        }
    };
    {
        std::lock_guard<std::mutex> lock(mutex);
        seen.clear();
    }
    climb_to(-10.0, 1000ms);
    cam = service.camera(1);
    CHECK(cam.landing_active && cam.now == (CameraMode{128, 72, 10.0, 3000}));
    CHECK(service.mode_now(1) == cam.now);
    CHECK(service.stats().landing == 1);
    CHECK(!service.camera(2).landing_active); // parked: never
    {
        std::lock_guard<std::mutex> lock(mutex);
        const bool landing_seen = std::any_of(seen.begin(), seen.end(), [](const Seen &s) {
            return s.id == 1 && s.mode.fps == 10.0 && s.mode.bitrate_kbps == 3000;
        });
        CHECK(landing_seen);
        CHECK(std::all_of(seen.begin(), seen.end(),
                          [](const Seen &s) { return s.width == 128 && s.height == 72; }));
    }
    climb_to(-33.0, 500ms); // inside the margin: still landing
    CHECK(service.camera(1).landing_active);
    climb_to(-40.0, 300ms);
    watch(service, 200ms);
    cam = service.camera(1);
    CHECK(!cam.landing_active && cam.now == cam.normal);

    // Back down onto the dock: landing on the way, and once it has sat still for settle_s on the
    // ground, its normal mode again.
    climb_to(-5.0, 600ms);
    CHECK(service.camera(1).landing_active);
    climb_to(-0.1, 400ms);
    watch(service, 200ms);
    CHECK(service.camera(1).landing_active); // just touched down: not at rest yet
    watch(service, 700ms);
    CHECK(!service.camera(1).landing_active);

    // A hover over the dock, however still, is not at rest: the landing mode stays.
    climb_to(-12.0, 400ms);
    watch(service, 1000ms);
    CHECK(service.camera(1).landing_active);

    // Turned off for the vehicle: its normal mode at once; reset gives the fleet's back.
    skysim::render::CameraSettingsPatch off;
    off.landing_enabled = false;
    cam = *service.configure(1, off, &error);
    CHECK(!cam.landing_available);
    watch(service, 200ms);
    CHECK(!service.camera(1).landing_active && service.mode_now(1) == service.camera(1).normal);
    skysim::render::CameraSettingsPatch reset;
    reset.reset = true;
    cam = *service.configure(1, reset, &error);
    CHECK(cam.landing_available && !cam.normal_own);

    // Its own landing mode, from the fleet's: a patch that names only a rate; and its own picture
    // size (the fleet's draw size does not fit it, so its normal mode is drawn at the picture size).
    skysim::render::CameraSettingsPatch own;
    own.landing.fps = 4.0;
    cam = *service.configure(1, own, &error);
    CHECK(cam.landing_own && cam.landing == (CameraMode{128, 72, 4.0, 3000}));
    skysim::render::CameraSettingsPatch picture;
    picture.picture_width = 256;
    picture.picture_height = 144;
    cam = *service.configure(1, picture, &error);
    // (its landing mode, drawn at the picture's own size, follows the picture)
    CHECK(cam.picture_own && cam.picture_width == 256 && cam.normal.width == 64 && cam.landing.width == 256);

    // forget(): a despawned vehicle's settings go with it.
    service.forget(2);
    CHECK(!service.camera(2).normal_own);
    (void)cfg;
    service.forget(1);
    service.publish_poses({}); // nothing left to draw for the sink the next service shares
    std::this_thread::sleep_for(200ms);

    // The rest rate: a vehicle parked on its dock is drawn at it (its picture unchanged, so these
    // are re-sends), and the stream is still told its mode's rate.
    {
        RenderService rest(make(2.0));
        RenderService::Pose p = a;
        p.position_ned = {0.0, 0.0, -0.1};
        rest.publish_poses({p});
        watch(rest, 300ms);
        {
            std::lock_guard<std::mutex> lock(mutex);
            seen.clear();
        }
        watch(rest, 2000ms);
        std::lock_guard<std::mutex> lock(mutex);
        const auto n = std::count_if(seen.begin(), seen.end(), [](const Seen &s) { return s.id == 1; });
        std::printf("test_render: 2 s parked at a 2 fps rest rate: %ld pictures\n", static_cast<long>(n));
        CHECK(n >= 3 && n <= 6);
        CHECK(std::all_of(seen.begin(), seen.end(), [](const Seen &s) { return s.mode.fps == 20.0; }));
        CHECK(rest.camera(1).resting && rest.camera(1).now.fps == 2.0);
    }
}

} // namespace

int main() {
    const auto dir = std::filesystem::temp_directory_path() / "skysim_render_tiles";
    std::filesystem::create_directories(dir);
    const auto obj = dir / "building.obj";
    write_test_obj(obj);

    std::string err;
    if (!skysim::terrain::cook_obj_tile(obj, dir / "building.jshape", nullptr, &err)) {
        std::printf("FAIL cook: %s\n", err.c_str());
        return 1;
    }

    skysim::core::World w(cfg());
    w.add_ground_plane();
    CHECK(w.load_tiles(dir) == 1);

    skysim::render::CameraConfig cam;
    cam.width = 128;
    cam.height = 72;
    cam.pitch_deg = 0.0; // level, so the horizon lands across the middle
    const skysim::render::Renderer renderer(cam);

    // Level, facing north (identity attitude), 20 m up, well south of the building.
    const skysim::core::Vec3 pos{-60.0, 0.0, -20.0};
    const skysim::core::Quat level{1.0, 0.0, 0.0, 0.0};

    const skysim::render::Image img = renderer.render(w, 0, pos, level, nullptr);
    CHECK(img.width == cam.width);
    CHECK(img.height == cam.height);
    CHECK(img.rgb.size() == static_cast<size_t>(cam.width) * cam.height * 3);

    // --- The world is the right way up. ---
    // A level camera 20 m up sees sky along the top edge and ground along the
    // bottom. Upside down, or with the pitch sign flipped, this fails.
    CHECK(looks_like_sky(pixel(img, cam.width / 2, 1)));
    CHECK(!looks_like_sky(pixel(img, cam.width / 2, cam.height - 2)));

    // --- The building is in front, and it is not sky. ---
    // Straight ahead at eye level the ray runs into the box's south wall.
    const Px ahead = pixel(img, cam.width / 2, cam.height / 2 - 1);
    CHECK(!looks_like_sky(ahead));

    // --- Turning away from it shows sky at the same pixel. ---
    // Yaw 180 degrees: nothing behind but empty world, so the centre row above
    // the horizon must be sky. This is what catches a camera that ignores
    // attitude entirely and always renders the same frame.
    const skysim::core::Quat about_face{0.0, 0.0, 0.0, 1.0}; // 180 deg about NED down
    const skysim::render::Image behind = renderer.render(w, 0, pos, about_face, nullptr);
    CHECK(looks_like_sky(pixel(behind, cam.width / 2, cam.height / 2 - 1)));

    // --- Ground gets nearer as you look further down. ---
    // Bottom of the frame is closer ground than the middle, so it fogs less and
    // stays darker than the horizon haze.
    {
        skysim::render::CameraConfig down = cam;
        down.pitch_deg = -60.0;
        const skysim::render::Renderer looking_down(down);
        const skysim::render::Image g = looking_down.render(w, 0, pos, level, nullptr);
        for (int x = 0; x < g.width; x += 16) {
            CHECK(!looks_like_sky(pixel(g, x, g.height - 2)));
        }
    }

    // --- A flat roof is not a lawn. ---
    //
    // A roof normal points straight up, exactly like the ground's, so a renderer
    // that decides "ground or building" from the normal turns every rooftop in
    // the city into grass. Both renderers have to get this from the geometry's
    // provenance instead.
    //
    // Camera 50 m above the roof and 29 m south of its centre, pitched 60 deg
    // down: the centre ray clears the south wall and lands on the roof.
    {
        skysim::render::CameraConfig over = cam;
        over.pitch_deg = -60.0;
        const skysim::core::Vec3 above{21.0, 0.0, -80.0};

        const skysim::render::Image roof = skysim::render::Renderer(over).render(w, 0, above, level, nullptr);
        CHECK(!looks_like_sky(pixel(roof, over.width / 2, over.height / 2)));
        CHECK(!looks_like_ground(pixel(roof, over.width / 2, over.height / 2)));

        skysim::render::RasterScene scene;
        scene.build(w.collect_static_triangles());
        const skysim::render::Image r =
            skysim::render::Rasterizer(over).render(scene, skysim::render::Camera(over, above, level), nullptr);
        CHECK(!looks_like_sky(pixel(r, over.width / 2, over.height / 2)));
        CHECK(!looks_like_ground(pixel(r, over.width / 2, over.height / 2)));

        // And the actual ground still reads as ground, or the check above would
        // pass just as well on a renderer that had stopped drawing grass at all.
        CHECK(looks_like_ground(pixel(img, cam.width / 2, cam.height - 2)));
    }

    // --- It encodes, and to something a decoder will accept. ---
    const std::vector<uint8_t> jpeg = skysim::render::encode_jpeg(img, 70);
    CHECK(jpeg.size() > 256);
    CHECK(jpeg.size() < img.rgb.size()); // compression actually happened
    // Guarded: CHECK keeps going after a failure, and indexing an empty buffer
    // would crash the run instead of reporting which assertion failed.
    if (jpeg.size() >= 4) {
        CHECK(jpeg[0] == 0xFF && jpeg[1] == 0xD8);                             // SOI
        CHECK(jpeg[jpeg.size() - 2] == 0xFF && jpeg[jpeg.size() - 1] == 0xD9); // EOI
    }

    // --- An empty image encodes to nothing rather than crashing. ---
    CHECK(skysim::render::encode_jpeg(skysim::render::Image{}, 70).empty());

    // --- The rasteriser draws the same world as the ray caster. ---
    //
    // It is a different algorithm against the same geometry, so it is checked
    // against the same facts rather than against the other renderer's pixels:
    // sky up, ground down, building ahead, sky when you turn away.
    {
        skysim::render::RasterScene scene;
        scene.build(w.collect_static_triangles());
        CHECK(scene.triangle_count() > 0); // 0 means the extraction found nothing

        const skysim::render::Rasterizer raster(cam);

        const skysim::render::Image r = raster.render(scene, skysim::render::Camera(cam, pos, level), nullptr);
        CHECK(r.width == cam.width && r.height == cam.height);
        CHECK(looks_like_sky(pixel(r, cam.width / 2, 1)));
        CHECK(!looks_like_sky(pixel(r, cam.width / 2, cam.height - 2)));
        CHECK(!looks_like_sky(pixel(r, cam.width / 2, cam.height / 2 - 1))); // the building

        const skysim::render::Image back =
            raster.render(scene, skysim::render::Camera(cam, pos, about_face), nullptr);
        CHECK(looks_like_sky(pixel(back, cam.width / 2, cam.height / 2 - 1)));

        // Threaded and serial must agree exactly: each band owns its pixels, so
        // splitting the work cannot change a single one of them.
        skysim::core::ThreadPool pool(4);
        const skysim::render::Image threaded =
            raster.render(scene, skysim::render::Camera(cam, pos, level), &pool);
        CHECK(threaded.rgb == r.rgb);
    }

    // --- FrameStore hands one frame per vehicle across threads. ---
    {
        skysim::render::FrameStore store;
        CHECK(store.get(1).jpeg.empty()); // never rendered
        CHECK(store.get(1).sim_time_s == 0.0);

        store.publish(1, {1, 2, 3}, 4.5);
        CHECK(store.get(1).jpeg.size() == 3);
        CHECK(store.get(1).sim_time_s == 4.5);
        CHECK(store.get(2).jpeg.empty()); // one vehicle's frame is not another's

        // Replaced whole, not appended to: a viewer wants the newest picture.
        store.publish(1, {9}, 5.0);
        CHECK(store.get(1).jpeg.size() == 1);
        CHECK(store.get(1).jpeg[0] == 9);

        store.erase(1);
        CHECK(store.get(1).jpeg.empty());
        store.erase(404); // erasing a vehicle that was never there is not an error

        // A despawned vehicle's last frame must go with it, or the camera route
        // serves a picture of an aircraft that no longer exists.
        store.publish(1, {1}, 1.0);
        store.publish(2, {2}, 1.0);
        store.retain({2});
        CHECK(store.get(1).jpeg.empty());
        CHECK(!store.get(2).jpeg.empty());
        store.retain({}); // whole fleet gone
        CHECK(store.get(2).jpeg.empty());
    }

    // --- RenderService draws on its own thread, off its own world. ---
    //
    // The service is what actually runs in flight, and its two failure modes are
    // both silent: a world that loaded no geometry, and a thread that never
    // produces a frame. Both look like a working server serving nothing.
    {
        const auto tiles = std::filesystem::temp_directory_path() / "skysim_service_tiles";
        std::filesystem::create_directories(tiles);
        write_test_obj(tiles / "building.obj");
        CHECK(skysim::terrain::cook_obj_tile(tiles / "building.obj", tiles / "building.jshape", nullptr, &err));

        skysim::render::RenderService::Config scfg;
        scfg.camera = cam;
        scfg.tiles_dir = tiles.string();
        scfg.fps = 60.0; // fast, so the test is not mostly sleeping
        scfg.threads = 2;

        skysim::render::RenderService service(scfg);
        CHECK(service.tiles_loaded() == 1);
        CHECK(service.triangle_count() > 0);
        CHECK(service.frame(1).jpeg.empty()); // nothing published yet, so nothing to draw

        skysim::render::RenderService::Pose pose;
        pose.vehicle_id = 1;
        pose.position_ned = pos;
        pose.quat_ned_frd = level;
        pose.sim_time_s = 12.5;
        service.publish_poses({pose});

        std::vector<uint8_t> frame;
        for (int i = 0; i < 200 && frame.empty(); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            frame = service.frame(1).jpeg;
        }
        CHECK(!frame.empty());
        CHECK(frame.size() > 2 && frame[0] == 0xFF && frame[1] == 0xD8); // a real JPEG
        CHECK(service.frame(2).jpeg.empty());                            // only the posed vehicle

        // Despawn it: the next pass must drop the frame rather than keep serving
        // a picture of a vehicle that is gone.
        service.publish_poses({});
        bool cleared = false;
        for (int i = 0; i < 200 && !cleared; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            cleared = service.frame(1).jpeg.empty();
        }
        CHECK(cleared);

        std::filesystem::remove_all(tiles);
    } // ~RenderService must stop and join its thread rather than hang here

    // --- RenderService draws only what somebody is watching. ---
    //
    // It used to draw every vehicle in the fleet at the camera rate whether or not anyone was
    // looking: a dev server with one parked vehicle slot and no viewer burned two vCPU on it.
    // Idle must cost nothing, a request must still get a frame promptly, and a vehicle that
    // has not moved must not be drawn again.
    {
        using namespace std::chrono_literals;
        using Clock = std::chrono::steady_clock;
        skysim::render::RenderService::Config dcfg;
        dcfg.camera = cam;
        dcfg.scene = skysim::render::load_scene("", 1); // ground plane only: cheap frames
        dcfg.fps = 50.0;
        dcfg.threads = 2;
        dcfg.demand_window_s = 0.3;
        skysim::render::RenderService service(dcfg);
        CHECK(service.threads() == 2);

        // A vehicle in the fleet and nobody asking: nothing drawn, and no CPU spent.
        skysim::render::RenderService::Pose pose;
        pose.vehicle_id = 7;
        pose.position_ned = pos;
        pose.quat_ned_frd = level;
        pose.sim_time_s = 1.0;
        service.publish_poses({pose});
        double cpu0 = process_cpu_s();
        std::this_thread::sleep_for(500ms);
        const double idle_cpu = process_cpu_s() - cpu0;
        std::printf("test_render: idle camera with one unwatched vehicle used %.1f ms CPU in 500 ms\n",
                    idle_cpu * 1e3);
        CHECK(service.stats().frames_drawn == 0);
        CHECK(service.stats().watched == 0);
        CHECK(idle_cpu < 0.05); // it used to be the whole frame rate's worth: >1 core

        // The first request starts the drawing and gets its frame within the wait.
        const auto first = service.frame(7, 0, 2000ms);
        CHECK(!first.jpeg.empty() && first.jpeg[0] == 0xFF && first.jpeg[1] == 0xD8);
        CHECK(first.seq > 0);

        // A stream waits for the next frame rather than reading the same one again.
        pose.position_ned[0] += 1.0;
        pose.sim_time_s = 2.0;
        service.publish_poses({pose});
        auto next = service.frame(7, first.seq, 2000ms);
        CHECK(next.seq > first.seq);
        for (int i = 0; i < 10 && next.sim_time_s != 2.0; ++i) {
            next = service.frame(7, next.seq, 2000ms); // the pass that read the old pose
        }
        CHECK(next.sim_time_s == 2.0);

        // Not moving: the frame is re-sent each pass, not drawn again.
        const uint64_t drawn_before = service.stats().frames_drawn;
        uint64_t seq = next.seq;
        for (int i = 0; i < 6; ++i) {
            const auto f = service.frame(7, seq, 2000ms);
            CHECK(f.seq > seq && f.jpeg == next.jpeg);
            seq = f.seq;
        }
        const auto busy = service.stats();
        CHECK(busy.frames_drawn <= drawn_before + 1); // at most the one being recorded as this ran
        CHECK(busy.frames_reused >= 5);
        CHECK(busy.target_fps == 50.0 && busy.threads == 2);

        // TASK-2548: the gimbal moving while the vehicle holds still is a new picture, drawn.
        pose.gimbal = true;
        pose.gimbal_pitch_rad = -60.0 * M_PI / 180.0;
        service.publish_poses({pose});
        auto tilted = service.frame(7, seq, 2000ms);
        for (int i = 0; i < 10 && tilted.jpeg == next.jpeg; ++i) {
            tilted = service.frame(7, tilted.seq, 2000ms); // the pass that read the old pose
        }
        CHECK(!tilted.jpeg.empty() && tilted.jpeg != next.jpeg);
        CHECK(service.stats().frames_drawn > busy.frames_drawn);

        // A vehicle that is not in the fleet never gets a frame, so asking does not wait.
        const auto asked = Clock::now();
        CHECK(service.frame(99, 0, 2000ms).jpeg.empty());
        CHECK(Clock::now() - asked < 500ms);

        // A stream watching for a couple of seconds sees the camera rate, and /metrics says so.
        const auto watch_until = Clock::now() + 2300ms;
        while (Clock::now() < watch_until) {
            seq = service.frame(7, seq, 2000ms).seq;
        }
        const auto watched = service.stats();
        std::printf("test_render: watched at %.1f fps (target 50), render %.2f ms, encode %.2f ms\n",
                    watched.fps, watched.render_ms, watched.encode_ms);
        CHECK(watched.watched == 1);
        CHECK(watched.fps > 10.0 && watched.fps < 60.0);
        CHECK(watched.render_ms > 0.0 && watched.render_ms_max >= watched.render_ms);

        // Stop asking: within the demand window the service stops drawing, reports idle,
        // drops the frame (it is no longer the camera's view) and goes back to costing nothing.
        std::this_thread::sleep_for(800ms);
        const auto lapsed = service.stats();
        CHECK(lapsed.watched == 0 && lapsed.fps == 0.0);
        // Requests for vehicles that are not in the fleet go nowhere: not on the draw list, not
        // waking the render thread. 20000 made-up ids used to be 20000 wake-ups, each scanning
        // a draw list as long as the ids so far.
        cpu0 = process_cpu_s();
        const double caller0 = thread_cpu_s();
        for (uint32_t id = 1000; id < 21000; ++id) {
            (void)service.frame(id);
        }
        std::this_thread::sleep_for(400ms);
        const double after_cpu = (process_cpu_s() - cpu0) - (thread_cpu_s() - caller0);
        const auto still_lapsed = service.stats();
        CHECK(still_lapsed.frames_drawn == lapsed.frames_drawn);
        CHECK(still_lapsed.frames_reused == lapsed.frames_reused);
        CHECK(after_cpu < 0.05);
        // Asking again restarts the drawing, but never serves the stale frame: what comes back is
        // nothing yet, or (if the renderer was quick) a frame of the pose published now.
        pose.sim_time_s = 9.0;
        service.publish_poses({pose});
        const auto again = service.frame(7);
        CHECK(again.jpeg.empty() || again.sim_time_s == 9.0);
    }

    // The "camera:" log: a line a period while drawing, none while idle, and a viewer that
    // comes and goes neither floods it nor loses what it watched.
    {
        using namespace std::chrono_literals;
        using Clock = std::chrono::steady_clock;
        std::mutex lines_mutex;
        std::vector<std::pair<Clock::time_point, std::string>> lines;
        skysim::render::RenderService::Config lcfg;
        lcfg.camera = cam;
        lcfg.scene = skysim::render::load_scene("", 1);
        lcfg.fps = 50.0;
        lcfg.threads = 1;
        lcfg.demand_window_s = 0.2;
        lcfg.log_every_s = 0.8;
        lcfg.log_sink = [&](const char *line) {
            std::lock_guard<std::mutex> lock(lines_mutex);
            lines.emplace_back(Clock::now(), line);
        };
        skysim::render::RenderService service(lcfg);
        skysim::render::RenderService::Pose pose;
        pose.vehicle_id = 4;
        pose.position_ned = pos;
        service.publish_poses({pose});
        std::this_thread::sleep_for(300ms);
        {
            std::lock_guard<std::mutex> lock(lines_mutex);
            CHECK(lines.empty()); // nothing drawn, nothing to say
        }

        // Four short looks, each well under the log period, then nobody for two periods.
        for (int look = 0; look < 4; ++look) {
            uint64_t seq = 0;
            const auto until = Clock::now() + 150ms;
            while (Clock::now() < until) {
                seq = service.frame(4, seq, 1000ms).seq;
            }
            std::this_thread::sleep_for(350ms); // past the demand window: idle again
        }
        std::this_thread::sleep_for(1700ms);
        const auto totals = service.stats();

        std::lock_guard<std::mutex> lock(lines_mutex);
        std::printf("test_render: camera log lines:\n");
        for (const auto &[at, line] : lines) {
            std::printf("  %s\n", line.c_str());
        }
        // Never more than one log event a period: a summary and the idle line right behind it
        // are one event; anything else is a period apart (less a little scheduling slack).
        size_t events = 0;
        unsigned long long summarised = 0;
        for (size_t i = 0; i < lines.size(); ++i) {
            const bool idle = lines[i].second.find("idle, nobody watching") != std::string::npos;
            const bool paired = idle && i > 0 && lines[i].first - lines[i - 1].first < 20ms &&
                                lines[i - 1].second.find("fps each") != std::string::npos;
            if (!paired) {
                if (events > 0) {
                    CHECK(lines[i].first - lines[i - 1].first > 700ms);
                }
                ++events;
            }
            unsigned long long drawn = 0, resent = 0;
            const size_t at = lines[i].second.find(" drawn + ");
            if (at != std::string::npos) {
                const size_t num = lines[i].second.rfind(", ", at);
                std::sscanf(lines[i].second.c_str() + num + 2, "%llu drawn + %llu re-sent", &drawn, &resent);
                summarised += drawn + resent;
            }
        }
        // Nothing the viewer watched went unreported: the summaries add up to every frame, and
        // the last thing said is that the camera went idle.
        CHECK(summarised == totals.frames_drawn + totals.frames_reused);
        CHECK(totals.frames_drawn + totals.frames_reused > 4);
        CHECK(events >= 2 && events <= 4);
        CHECK(!lines.empty() && lines.back().second.find("idle, nobody watching") != std::string::npos);
    }

    // One render thread is the service's own: no pool at all.
    {
        skysim::render::RenderService::Config one;
        one.camera = cam;
        one.scene = skysim::render::load_scene("", 1);
        one.threads = 1;
        skysim::render::RenderService service(one);
        CHECK(service.threads() == 1);
        skysim::render::RenderService::Pose pose;
        pose.vehicle_id = 1;
        pose.position_ned = pos;
        service.publish_poses({pose});
        CHECK(!service.frame(1, 0, std::chrono::milliseconds(2000)).jpeg.empty());
    }

    test_ground_cache(dir);
    test_resize();
    test_camera_modes();

    std::filesystem::remove_all(dir);
    if (g_failures == 0) {
        std::printf("test_render: OK\n");
    }
    return g_failures == 0 ? 0 : 1;
}
