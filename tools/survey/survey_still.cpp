// Render survey stills of a cooked world from poses given on stdin, without a simulator.
//
// The server's GET /instances/<n>/still.jpg draws a frame from wherever a vehicle is; this
// draws the same frame — same scene, same texture seed, same render_still() — from wherever
// it is told, so a survey's photo set can be produced without flying anything. It is the
// renderer behind tools/survey/survey_fixture.py, which plans the grid and writes the
// geotags; on its own it is also the quickest way to benchmark a still.
//
//   survey_still --tiles build/demo/tiles --out /tmp/frames < poses.csv
//
// Pose lines: name,north_m,east_m,down_m,yaw_deg,pitch_deg,roll_deg (camera axes, NED; '#'
// starts a comment). For each frame one CSV line goes to stdout:
//   name,lat,lon,alt_amsl,alt_rel,yaw,pitch,roll,render_ms
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "core/geo.h"
#include "core/thread_pool.h"
#include "render/jpeg.h"
#include "render/scene.h"
#include "render/still.h"

namespace {

struct Options {
    std::string tiles;
    std::string out = ".";
    uint32_t seed = 1;
    int threads = 0; // 0 = hardware concurrency
    int repeat = 1;
    skysim::render::StillRequest request;
    skysim::core::GeoOrigin home{42.1403890, 24.7645490, 0.0}; // skysim's default --spawn-home
};

[[noreturn]] void usage(const char *why) {
    std::fprintf(stderr, "survey_still: %s\n", why);
    std::fprintf(stderr, "usage: survey_still [--tiles DIR] --out DIR [--seed N] [--threads N] [--size WxH]\n"
                         "                    [--hfov DEG] [--range M] [--quality Q] [--home LAT,LON,ALT]\n"
                         "                    [--repeat N] < poses.csv\n");
    std::exit(2);
}

double number(const char *flag, const char *text) {
    char *end = nullptr;
    const double v = std::strtod(text, &end);
    if (end == text || *end != '\0' || !std::isfinite(v)) {
        usage(flag);
    }
    return v;
}

// A whole number in [lo, hi]; anything else is a usage error, never an out-of-range cast.
long long whole(const char *flag, const char *text, long long lo, long long hi) {
    const double v = number(flag, text);
    if (v != std::floor(v) || v < static_cast<double>(lo) || v > static_cast<double>(hi)) {
        usage(flag);
    }
    return static_cast<long long>(v);
}

Options parse(int argc, char **argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const auto value = [&](const char *flag) -> const char * {
            if (i + 1 >= argc) {
                usage(flag);
            }
            return argv[++i];
        };
        const char *a = argv[i];
        if (std::strcmp(a, "--tiles") == 0) {
            o.tiles = value(a);
        } else if (std::strcmp(a, "--out") == 0) {
            o.out = value(a);
        } else if (std::strcmp(a, "--seed") == 0) {
            o.seed = static_cast<uint32_t>(whole(a, value(a), 0, 4294967295LL));
        } else if (std::strcmp(a, "--threads") == 0) {
            o.threads = static_cast<int>(whole(a, value(a), 0, 256));
        } else if (std::strcmp(a, "--repeat") == 0) {
            o.repeat = static_cast<int>(whole(a, value(a), 1, 100000));
        } else if (std::strcmp(a, "--size") == 0) {
            const char *v = value(a);
            const char *x = std::strchr(v, 'x');
            if (x == nullptr) {
                usage(a);
            }
            const std::string w(v, x);
            o.request.width = static_cast<int>(whole(a, w.c_str(), 1, 100000));
            o.request.height = static_cast<int>(whole(a, x + 1, 1, 100000));
        } else if (std::strcmp(a, "--hfov") == 0) {
            o.request.hfov_deg = number(a, value(a));
        } else if (std::strcmp(a, "--range") == 0) {
            o.request.range_m = number(a, value(a));
        } else if (std::strcmp(a, "--quality") == 0) {
            o.request.quality = static_cast<int>(whole(a, value(a), 1, 100));
        } else if (std::strcmp(a, "--home") == 0) {
            if (std::sscanf(value(a), "%lf,%lf,%lf", &o.home.lat_deg, &o.home.lon_deg, &o.home.alt_amsl_m) !=
                3) {
                usage(a);
            }
        } else {
            usage(a);
        }
    }
    const std::string bad = o.request.invalid();
    if (!bad.empty()) {
        usage(bad.c_str());
    }
    if (o.repeat < 1) {
        usage("--repeat must be at least 1");
    }
    return o;
}

} // namespace

int main(int argc, char **argv) {
    const Options o = parse(argc, argv);
    std::filesystem::create_directories(o.out);

    size_t tiles = 0;
    const auto scene = skysim::render::load_scene(o.tiles, o.seed, &tiles);
    std::fprintf(stderr, "survey_still: %zu tile(s), %zu triangle(s), %zu building(s), seed %u\n", tiles,
                 scene->triangle_count(), scene->building_count(), o.seed);

    const int threads =
        o.threads > 0 ? o.threads : std::max(1, static_cast<int>(std::thread::hardware_concurrency()));
    // The caller helps, so threads - 1 workers make `threads` in all.
    skysim::core::ThreadPool pool(std::max(1, threads - 1));
    skysim::core::ThreadPool *workers = threads > 1 ? &pool : nullptr;

    std::printf("name,lat,lon,alt_amsl,alt_rel,yaw,pitch,roll,render_ms\n");
    std::string line;
    int frames = 0;
    while (std::getline(std::cin, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::stringstream fields(line);
        std::string name;
        std::getline(fields, name, ',');
        double v[6];
        bool ok = !name.empty() && name.find('/') == std::string::npos;
        for (double &x : v) {
            std::string f;
            ok = ok && static_cast<bool>(std::getline(fields, f, ','));
            char *end = nullptr;
            x = ok ? std::strtod(f.c_str(), &end) : 0.0;
            ok = ok && end != f.c_str() && std::isfinite(x);
        }
        if (!ok) {
            std::fprintf(stderr, "survey_still: bad pose line: %s\n", line.c_str());
            return 2;
        }

        const skysim::core::Vec3 position{v[0], v[1], v[2]};
        const skysim::core::Quat camera = skysim::render::camera_quat_from_angles(v[3], v[4], v[5]);
        const skysim::render::CameraAngles angles = skysim::render::angles_from_camera_quat(camera);

        skysim::render::Image image;
        const auto t0 = std::chrono::steady_clock::now();
        for (int r = 0; r < o.repeat; ++r) {
            image = skysim::render::render_still(*scene, position, camera, o.request, workers);
        }
        const auto t1 = std::chrono::steady_clock::now();
        const auto jpeg = skysim::render::encode_jpeg(image, o.request.quality);
        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count() / o.repeat;

        const std::string path = (std::filesystem::path(o.out) / name).string();
        std::ofstream f(path, std::ios::binary);
        f.write(reinterpret_cast<const char *>(jpeg.data()), static_cast<std::streamsize>(jpeg.size()));
        f.close();
        if (jpeg.empty() || !f) {
            std::fprintf(stderr, "survey_still: failed writing %s\n", path.c_str());
            return 1;
        }

        const skysim::core::GeoPoint geo = skysim::core::geo_from_ned(o.home, v[0], v[1], v[2]);
        std::printf("%s,%.9f,%.9f,%.3f,%.3f,%.4f,%.4f,%.4f,%.1f\n", name.c_str(), geo.lat_deg, geo.lon_deg,
                    geo.alt_amsl_m, geo.alt_rel_m, angles.yaw_deg, angles.pitch_deg, angles.roll_deg, ms);
        std::fflush(stdout);
        ++frames;
    }
    std::fprintf(stderr, "survey_still: %d frame(s) in %s\n", frames, o.out.c_str());
    return 0;
}
