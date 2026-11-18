// Render one frame of a cooked world from a pose given on the command line.
//
// The camera in the server draws from wherever the aircraft happens to be, which
// makes "does this look right from 60 m up, facing east" an awkward question to
// ask: you have to fly there first. This asks it directly, and writes a JPEG.
//
//   render_probe --tiles build/demo_map/tiles --north 0 --east 0 --alt 60 \
//                --yaw 90 --out /tmp/view.jpg
//
// What the pilot camera sees over a dock (TASK-2661): --dock N,E lays Observer 1's dock marker
// there at its real size, as the server draws it; --pitch -90 looks straight down; --cache MB
// reads the ground from the pilot camera's ground cache (render/ground_cache.h) instead of
// computing it per pixel; an --out ending in .ppm is written losslessly. With --repeat the first
// frame (cold cache) and the rest (warm) are timed apart.
//
//   render_probe --tiles build/demo_map/tiles --raster --size 1920x1080 --fov 81 --pitch -90 \
//                --alt 30 --dock 0,0 --cache 256 --threads 4 --repeat 5 --out /tmp/dock_30m.ppm
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "core/thread_pool.h"
#include "core/world.h"
#include "render/dock_marker.h"
#include "render/ground_cache.h"
#include "render/jpeg.h"
#include "render/raster.h"
#include "render/renderer.h"

namespace {

struct Options {
    std::string tiles;
    std::string out = "frame.jpg";
    double north = 0.0;
    double east = 0.0;
    double alt = 40.0; // metres above the ground plane
    double yaw_deg = 0.0;
    double pitch_deg = -15.0;
    int width = 320;
    int height = 180;
    double fov_deg = 78.0;
    int quality = 80;
    int repeat = 1; // >1 reports render cost per frame
    int threads = 0; // render worker threads; 0 = serial
    bool raster = false; // use the rasteriser instead of the ray caster
    bool dock = false;   // lay a dock marker at (dock_north, dock_east)
    double dock_north = 0.0;
    double dock_east = 0.0;
    double cache_mb = 0.0; // > 0: the rasteriser reads the ground from a cache this big
};

[[noreturn]] void usage(const char *why) {
    std::fprintf(stderr, "render_probe: %s\n", why);
    std::fprintf(stderr,
                 "usage: render_probe --tiles DIR [--out F] [--north M] [--east M] [--alt M]\n"
                 "                    [--yaw DEG] [--pitch DEG] [--size WxH] [--fov DEG]\n"
                 "                    [--raster] [--threads N] [--repeat N] [--quality Q]\n"
                 "                    [--dock N,E] [--cache MB]   (--dock and --cache need --raster)\n");
    std::exit(2);
}

Options parse(int argc, char **argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        auto value = [&](const char *flag) -> const char * {
            if (i + 1 >= argc) {
                usage(flag);
            }
            return argv[++i];
        };
        if (std::strcmp(argv[i], "--tiles") == 0) {
            o.tiles = value("--tiles");
        } else if (std::strcmp(argv[i], "--out") == 0) {
            o.out = value("--out");
        } else if (std::strcmp(argv[i], "--north") == 0) {
            o.north = std::atof(value("--north"));
        } else if (std::strcmp(argv[i], "--east") == 0) {
            o.east = std::atof(value("--east"));
        } else if (std::strcmp(argv[i], "--alt") == 0) {
            o.alt = std::atof(value("--alt"));
        } else if (std::strcmp(argv[i], "--yaw") == 0) {
            o.yaw_deg = std::atof(value("--yaw"));
        } else if (std::strcmp(argv[i], "--pitch") == 0) {
            o.pitch_deg = std::atof(value("--pitch"));
        } else if (std::strcmp(argv[i], "--raster") == 0) {
            o.raster = true;
        } else if (std::strcmp(argv[i], "--threads") == 0) {
            o.threads = std::atoi(value("--threads"));
        } else if (std::strcmp(argv[i], "--repeat") == 0) {
            o.repeat = std::atoi(value("--repeat"));
        } else if (std::strcmp(argv[i], "--quality") == 0) {
            o.quality = std::atoi(value("--quality"));
        } else if (std::strcmp(argv[i], "--dock") == 0) {
            const char *v = value("--dock");
            const char *comma = std::strchr(v, ',');
            if (comma == nullptr) {
                usage("--dock wants N,E (metres)");
            }
            o.dock = true;
            o.dock_north = std::atof(v);
            o.dock_east = std::atof(comma + 1);
        } else if (std::strcmp(argv[i], "--cache") == 0) {
            o.cache_mb = std::atof(value("--cache"));
        } else if (std::strcmp(argv[i], "--fov") == 0) {
            o.fov_deg = std::atof(value("--fov"));
        } else if (std::strcmp(argv[i], "--size") == 0) {
            const char *v = value("--size");
            const char *x = std::strchr(v, 'x');
            o.width = std::atoi(v);
            o.height = (x != nullptr) ? std::atoi(x + 1) : 0;
        } else {
            usage(argv[i]);
        }
    }
    if (o.tiles.empty()) {
        usage("--tiles is required");
    }
    if (o.width <= 0 || o.height <= 0) {
        usage("--size must be WxH with both > 0");
    }
    if ((o.dock || o.cache_mb > 0.0) && !o.raster) {
        usage("--dock and --cache draw with the rasteriser: add --raster");
    }
    if (o.quality < 1 || o.quality > 100) {
        usage("--quality must be 1-100");
    }
    return o;
}

} // namespace

int main(int argc, char **argv) {
    const Options o = parse(argc, argv);

    skysim::core::WorldConfig wcfg;
    wcfg.dt_s = 1.0 / 800.0;
    wcfg.worker_threads = 0;
    skysim::core::World world(wcfg);
    world.add_ground_plane();
    const size_t tiles = world.load_tiles(o.tiles);
    std::printf("render_probe: %zu tile(s) from %s\n", tiles, o.tiles.c_str());

    // Yaw only: level flight, which is what a probe of a static world wants.
    const double half = (o.yaw_deg * M_PI / 180.0) * 0.5;
    const skysim::core::Quat attitude{std::cos(half), 0.0, 0.0, std::sin(half)};

    skysim::render::CameraConfig cfg;
    cfg.width = o.width;
    cfg.height = o.height;
    cfg.fov_deg = o.fov_deg;
    cfg.pitch_deg = o.pitch_deg;
    const skysim::render::Renderer renderer(cfg);

    const skysim::core::Vec3 pos{o.north, o.east, -o.alt};
    // --threads 0 is serial: no pool at all, rather than a pool of one worker plus the caller.
    skysim::core::ThreadPool pool(std::max(o.threads, 1));
    skysim::core::ThreadPool *workers = o.threads > 0 ? &pool : nullptr;

    auto scene = std::make_shared<skysim::render::RasterScene>();
    const skysim::render::Rasterizer raster(cfg);
    if (o.raster) {
        auto tris = world.collect_static_triangles();
        scene->build(std::move(tris));
        std::printf("render_probe: %zu triangle(s) in the scene\n", scene->triangle_count());
    }
    const skysim::render::Camera camera(cfg, pos, attitude);

    // The dock as the server lays one: the default marker (DICT_7X7_250 228 with 11 inside, 1.00 m
    // and 0.09 m), flat on the ground, image top to the north.
    skysim::render::GroundMarkers markers;
    if (o.dock) {
        skysim::render::GroundMarker m;
        m.vehicle_id = 1;
        m.pose.north_m = o.dock_north;
        m.pose.east_m = o.dock_east;
        markers.push_back(m);
    }
    std::unique_ptr<skysim::render::GroundCache> cache;
    if (o.cache_mb > 0.0) {
        cache =
            std::make_unique<skysim::render::GroundCache>(scene, static_cast<size_t>(o.cache_mb * 1048576.0));
    }

    using Clock = std::chrono::steady_clock;
    const auto ms_since = [](Clock::time_point t) {
        return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
    };
    skysim::render::Image image;
    double first_ms = 0.0;
    double rest_ms = 0.0;
    double fastest_ms = 1e300;
    const int frames = std::max(1, o.repeat);
    for (int i = 0; i < frames; ++i) {
        const auto t = Clock::now();
        image = o.raster ? raster.render(*scene, camera, workers, nullptr, true, o.dock ? &markers : nullptr,
                                         cache.get())
                         : renderer.render(world, 0, pos, attitude, workers);
        if (cache) {
            cache->evict();
        }
        const double ms = ms_since(t);
        (i == 0 ? first_ms : rest_ms) += ms;
        fastest_ms = std::min(fastest_ms, ms);
    }
    if (o.repeat > 1) {
        const double ms = (first_ms + rest_ms) / frames;
        std::printf("render_probe: %.2f ms/frame with %d thread(s) (%.1f fps) at %dx%d\n", ms, o.threads,
                    1000.0 / ms, image.width, image.height);
        std::printf("render_probe: first frame %.2f ms, then %.2f ms/frame, fastest %.2f ms%s\n", first_ms,
                    rest_ms / (frames - 1), fastest_ms, cache ? " (ground cache: cold, then warm)" : "");
    }
    if (cache) {
        const auto s = cache->stats();
        std::printf("render_probe: ground cache %zu tile(s), %.1f MB, %llu built\n", s.tiles,
                    static_cast<double>(s.bytes) / 1048576.0, static_cast<unsigned long long>(s.tiles_built));
    }

    std::vector<uint8_t> bytes;
    const bool ppm = o.out.size() >= 4 && o.out.compare(o.out.size() - 4, 4, ".ppm") == 0;
    if (ppm) {
        const std::string head =
            "P6\n" + std::to_string(image.width) + " " + std::to_string(image.height) + "\n255\n";
        bytes.assign(head.begin(), head.end());
        bytes.insert(bytes.end(), image.rgb.begin(), image.rgb.end());
    } else {
        bytes = skysim::render::encode_jpeg(image, o.quality);
        if (bytes.empty()) {
            std::fprintf(stderr, "render_probe: encode failed\n");
            return 1;
        }
    }

    std::ofstream f(o.out, std::ios::binary);
    if (!f) {
        std::fprintf(stderr, "render_probe: cannot open %s\n", o.out.c_str());
        return 1;
    }
    f.write(reinterpret_cast<const char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    f.close();
    if (!f) {
        std::fprintf(stderr, "render_probe: failed writing %s\n", o.out.c_str());
        return 1;
    }
    std::printf("render_probe: wrote %s (%zu bytes, %dx%d)\n", o.out.c_str(), bytes.size(), image.width,
                image.height);
    return 0;
}
