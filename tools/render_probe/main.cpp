// Render one frame of a cooked world from a pose given on the command line.
//
// The camera in the server draws from wherever the aircraft happens to be, which
// makes "does this look right from 60 m up, facing east" an awkward question to
// ask: you have to fly there first. This asks it directly, and writes a JPEG.
//
//   render_probe --tiles build/demo_map/tiles --north 0 --east 0 --alt 60 \
//                --yaw 90 --out /tmp/view.jpg
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <string>

#include "core/thread_pool.h"
#include "core/world.h"
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
};

[[noreturn]] void usage(const char *why) {
    std::fprintf(stderr, "render_probe: %s\n", why);
    std::fprintf(stderr, "usage: render_probe --tiles DIR [--out F] [--north M] [--east M] [--alt M]\n"
                         "                    [--yaw DEG] [--pitch DEG] [--size WxH] [--fov DEG]\n");
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
    skysim::core::ThreadPool pool(o.threads);

    skysim::render::RasterScene scene;
    const skysim::render::Rasterizer raster(cfg);
    if (o.raster) {
        const auto tris = world.collect_static_triangles();
        scene.build(tris);
        std::printf("render_probe: %zu triangle(s) in the scene\n", scene.triangle_count());
    }
    const skysim::render::Camera camera(cfg, pos, attitude);

    const auto t0 = std::chrono::steady_clock::now();
    skysim::render::Image image;
    for (int i = 0; i < std::max(1, o.repeat); ++i) {
        image = o.raster ? raster.render(scene, camera, &pool) : renderer.render(world, 0, pos, attitude, &pool);
    }
    const auto t1 = std::chrono::steady_clock::now();
    if (o.repeat > 1) {
        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count() / o.repeat;
        std::printf("render_probe: %.2f ms/frame with %d thread(s) (%.1f fps) at %dx%d\n", ms, o.threads,
                    1000.0 / ms, image.width, image.height);
    }
    const auto jpeg = skysim::render::encode_jpeg(image, o.quality);
    if (jpeg.empty()) {
        std::fprintf(stderr, "render_probe: encode failed\n");
        return 1;
    }

    std::ofstream f(o.out, std::ios::binary);
    f.write(reinterpret_cast<const char *>(jpeg.data()), static_cast<std::streamsize>(jpeg.size()));
    std::printf("render_probe: wrote %s (%zu bytes, %dx%d)\n", o.out.c_str(), jpeg.size(), image.width, image.height);
    return 0;
}
