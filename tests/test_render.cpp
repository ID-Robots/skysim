// Camera render acceptance: put one real cooked building in a world, point a
// camera at it, and check the picture says what the world says — sky above the
// horizon, ground below it, and the building where the building is.
//
// Asserting on pixels rather than on "it produced bytes" is the point: a renderer
// that silently draws a blank frame, or draws the world upside down, still
// produces a valid JPEG.
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

#include "core/world.h"
#include "render/frame_store.h"
#include "render/jpeg.h"
#include "render/raster.h"
#include "render/render_service.h"
#include "render/renderer.h"
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

// Sky is the only thing in the palette that is markedly more blue than red.
bool looks_like_sky(const Px &p) { return p.b > p.r + 20; }

// Ground is the only thing in the palette with a green cast; building grey has
// its three channels within a few counts of each other.
bool looks_like_ground(const Px &p) { return p.g > p.b + 12; }

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

    std::filesystem::remove_all(dir);
    if (g_failures == 0) {
        std::printf("test_render: OK\n");
    }
    return g_failures == 0 ? 0 : 1;
}
