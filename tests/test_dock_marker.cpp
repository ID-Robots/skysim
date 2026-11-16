// Observer 1's e-ArUco dock marker (TASK-2562): the image compiled in is the committed one, its
// texture keeps both markers' bits where the generator put them at the configured sizes, frames
// draw it where the camera model says it is, a frame without one in shot is untouched, the
// pilot camera redraws a parked vehicle's picture when its dock moves, and a vehicle's camera
// draws its own dock alone unless every dock is to be visible (TASK-2609).
//
// That the drawn marker decodes with cv2.aruco, and solvePnP recovers its pose, is
// tests/dock_marker_smoke.py's job (it needs OpenCV); this holds the pixels to the geometry.
//
// With SKYSIM_DOCK_FRAMES=<dir> set, the nadir frames are written there as PPM for a look.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "core/geo.h"
#include "core/world.h"
#include "render/dock.h"
#include "render/dock_marker.h"
#include "render/render_service.h"
#include "render/scene.h"
#include "render/still.h"

namespace {

int g_failures = 0;

#define CHECK(cond)                                                                                          \
    do {                                                                                                     \
        if (!(cond)) {                                                                                       \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                      \
            ++g_failures;                                                                                    \
        }                                                                                                    \
    } while (0)

using namespace skysim;
using render::DockMarkerSpec;
using render::DockPose;

// The 9x9 cells of DICT_7X7_250 IDs 228 and 11 as cv2.aruco draws them ('#' black), border
// included: printed by cv2.aruco.drawMarker(dict, id, 9) (OpenCV 4.6).
const char *const kOuter228[9] = {"#########", "#..###..#", "#####.###", "#.####.##", "#..###.##",
                                  "#####...#", "###.#####", "##...####", "#########"};
const char *const kInner11[9] = {"#########", "#######.#", "####.##.#", "#.##...##", "####.#..#",
                                 "#.####.##", "#.#.#####", "##..#.###", "#########"};

std::shared_ptr<const render::MarkerTexture> asset(const DockMarkerSpec &spec = {}) {
    std::string error;
    auto tex = render::MarkerTexture::from_png(render::dock_marker_png(), render::dock_marker_png_size(),
                                               spec, error);
    if (!tex) {
        std::printf("marker texture: %s\n", error.c_str());
    }
    return tex;
}

void test_embedded_image() {
    // The bytes compiled in are the committed file's.
    std::ifstream f(std::string(SKYSIM_SOURCE_DIR) + "/assets/markers/e_aruco_228_11_100cm.png",
                    std::ios::binary);
    const std::vector<char> file((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    CHECK(file.size() == render::dock_marker_png_size());
    CHECK(!file.empty() &&
          std::equal(file.begin(), file.end(), reinterpret_cast<const char *>(render::dock_marker_png())));
}

void test_layout() {
    const auto tex = asset();
    CHECK(tex != nullptr);
    if (!tex) {
        return;
    }
    const render::DockLayout &l = tex->layout();
    // 3937 px of black square is 1.00 m; a 78 px white border; cells of 437 px; the inner marker's
    // centre 2 px up and left of the outer's (the generator's whole-pixel layout).
    CHECK(std::fabs(tex->texels_per_m() - 3937.0) < 1e-9);
    CHECK(l.outer_size_m == 1.0 && l.inner_size_m == 0.09);
    CHECK(std::fabs(l.cell_size_m - 437.0 / 3937.0) < 1e-12);
    CHECK(std::fabs(l.plate_u_min + 2046.5 / 3937.0) < 1e-12 &&
          std::fabs(l.plate_u_max - 2046.5 / 3937.0) < 1e-12);
    CHECK(std::fabs(l.plate_v_min + 2046.5 / 3937.0) < 1e-12 &&
          std::fabs(l.plate_v_max - 2046.5 / 3937.0) < 1e-12);
    CHECK(std::fabs(l.inner_u_m + 2.0 / 3937.0) < 1e-12 && std::fabs(l.inner_v_m + 2.0 / 3937.0) < 1e-12);
    CHECK(tex->size() == 4096 && tex->levels() == 13);
    CHECK(tex->texel(-1, 0) == 255 && tex->texel(5000, 0) == 255); // off the texture: white
    CHECK(tex->texel(4094, 4094) == 255);                          // the padding
    CHECK(tex->texel(78, 78) == 0 && tex->texel(77, 77) == 255);   // the black square's corner

    // Every cell of both markers, at its centre, sampled finer than a cell: the outer at the
    // image's scale, the inner stretched to 0.09 m about its own centre.
    int wrong = 0;
    for (int r = 0; r < 9; ++r) {
        for (int c = 0; c < 9; ++c) {
            if (r != 4 || c != 4) { // the centre cell is the inner marker's white square
                const double g = tex->sample((c + 0.5) / 9.0 - 0.5, (r + 0.5) / 9.0 - 0.5, 0.002);
                wrong += (kOuter228[r][c] == '#') != (g < 0.5) || (g > 0.05 && g < 0.95) ? 1 : 0;
            }
            const double s = l.inner_size_m;
            const double g = tex->sample(l.inner_u_m + (c + 0.5) * s / 9.0 - s / 2,
                                         l.inner_v_m + (r + 0.5) * s / 9.0 - s / 2, 0.0005);
            wrong += (kInner11[r][c] == '#') != (g < 0.5) || (g > 0.05 && g < 0.95) ? 1 : 0;
        }
    }
    CHECK(wrong == 0);
    // The inner marker's black square ends where its size says, inside the white ring.
    const double edge = l.inner_u_m + l.inner_size_m / 2;
    CHECK(tex->sample(edge - 0.002, l.inner_v_m, 0.0005) < 0.05);
    CHECK(tex->sample(edge + 0.002, l.inner_v_m, 0.0005) > 0.95);
    CHECK(tex->sample(l.inner_u_m + l.cell_size_m / 2 - 0.001, l.inner_v_m, 0.0005) > 0.95); // still the cell
    CHECK(tex->sample(l.inner_u_m + l.cell_size_m / 2 + 0.002, l.inner_v_m, 0.0005) <
          0.05); // the outer's black
    // The plate's border is white; a pixel the size of the whole marker sees its mean; garbage in
    // is white out.
    CHECK(tex->sample(0.51, 0.0, 0.001) > 0.95);
    const double mean = tex->sample(0.0, 0.0, 1.0);
    CHECK(mean > 0.15 && mean < 0.6);
    CHECK(tex->sample(0.0, 0.0, 1e6) > 0.99);
    CHECK(tex->sample(NAN, 0.0, 0.001) == 1.0);
    CHECK(tex->sample(0.3, 0.3, -1.0) == tex->sample(0.3, 0.3, 1e-9)); // no footprint: the finest

    // The image's own inner size changes nothing inside the inner marker.
    DockMarkerSpec native;
    native.inner_size_m = 349.0 / 3937.0;
    const auto same = asset(native);
    CHECK(same != nullptr);
    if (same) {
        // Stretched by exactly 1 onto whole texels: no texel of the cell blurred to grey, and the
        // inner square where the generator put it (px 1870 to 2218).
        int grey = 0;
        for (int y = 1826; y < 2263; ++y) {
            for (int x = 1826; x < 2263; ++x) {
                grey += same->texel(x, y) != 0 && same->texel(x, y) != 255 ? 1 : 0;
            }
        }
        CHECK(grey == 0);
        CHECK(same->texel(1870, 1870) == 0 && same->texel(1869, 1900) == 255 &&
              same->texel(2218, 2218) == 0 && same->texel(2219, 2000) == 255);
    }
}

void test_bad_images() {
    std::string error;
    DockMarkerSpec spec;
    CHECK(render::MarkerTexture::from_png(nullptr, 0, spec, error) == nullptr && !error.empty());
    const uint8_t junk[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    CHECK(render::MarkerTexture::from_png(junk, sizeof(junk), spec, error) == nullptr &&
          error.find("PNG") != std::string::npos);

    std::vector<uint8_t> white(100 * 100, 255);
    CHECK(render::MarkerTexture::build(white.data(), 100, 90, spec, error) == nullptr); // not square
    CHECK(render::MarkerTexture::build(nullptr, 100, 100, spec, error) == nullptr);
    CHECK(render::MarkerTexture::build(white.data(), 100, 100, spec, error) == nullptr &&
          error.find("no square") != std::string::npos);
    // A plain black square: no inner marker on a white cell.
    std::vector<uint8_t> solid = white;
    for (int y = 5; y < 95; ++y) {
        std::fill_n(solid.begin() + y * 100 + 5, 90, uint8_t{0});
    }
    CHECK(render::MarkerTexture::build(solid.data(), 100, 100, spec, error) == nullptr &&
          error.find("inner marker") != std::string::npos);
    // A black rectangle is not a marker.
    std::vector<uint8_t> wide = white;
    for (int y = 40; y < 60; ++y) {
        std::fill_n(wide.begin() + y * 100 + 5, 90, uint8_t{0});
    }
    CHECK(render::MarkerTexture::build(wide.data(), 100, 100, spec, error) == nullptr);

    // Sizes that cannot be drawn.
    DockMarkerSpec bad;
    bad.inner_size_m = 0.2;
    CHECK(!bad.invalid().empty());
    CHECK(render::MarkerTexture::build(white.data(), 100, 100, bad, error) == nullptr &&
          error == bad.invalid());
    bad.inner_size_m = 0.0;
    CHECK(!bad.invalid().empty());
    bad = DockMarkerSpec{};
    bad.outer_size_m = 0.01;
    CHECK(!bad.invalid().empty());
    CHECK(DockMarkerSpec{}.invalid().empty());
    // Over 95 % of a cell is refused up front, so a size that starts the server always fits the
    // committed image's whole-pixel cell; the largest that passes builds.
    DockMarkerSpec tight;
    tight.inner_size_m = 0.1111;
    CHECK(!tight.invalid().empty());
    CHECK(render::MarkerTexture::from_png(render::dock_marker_png(), render::dock_marker_png_size(), tight,
                                          error) == nullptr &&
          error == tight.invalid());
    CHECK(render::dock_marker_texture(tight) == nullptr); // logged once, cached as a failure
    CHECK(render::dock_marker_texture(tight) == nullptr);
    DockMarkerSpec widest;
    widest.inner_size_m = 1.0 / 9.0 * DockMarkerSpec::kMaxInnerCellFraction;
    CHECK(widest.invalid().empty() && asset(widest) != nullptr);

    // A small generator-like image (a 99 px square, cells of 11 px, an 8 px inner marker 1 px in
    // from its cell's edge): it builds, but not with an inner marker stretched past its cell —
    // the whole-pixel layout puts this one half a pixel off the cell's centre.
    std::vector<uint8_t> small(109 * 109, 255);
    for (int y = 5; y < 104; ++y) {
        std::fill_n(small.begin() + y * 109 + 5, 99, uint8_t{0});
    }
    for (int y = 49; y < 60; ++y) {
        std::fill_n(small.begin() + y * 109 + 49, 11, uint8_t{255});
    }
    for (int y = 50; y < 58; ++y) {
        std::fill_n(small.begin() + y * 109 + 50, 8, uint8_t{0});
    }
    DockMarkerSpec modest;
    modest.inner_size_m = 0.08;
    const auto built = render::MarkerTexture::build(small.data(), 109, 109, modest, error);
    CHECK(built != nullptr && built->size() == 128 && built->levels() == 8);
    CHECK(render::MarkerTexture::build(small.data(), 109, 109, widest, error) == nullptr &&
          error.find("does not fit") != std::string::npos);
    CHECK(render::dock_marker_texture(DockMarkerSpec{}) != nullptr);
    CHECK(render::dock_marker_texture(DockMarkerSpec{}) == render::dock_marker_texture(DockMarkerSpec{}));
}

void test_pose_math() {
    for (const double yaw_deg : {0.0, 37.0, 90.0, -135.0, 270.0}) {
        const DockPose p{12.5, -7.25, yaw_deg * M_PI / 180.0};
        const auto ne = render::dock_to_ned(p, 0.3, -0.2);
        const auto uv = render::ned_to_dock(p, ne[0], ne[1]);
        CHECK(std::fabs(uv[0] - 0.3) < 1e-12 && std::fabs(uv[1] + 0.2) < 1e-12);
    }
    // Upright with north up: the image's top-left corner is north-west, then clockwise.
    const auto c = render::dock_corners_ned(DockPose{}, 0.0, 0.0, 1.0);
    CHECK(c[0][0] == 0.5 && c[0][1] == -0.5);
    CHECK(c[1][0] == 0.5 && c[1][1] == 0.5);
    CHECK(c[2][0] == -0.5 && c[2][1] == 0.5);
    CHECK(c[3][0] == -0.5 && c[3][1] == -0.5);
    // Turned to face east: the top edge is the east edge, the top-left corner north-east.
    const auto e = render::dock_corners_ned(DockPose{0.0, 0.0, M_PI / 2}, 0.0, 0.0, 1.0);
    CHECK(std::fabs(e[0][0] - 0.5) < 1e-12 && std::fabs(e[0][1] - 0.5) < 1e-12);
    CHECK(std::fabs(e[3][0] - 0.5) < 1e-12 && std::fabs(e[3][1] + 0.5) < 1e-12);

    // lat/lon -> NED is geo_from_ned backwards, across the antimeridian too.
    const core::GeoOrigin home{42.140389, 24.764549, 160.0};
    const auto g = core::geo_from_ned(home, 123.4, -567.8, 0.0);
    const auto back = core::ned_from_geo(home, g.lat_deg, g.lon_deg);
    CHECK(std::fabs(back.north_m - 123.4) < 1e-6 && std::fabs(back.east_m + 567.8) < 1e-6);
    const core::GeoOrigin dateline{0.0, 179.9999, 0.0};
    const auto w = core::geo_from_ned(dateline, 0.0, 100.0, 0.0);
    CHECK(std::fabs(core::ned_from_geo(dateline, w.lat_deg, w.lon_deg).east_m - 100.0) < 1e-6);
}

render::StillRequest frame_request() {
    render::StillRequest r;
    r.width = 640;
    r.height = 360;
    r.hfov_deg = 81.0;
    return r;
}

void maybe_write(const render::Image &img, const std::string &name) {
    const char *dir = std::getenv("SKYSIM_DOCK_FRAMES");
    if (dir == nullptr || *dir == '\0') {
        return;
    }
    std::ofstream f(std::string(dir) + "/" + name + ".ppm", std::ios::binary);
    f << "P6\n" << img.width << " " << img.height << "\n255\n";
    f.write(reinterpret_cast<const char *>(img.rgb.data()), static_cast<std::streamsize>(img.rgb.size()));
}

double luma(const render::Image &img, int x, int y) {
    const size_t i = (static_cast<size_t>(y) * img.width + static_cast<size_t>(x)) * 3;
    return (img.rgb[i] + img.rgb[i + 1] + img.rgb[i + 2]) / 3.0;
}

render::Image img_upright(const render::RasterScene &scene, const render::Vec3 &eye, const render::Quat &q,
                          const render::StillRequest &req, const render::GroundMarkers &markers) {
    return render::render_still(scene, eye, q, req, nullptr, true, nullptr, &markers);
}

void test_frames() {
    render::RasterScene scene;
    scene.build({});
    const render::StillRequest req = frame_request();
    const render::CameraConfig cfg = render::still_camera_config(req);
    const render::Quat nadir = render::camera_quat_from_angles(0.0, -90.0, 0.0);
    const render::GroundMarkers markers{render::GroundMarker{7, DockPose{}, DockMarkerSpec{}, nullptr}};
    const render::GroundMarkers none;
    const render::GroundMarkers far{
        render::GroundMarker{7, DockPose{5000.0, 0.0, 0.0}, DockMarkerSpec{}, nullptr}};
    const auto tex = render::dock_marker_texture(DockMarkerSpec{});
    CHECK(tex != nullptr);
    if (!tex) {
        return;
    }

    for (const double h : {30.0, 15.0, 5.0, 2.0, 1.2, 0.7}) {
        const render::Vec3 eye{0.0, 0.0, -h};
        const render::Camera camera = render::Camera::with_attitude(cfg, eye, nadir);
        std::vector<int32_t> surface;
        const render::Image img =
            render::render_still(scene, eye, nadir, req, nullptr, true, &surface, &markers);
        maybe_write(img, "nadir_" + std::to_string(static_cast<int>(h * 10)) + "dm");
        // Every outer cell and every inner cell the frame resolves (2.5 px or more, so the pixel
        // under a cell's centre lies wholly in it), at its centre: dark where the generator drew
        // black, bright where it drew white.
        int checked = 0;
        int wrong = 0;
        const auto probe = [&](double u, double v, bool black, double cell_m) {
            double px = 0.0;
            double py = 0.0;
            if (cell_m * camera.focal_px() / h < 2.5 || !camera.project({-v, u, 0.0}, px, py) || px < 1 ||
                py < 1 || px >= cfg.width - 1 || py >= cfg.height - 1) {
                return;
            }
            const double y = luma(img, static_cast<int>(px), static_cast<int>(py));
            ++checked;
            wrong += black ? (y > 40.0 ? 1 : 0) : (y < 150.0 ? 1 : 0);
        };
        const render::DockLayout &l = tex->layout();
        for (int r = 0; r < 9; ++r) {
            for (int c = 0; c < 9; ++c) {
                if (r != 4 || c != 4) {
                    probe((c + 0.5) / 9.0 - 0.5, (r + 0.5) / 9.0 - 0.5, kOuter228[r][c] == '#', 1.0 / 9.0);
                }
                const double s = l.inner_size_m;
                probe(l.inner_u_m + (c + 0.5) * s / 9.0 - s / 2, l.inner_v_m + (r + 0.5) * s / 9.0 - s / 2,
                      kInner11[r][c] == '#', s / 9.0);
            }
        }
        CHECK(h > 15.0 || checked >= 40);
        CHECK(wrong == 0);
        // The outer square's left edge, along the row through cell (5, 0) of its black border,
        // lands where the camera model projects it — to a pixel: walk left from inside the border
        // to the plate's white.
        double edge_px = 0.0;
        double inside_px = 0.0;
        double row_py = 0.0;
        if (h <= 5.0 && camera.project({-0.15, -0.5, 0.0}, edge_px, row_py) &&
            camera.project({-0.15, -0.45, 0.0}, inside_px, row_py) && edge_px > 2) {
            const int row = static_cast<int>(row_py);
            int x = static_cast<int>(inside_px);
            while (x > 0 && luma(img, x - 1, row) < 110.0) {
                --x;
            }
            CHECK(std::fabs(x - edge_px) <= 1.0);
        }
        // Marker pixels are still ground to whoever asks what a pixel shows.
        CHECK(surface[static_cast<size_t>(cfg.height / 2) * cfg.width + cfg.width / 2] ==
              render::kSurfaceGround);

        // No marker, no marker in range, an empty list: the same picture, byte for byte.
        const render::Image bare =
            render::render_still(scene, eye, nadir, req, nullptr, true, nullptr, nullptr);
        CHECK(render::render_still(scene, eye, nadir, req, nullptr, true, nullptr, &none).rgb == bare.rgb);
        CHECK(render::render_still(scene, eye, nadir, req, nullptr, true, nullptr, &far).rgb == bare.rgb);
        CHECK(img.rgb != bare.rgb);
    }

    // Turned 90 degrees, the marker turns with it: what was the top-left cell is now top-right.
    const render::GroundMarkers turned{
        render::GroundMarker{7, DockPose{0.0, 0.0, M_PI / 2}, DockMarkerSpec{}, nullptr}};
    const render::Vec3 eye{0.0, 0.0, -2.0};
    const render::Camera camera = render::Camera::with_attitude(cfg, eye, nadir);
    const render::Image img = render::render_still(scene, eye, nadir, req, nullptr, true, nullptr, &turned);
    maybe_write(img, "nadir_yaw90");
    // Cell (1,1) of 228 is white, (1,3) black. Rotated to face east, cell (r, c) lies at
    // u' = -(v), v' = u in the upright frame: probe both through dock_to_ned.
    for (const auto &[r, c] : {std::pair{1, 1}, std::pair{1, 3}, std::pair{5, 6}, std::pair{2, 5}}) {
        const auto ne = render::dock_to_ned(turned[0].pose, (c + 0.5) / 9.0 - 0.5, (r + 0.5) / 9.0 - 0.5);
        double px = 0.0;
        double py = 0.0;
        CHECK(camera.project({ne[0], ne[1], 0.0}, px, py));
        const double y = luma(img, static_cast<int>(px), static_cast<int>(py));
        CHECK(kOuter228[r][c] == '#' ? y < 40.0 : y > 150.0);
    }

    // A camera too low to see the whole plate (corners behind it) still draws the marker under
    // it, and one below the ground draws none.
    const render::Vec3 low{0.0, 0.0, -0.05};
    const render::Quat ahead = render::camera_quat_from_angles(0.0, -10.0, 0.0);
    const render::Camera low_cam = render::Camera::with_attitude(cfg, low, ahead);
    const render::MarkerView view(&markers, low_cam);
    CHECK(view.size() == 1);
    const render::Image low_img =
        render::render_still(scene, low, ahead, req, nullptr, true, nullptr, &markers);
    CHECK(low_img.rgb != render::render_still(scene, low, ahead, req, nullptr, true, nullptr, nullptr).rgb);
    CHECK(render::MarkerView(&markers, render::Camera::with_attitude(cfg, {0.0, 0.0, 0.5}, nadir)).empty());
    CHECK(render::MarkerView(nullptr, camera).empty());
    // Looking away from it: in range, off screen.
    const render::Quat up = render::camera_quat_from_angles(0.0, -10.0, 0.0);
    CHECK(render::MarkerView(&markers, render::Camera::with_attitude(cfg, {20.0, 0.0, -2.0}, up)).empty());
    // A scanned tile's ground mesh lying on the ground carries the marker like the ground plane
    // does; a face raised above the ground hides it.
    const auto quad = [](double z) {
        auto t = [](core::Vec3 a, core::Vec3 b, core::Vec3 c) {
            core::World::Triangle tri;
            tri.a = a;
            tri.b = b;
            tri.c = c;
            return tri;
        };
        return std::vector<core::World::Triangle>{t({-5, -5, z}, {5, -5, z}, {5, 5, z}),
                                                  t({-5, -5, z}, {5, 5, z}, {-5, 5, z})};
    };
    render::RasterScene ground_mesh;
    ground_mesh.build(quad(0.0));
    render::RasterScene table;
    table.build(quad(-0.5));
    std::vector<int32_t> mesh_surface;
    const render::Image on_mesh =
        render::render_still(ground_mesh, eye, nadir, req, nullptr, true, &mesh_surface, &markers);
    CHECK(mesh_surface[static_cast<size_t>(cfg.height / 2) * cfg.width + cfg.width / 2] >= 0); // the mesh won
    // The same picture as on the bare ground plane, to rounding (the mesh's plane is hit by a
    // different formula): every byte within 2.
    const render::Image on_plane = img_upright(scene, eye, nadir, req, markers);
    int worst = 0;
    for (size_t i = 0; i < on_mesh.rgb.size() && on_mesh.rgb.size() == on_plane.rgb.size(); ++i) {
        worst =
            std::max(worst, std::abs(static_cast<int>(on_mesh.rgb[i]) - static_cast<int>(on_plane.rgb[i])));
    }
    CHECK(on_mesh.rgb.size() == on_plane.rgb.size() && worst <= 2);
    CHECK(render::render_still(table, eye, nadir, req, nullptr, true, nullptr, &markers).rgb ==
          render::render_still(table, eye, nadir, req, nullptr, true, nullptr, nullptr).rgb);

    // A marker given its own texture draws from it.
    const render::GroundMarkers own{render::GroundMarker{7, DockPose{}, DockMarkerSpec{}, tex}};
    CHECK(render::render_still(scene, eye, nadir, req, nullptr, true, nullptr, &own).rgb ==
          render::render_still(scene, eye, nadir, req, nullptr, true, nullptr, &markers).rgb);
    double albedo = -1.0;
    CHECK(render::MarkerView(&own, camera).albedo(0, 0, 50.0, 50.0, 0.01, albedo) == 0.0 && albedo == -1.0);
}

void test_board() {
    render::DockBoard board{DockMarkerSpec{}};
    CHECK(board.spec().outer_size_m == 1.0);
    const auto empty = board.markers();
    CHECK(empty != nullptr && empty->empty());
    board.add(3, 0.0, 10.0);
    board.add(4, 0.0, 15.0);
    const auto two = board.markers();
    CHECK(two != empty && two->size() == 2);
    CHECK(board.markers() == two); // unchanged: the same list
    const auto e = board.get(3);
    CHECK(e && e->at_home && e->pose.east_m == 10.0 && e->pose.yaw_rad == 0.0);
    CHECK(!board.get(9) && !board.set(9, DockPose{}) && !board.reset(9));
    const auto moved = board.set(3, DockPose{1.0, 2.0, 0.5});
    CHECK(moved && !moved->at_home && moved->pose.north_m == 1.0 && moved->home.east_m == 10.0);
    CHECK(board.markers() != two);
    const auto list = board.markers();
    CHECK(list->size() == 2 && (*list)[0].vehicle_id == 3 && (*list)[0].pose.yaw_rad == 0.5);
    // A patch replaces only what it names, in one step.
    render::DockPatch yaw_only;
    yaw_only.yaw_rad = 1.25;
    const auto turned = board.update(3, yaw_only);
    CHECK(turned && turned->pose.north_m == 1.0 && turned->pose.east_m == 2.0 &&
          turned->pose.yaw_rad == 1.25);
    const auto same = board.markers();
    CHECK(board.update(3, render::DockPatch{}) && board.markers() == same); // nothing named: no change
    render::DockPatch north_only;
    north_only.north_m = -4.0;
    CHECK(board.update(3, north_only)->pose.east_m == 2.0 && board.get(3)->pose.north_m == -4.0);
    CHECK(!board.update(9, north_only));
    const auto back = board.reset(3);
    CHECK(back && back->at_home && back->pose.north_m == 0.0 && back->pose.east_m == 10.0);
    board.remove(4);
    board.remove(4); // twice is fine
    CHECK(board.markers()->size() == 1 && !board.get(4));
}

// The pilot camera re-sends a parked vehicle's last picture until something in it changes — and
// a dock moving is such a change.
void test_render_service_redraws_on_move() {
    auto scene = std::make_shared<render::RasterScene>();
    scene->build({});
    auto board = std::make_shared<render::DockBoard>(DockMarkerSpec{});
    board->add(5, 0.0, 0.0);
    render::RenderService::Config cfg;
    cfg.camera.width = 160;
    cfg.camera.height = 90;
    cfg.camera.fov_deg = 81.0;
    cfg.scene = scene;
    cfg.docks = board;
    cfg.fps = 50.0;
    cfg.threads = 1;
    cfg.background_priority = false;
    render::RenderService service(cfg);
    render::RenderService::Pose pose;
    pose.vehicle_id = 5;
    pose.position_ned = {0.0, 0.0, -3.0};
    pose.gimbal = true;
    pose.gimbal_pitch_rad = -M_PI / 2;
    service.publish_poses({pose});

    const auto wait = std::chrono::milliseconds(3000);
    const auto first = service.frame(5, 0, wait);
    const auto again = service.frame(5, first.seq, wait);
    CHECK(!first.jpeg.empty() && again.seq > first.seq && again.jpeg == first.jpeg);
    CHECK(service.stats().frames_drawn == 1);

    board->set(5, DockPose{0.6, 0.0, 0.0});
    auto frame = again;
    const auto deadline = std::chrono::steady_clock::now() + wait;
    while (service.stats().frames_drawn < 2 && std::chrono::steady_clock::now() < deadline) {
        frame = service.frame(5, frame.seq, std::chrono::milliseconds(200));
    }
    CHECK(service.stats().frames_drawn == 2);
    frame = service.frame(5, frame.seq, wait);
    CHECK(frame.jpeg != first.jpeg);
}

// --dock-markers-visible (TASK-2609): exactly "own" or "all".
void test_visibility_names() {
    CHECK(render::parse_dock_visibility("own") == render::DockVisibility::kOwn);
    CHECK(render::parse_dock_visibility("all") == render::DockVisibility::kAll);
    for (const char *bad : {"", "Own", "ALL", "own ", " all", "none", "1", "own,all"}) {
        CHECK(!render::parse_dock_visibility(bad));
    }
    CHECK(std::string(render::dock_visibility_name(render::DockVisibility::kOwn)) == "own");
    CHECK(std::string(render::dock_visibility_name(render::DockVisibility::kAll)) == "all");
    CHECK(render::DockBoard{DockMarkerSpec{}}.visibility() == render::DockVisibility::kOwn); // the default
}

// What a vehicle's own cameras are given (TASK-2609): under kOwn its own dock alone, a list that
// keeps its pointer while other vehicles' docks come, move and go; under kAll every dock.
void test_board_seen_by() {
    render::DockBoard board{DockMarkerSpec{}};
    const auto nobody = board.seen_by(1);
    CHECK(nobody != nullptr && nobody->empty());
    board.add(1, 0.0, 0.0);
    board.add(2, 0.0, 5.0);
    board.add(3, 0.0, 10.0);
    CHECK(board.markers()->size() == 3);
    for (uint32_t id = 1; id <= 3; ++id) {
        const auto seen = board.seen_by(id);
        CHECK(seen && seen->size() == 1 && (*seen)[0].vehicle_id == id &&
              (*seen)[0].pose.east_m == 5.0 * (id - 1) && (*seen)[0].spec.outer_size_m == 1.0);
    }
    CHECK(board.seen_by(9) && board.seen_by(9)->empty()); // no dock of its own: nothing drawn
    const auto one = board.seen_by(1);
    CHECK(board.seen_by(1) == one); // unchanged: the same list

    // Another vehicle's dock moving, a new one arriving, one going: vehicle 1's list stands.
    const auto all_before = board.markers();
    board.set(2, DockPose{3.0, 4.0, 0.5});
    board.add(4, 0.0, 15.0);
    board.remove(3);
    CHECK(board.seen_by(1) == one);
    CHECK(board.markers() != all_before && board.markers()->size() == 3);
    CHECK(board.seen_by(2)->size() == 1 && (*board.seen_by(2))[0].pose.north_m == 3.0 &&
          (*board.seen_by(2))[0].pose.yaw_rad == 0.5);
    CHECK(board.seen_by(3)->empty());
    CHECK(board.seen_by(4)->size() == 1 && (*board.seen_by(4))[0].pose.east_m == 15.0);

    // Its own dock changing does change it, every way it can.
    render::DockPatch east;
    east.east_m = 0.4;
    CHECK(board.update(1, east));
    const auto patched = board.seen_by(1);
    CHECK(patched != one && (*patched)[0].pose.east_m == 0.4);
    CHECK(board.update(1, render::DockPatch{}) && board.seen_by(1) == patched); // nothing named
    CHECK(board.reset(1));
    const auto home = board.seen_by(1);
    CHECK(home != patched && (*home)[0].pose.east_m == 0.0);
    board.remove(1);
    CHECK(board.seen_by(1)->empty());
    board.add(1, 0.0, 0.0); // a vehicle spawned again at the same instance: its dock is back
    CHECK(board.seen_by(1)->size() == 1 && board.seen_by(1) != home);

    // kAll: every vehicle's cameras get every dock, the very list markers() gives.
    render::DockBoard all{DockMarkerSpec{}, render::DockVisibility::kAll};
    CHECK(all.visibility() == render::DockVisibility::kAll);
    CHECK(all.seen_by(1) == all.markers() && all.seen_by(1)->empty());
    all.add(1, 0.0, 0.0);
    all.add(2, 0.0, 5.0);
    CHECK(all.seen_by(1) == all.markers() && all.seen_by(2) == all.markers() &&
          all.seen_by(9) == all.markers());
    CHECK(all.seen_by(1)->size() == 2);
}

// The pilot camera of vehicle 1, hovering 30 m over its dock with two more docks 5 and 10 m east of
// it in shot, draws its own dock alone — the very picture of its own dock with no other — unless
// every camera is to see every dock. And under kOwn a parked picture stays current while another
// vehicle's dock moves: nothing it shows has changed.
void test_render_service_own_dock() {
    auto scene = std::make_shared<render::RasterScene>();
    scene->build({});
    for (const render::DockVisibility visibility :
         {render::DockVisibility::kOwn, render::DockVisibility::kAll}) {
        auto board = std::make_shared<render::DockBoard>(DockMarkerSpec{}, visibility);
        board->add(1, 0.0, 0.0);
        board->add(2, 0.0, 5.0);
        board->add(3, 0.0, 10.0);
        std::mutex mutex;
        std::shared_ptr<const render::Image> latest;
        render::RenderService::Config cfg;
        cfg.camera.width = 640;
        cfg.camera.height = 360;
        cfg.camera.fov_deg = 81.0;
        cfg.scene = scene;
        cfg.docks = board;
        cfg.fps = 50.0;
        cfg.threads = 2;
        cfg.background_priority = false;
        cfg.video_sink = [&](uint32_t id, std::shared_ptr<const render::Image> image, double,
                             std::chrono::steady_clock::time_point) {
            if (id == 1) {
                std::lock_guard<std::mutex> lock(mutex);
                latest = std::move(image);
            }
        };
        render::RenderService service(cfg);
        std::vector<render::RenderService::Pose> poses(3);
        for (uint32_t k = 0; k < 3; ++k) {
            poses[k].vehicle_id = k + 1;
            poses[k].position_ned = {0.0, 5.0 * k, -0.1};
        }
        poses[0].position_ned = {0.0, 0.0, -30.0};
        poses[0].gimbal = true;
        poses[0].gimbal_pitch_rad = -M_PI / 2;
        service.publish_poses(poses);

        const auto picture = [&](uint64_t drawn) {
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
            while (std::chrono::steady_clock::now() < deadline) {
                service.want_video(1);
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    if (latest && service.stats().frames_drawn >= drawn) {
                        return latest;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            std::lock_guard<std::mutex> lock(mutex);
            return latest;
        };
        const auto first = picture(1);
        CHECK(first != nullptr);
        if (!first) {
            continue;
        }
        const render::Camera camera = render::Camera::on_gimbal(cfg.camera, poses[0].position_ned,
                                                                poses[0].quat_ned_frd, 0.0, -M_PI / 2);
        const render::Rasterizer raster(cfg.camera);
        const render::GroundMarkers own_only{
            render::GroundMarker{1, DockPose{0.0, 0.0, 0.0}, DockMarkerSpec{}, nullptr}};
        const render::Image own = raster.render(*scene, camera, nullptr, nullptr, true, &own_only);
        const render::Image every =
            raster.render(*scene, camera, nullptr, nullptr, true, board->markers().get());
        // Both other docks are in shot from 30 m (the frame spans 51 m of ground across): what is
        // drawn of them is what the two settings differ by.
        CHECK(render::MarkerView(board->markers().get(), camera).size() == 3);
        CHECK(own.rgb != every.rgb);
        const bool own_mode = visibility == render::DockVisibility::kOwn;
        CHECK(first->rgb == (own_mode ? own.rgb : every.rgb));
        maybe_write(*first, std::string("service_") + render::dock_visibility_name(visibility) + "_30m");

        // Another vehicle's dock moves: under kOwn vehicle 1's picture is re-sent, not redrawn.
        const uint64_t drawn = service.stats().frames_drawn;
        board->set(2, DockPose{2.0, 5.0, 0.0});
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        const auto after = picture(own_mode ? drawn : drawn + 1);
        CHECK(after != nullptr);
        if (own_mode) {
            CHECK(service.stats().frames_drawn == drawn && after == first);
        } else {
            CHECK(service.stats().frames_drawn > drawn && after && after->rgb != first->rgb &&
                  after->rgb ==
                      raster.render(*scene, camera, nullptr, nullptr, true, board->markers().get()).rgb);
        }
    }
}

} // namespace

int main() {
    test_embedded_image();
    test_layout();
    test_bad_images();
    test_pose_math();
    test_frames();
    test_board();
    test_render_service_redraws_on_move();
    test_visibility_names();
    test_board_seen_by();
    test_render_service_own_dock();
    if (g_failures != 0) {
        std::printf("%d failure(s)\n", g_failures);
        return 1;
    }
    std::printf("test_dock_marker: all passed\n");
    return 0;
}
