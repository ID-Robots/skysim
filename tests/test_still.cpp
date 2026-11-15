// Survey still camera: the world texture, the scene it is drawn from, the still renderer, the
// service that bounds it, and the HTTP routes that serve it — against the facts a
// photogrammetry pipeline depends on:
//
//   * the same pose and seed give the same bytes (determinism);
//   * a world point has the same colour in two overlapping frames (texture consistency);
//   * a nadir frame's centre is the point under the vehicle, gimbal or not;
//   * the pose headers say exactly where the frame was taken from;
//   * every input is bounded, and a busy renderer says so instead of queueing.
#include <httplib.h>

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#include <stb_image.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "api/control_server.h"
#include "core/geo.h"
#include "core/thread_pool.h"
#include "render/jpeg.h"
#include "render/raster.h"
#include "render/render_service.h"
#include "render/renderer.h"
#include "render/scene.h"
#include "render/shading.h"
#include "render/still.h"
#include "render/texture.h"
#include "terrain/cook.h"
#include "vehicle/gimbal.h"

namespace {

int g_failures = 0;

#define CHECK(cond)                                                                                          \
    do {                                                                                                     \
        if (!(cond)) {                                                                                       \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                      \
            ++g_failures;                                                                                    \
        }                                                                                                    \
    } while (0)

constexpr int kPort = 18652; // outside the ports the other suites hold (see tests/README notes)

using skysim::core::Quat;
using skysim::core::Vec3;
namespace render = skysim::render;

// An axis-aligned box in the OBJ's ENU frame (x east, y north, z up), outward winding.
void write_box(std::ofstream &f, int base, double e0, double e1, double n0, double n1, double h) {
    f << "v " << e0 << " " << n0 << " 0\nv " << e1 << " " << n0 << " 0\nv " << e1 << " " << n1 << " 0\nv "
      << e0 << " " << n1 << " 0\n";
    f << "v " << e0 << " " << n0 << " " << h << "\nv " << e1 << " " << n0 << " " << h << "\nv " << e1 << " "
      << n1 << " " << h << "\nv " << e0 << " " << n1 << " " << h << "\n";
    const int quads[6][4] = {{1, 2, 6, 5}, {2, 3, 7, 6}, {3, 4, 8, 7},
                             {4, 1, 5, 8}, {5, 6, 7, 8}, {4, 3, 2, 1}};
    for (const auto &q : quads) {
        f << "f " << base + q[0] << " " << base + q[1] << " " << base + q[2] << "\n";
        f << "f " << base + q[0] << " " << base + q[2] << " " << base + q[3] << "\n";
    }
}

// Two buildings in two tiles: a 30 m tower (north 30..70, east -20..20) and a 2 m marker
// (north 0..2, east 100..102, 3 m tall) to aim nadir frames at.
std::filesystem::path make_tiles() {
    const auto dir = std::filesystem::temp_directory_path() / "skysim_still_tiles";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    {
        std::ofstream f(dir / "tower.obj");
        write_box(f, 0, -20, 20, 30, 70, 30);
    }
    {
        std::ofstream f(dir / "marker.obj");
        write_box(f, 0, 100, 102, 0, 2, 3);
    }
    std::string err;
    CHECK(skysim::terrain::cook_obj_tile(dir / "tower.obj", dir / "tower.jshape", nullptr, &err));
    CHECK(skysim::terrain::cook_obj_tile(dir / "marker.obj", dir / "marker.jshape", nullptr, &err));
    return dir;
}

int byte_at(const render::Image &img, int x, int y, int c) {
    return img.rgb[(static_cast<size_t>(y) * img.width + x) * 3 + c];
}

render::StillRequest small_request(int w = 320, int h = 240) {
    render::StillRequest r;
    r.width = w;
    r.height = h;
    return r;
}

Quat yaw_quat(double deg) {
    const double half = deg * M_PI / 360.0;
    return {std::cos(half), 0.0, 0.0, std::sin(half)};
}

// Vehicle attitude from Z-Y-X Euler angles (degrees), FRD -> NED.
Quat attitude(double yaw, double pitch, double roll) {
    return render::camera_quat_from_angles(yaw, pitch, roll);
}

void test_primitives() {
    using namespace render;
    // Hashes are pure: same input, same output, and they spread.
    CHECK(hash32(12345) == hash32(12345));
    CHECK(hash32(1) != hash32(2));
    CHECK(hash_cell(3, -4, 9) == hash_cell(3, -4, 9));
    CHECK(hash_cell(3, -4, 9) != hash_cell(-4, 3, 9));

    // Value noise stays in [0, 1] and is continuous across a lattice line.
    double lo = 1.0, hi = 0.0;
    for (int i = 0; i < 2000; ++i) {
        const double v = value_noise(i * 0.137, i * -0.291, 7);
        lo = std::min(lo, v);
        hi = std::max(hi, v);
    }
    CHECK(lo >= 0.0 && hi <= 1.0 && hi - lo > 0.5);
    CHECK(std::fabs(value_noise(4.999999, 1.5, 3) - value_noise(5.000001, 1.5, 3)) < 1e-4);

    // A pixel far bigger than every octave sees only the average.
    CHECK(fbm(10.0, 20.0, 1.0, 6, 0.5, 100.0, 1) == 0.5);
    // A small one sees detail.
    CHECK(fbm(10.3, 20.7, 1.0, 6, 0.5, 0.001, 1) != 0.5);

    // Box-filtered stripes: inside, outside, half on the edge, and in between.
    CHECK(band_coverage(0.0, 1.0, 0.1) == 1.0);
    CHECK(band_coverage(2.0, 1.0, 0.1) == 0.0);
    CHECK(std::fabs(band_coverage(1.0, 1.0, 0.1) - 0.5) < 1e-9);
    // A stripe thinner than the pixel covers its share of it.
    CHECK(std::fabs(band_coverage(0.0, 0.05, 1.0) - 0.1) < 1e-9);
    // A repeating stripe seen from far away is its duty cycle.
    CHECK(std::fabs(pulse_coverage(0.3, 1.0, 0.25, 50.0) - 0.25) < 1e-9);
    CHECK(std::fabs(pulse_coverage(0.1, 1.0, 0.25, 0.01) - 1.0) < 1e-9);
    CHECK(pulse_coverage(0.6, 1.0, 0.25, 0.01) == 0.0);
}

void test_texture() {
    using render::WorldTexture;
    const WorldTexture tex(1);
    const WorldTexture same(1);
    const WorldTexture other(2);
    CHECK(tex.seed() == 1);

    // Pure in the world point: same point, same seed, same colour; another seed, another world.
    double a[3], b[3], c[3];
    int differ = 0;
    for (int i = 0; i < 200; ++i) {
        const double n = i * 3.71 - 300.0;
        const double e = i * -2.33 + 120.0;
        tex.ground(n, e, 0.05, a);
        same.ground(n, e, 0.05, b);
        other.ground(n, e, 0.05, c);
        CHECK(a[0] == b[0] && a[1] == b[1] && a[2] == b[2]);
        differ += (a[0] != c[0] || a[1] != c[1] || a[2] != c[2]) ? 1 : 0;
        for (const double v : a) {
            CHECK(v >= 0.0 && v <= 1.0);
        }
    }
    CHECK(differ > 150);

    // The road grid: a carriageway is dark and neutral, its lane line is white, a kerb is
    // light — and all of it stays put (it is laid out in world coordinates).
    double road[3];
    tex.ground(WorldTexture::kRoadOffsetM + 2.0, 20.0, 0.05, road); // east-west road, off the line
    CHECK(road[0] < 0.35 && std::fabs(road[0] - road[2]) < 0.06);
    double line[3];
    tex.ground(WorldTexture::kRoadOffsetM, 19.5, 0.02, line); // its dashed centre line, mid-dash
    CHECK(line[0] > 0.45);
    double kerb[3];
    tex.ground(WorldTexture::kRoadOffsetM + WorldTexture::kRoadHalfWidthM + 0.12, 20.0, 0.01, kerb);
    CHECK(kerb[0] > road[0] + 0.2);

    // Land cover between the roads varies: not one flat colour across a block.
    double lo = 1.0, hi = 0.0, green = 0.0;
    for (int i = 0; i < 400; ++i) {
        double g[3];
        tex.ground(10.0 + (i % 20) * 1.9, -10.0 - (i / 20) * 1.9, 0.05, g);
        lo = std::min(lo, g[0] + g[1] + g[2]);
        hi = std::max(hi, g[0] + g[1] + g[2]);
        green = std::max(green, g[1] - g[2]);
    }
    CHECK(hi - lo > 0.3);

    // Fine detail fades with the footprint: from far away the same point is smoother.
    double fine_lo = 3.0, fine_hi = 0.0, far_lo = 3.0, far_hi = 0.0;
    for (int i = 0; i < 100; ++i) {
        double f1[3], f2[3];
        tex.ground(20.0 + i * 0.013, -20.0, 0.005, f1);
        tex.ground(20.0 + i * 0.013, -20.0, 2.0, f2);
        fine_lo = std::min(fine_lo, f1[0] + f1[1] + f1[2]);
        fine_hi = std::max(fine_hi, f1[0] + f1[1] + f1[2]);
        far_lo = std::min(far_lo, f2[0] + f2[1] + f2[2]);
        far_hi = std::max(far_hi, f2[0] + f2[1] + f2[2]);
    }
    CHECK(fine_hi - fine_lo > far_hi - far_lo);

    // Buildings: every wall style and roof type, at every storey, stays in gamut and is a
    // pure function of its inputs.
    render::SurfaceFrame wall;
    wall.kind = render::SurfaceFrame::Kind::kWall;
    wall.tangent_n = 0.0;
    wall.tangent_e = 1.0;
    wall.u_min = -20.0;
    wall.u_max = 20.0;
    wall.z_top = -30.0;
    wall.z_bottom = 0.0;
    render::SurfaceFrame roof;
    roof.kind = render::SurfaceFrame::Kind::kRoof;
    roof.n_min = 30.0;
    roof.n_max = 70.0;
    roof.e_min = -20.0;
    roof.e_max = 20.0;
    roof.z_top = roof.z_bottom = -30.0;
    render::SurfaceFrame under;
    under.kind = render::SurfaceFrame::Kind::kUnderside;
    int windows = 0;
    for (uint32_t key = 0; key < 64; ++key) {
        render::BuildingPoint bp;
        bp.building = key * 2654435761U;
        for (int i = 0; i < 300; ++i) {
            const double u = -19.9 + (i % 30) * 1.33;
            const double up = 0.1 + (i / 30) * 2.97;
            double w1[3], w2[3];
            bp.frame = &wall;
            tex.building({30.0, u, -up}, bp, 0.02, w1);
            tex.building({30.0, u, -up}, bp, 0.02, w2);
            CHECK(w1[0] == w2[0] && w1[1] == w2[1] && w1[2] == w2[2]);
            for (const double v : w1) {
                CHECK(v >= 0.0 && v <= 1.0);
            }
            // Dark glass is darker than any facade colour: count the window samples.
            windows += (w1[0] + w1[1] + w1[2] < 0.75) ? 1 : 0;

            bp.frame = &roof;
            bp.edge_distance_m = (i % 7) * 0.15;
            double r[3];
            tex.building({31.0 + (i % 30) * 1.3, -19.0 + (i / 30) * 3.9, -30.0}, bp, 0.02, r);
            for (const double v : r) {
                CHECK(v >= 0.0 && v <= 1.0);
            }
        }
        bp.frame = &under;
        double d[3];
        tex.building({50.0, 0.0, 0.0}, bp, 0.02, d);
        CHECK(d[0] == d[1] && d[1] == d[2]);
        bp.frame = nullptr;
        tex.building({50.0, 0.0, 0.0}, bp, 0.02, d);
        CHECK(d[0] == d[1]);
    }
    CHECK(windows > 1000);
}

void test_geo() {
    // ArduPilot's flat earth: 1000 m north is 1000 * 89.83204953368922e-7 degrees.
    const skysim::core::GeoOrigin home{42.0, 24.0, 160.0};
    const auto p = skysim::core::geo_from_ned(home, 1000.0, 0.0, -50.0);
    CHECK(std::fabs(p.lat_deg - (42.0 + 1000.0 * 89.83204953368922e-7)) < 1e-12);
    CHECK(p.lon_deg == 24.0);
    CHECK(p.alt_rel_m == 50.0 && p.alt_amsl_m == 210.0);
    // East is scaled by the cosine of the mid latitude.
    const auto q = skysim::core::geo_from_ned(home, 0.0, 1000.0, 0.0);
    CHECK(std::fabs(q.lon_deg - (24.0 + 1000.0 * 89.83204953368922e-7 / std::cos(42.0 * M_PI / 180.0))) <
          1e-12);
    // Longitude wraps; latitude does not leave the globe.
    const auto w = skysim::core::geo_from_ned({0.0, 179.9999, 0.0}, 0.0, 100.0, 0.0);
    CHECK(w.lon_deg < -179.99);
    const auto w2 = skysim::core::geo_from_ned({0.0, -179.9999, 0.0}, 0.0, -100.0, 0.0);
    CHECK(w2.lon_deg > 179.99);
    CHECK(skysim::core::geo_from_ned({89.9999, 0.0, 0.0}, 1000.0, 0.0, 0.0).lat_deg == 90.0);
    CHECK(skysim::core::geo_from_ned({0.0, -180.0, 0.0}, 0.0, 0.0, 0.0).lon_deg == 180.0); // (-180, 180]
    // An absurd offset wraps in one step rather than looping.
    const double far = skysim::core::geo_from_ned({0.0, 0.0, 0.0}, 0.0, 1e15, 0.0).lon_deg;
    CHECK(far > -180.0 && far <= 180.0);
    // Colour to byte: clamped, and NaN is black rather than undefined behaviour.
    CHECK(render::to_byte(-1.0) == 0 && render::to_byte(2.0) == 255 && render::to_byte(std::nan("")) == 0);
}

void test_scene(const render::RasterScene &scene) {
    CHECK(scene.triangle_count() == 24); // two boxes, no ground slab
    CHECK(scene.building_count() == 2);
    CHECK(std::fabs(scene.max_height() - 30.0) < 1e-3);

    // Tower roof: found, a roof, outline 20 m from its middle — and the roof's diagonal,
    // which runs through that middle, is not an outline edge.
    const int roof = scene.find_triangle({50.0, 0.0, -30.0});
    CHECK(roof >= 0);
    if (roof >= 0) {
        CHECK(scene.frame_of(static_cast<size_t>(roof)).kind == render::SurfaceFrame::Kind::kRoof);
        CHECK(std::fabs(scene.edge_distance(static_cast<size_t>(roof), {50.0, 0.0, -30.0}) - 20.0) < 1e-6);
        const int near_edge = scene.find_triangle({31.0, 0.5, -30.0});
        CHECK(near_edge >= 0 &&
              std::fabs(scene.edge_distance(static_cast<size_t>(near_edge), {31.0, 0.5, -30.0}) - 1.0) <
                  1e-6);
        const auto &f = scene.frame_of(static_cast<size_t>(roof));
        CHECK(f.n_min == 30.0 && f.n_max == 70.0 && f.e_min == -20.0 && f.e_max == 20.0);
    }
    const int marker = scene.find_triangle({1.0, 101.0, -3.0});
    CHECK(marker >= 0);
    if (roof >= 0 && marker >= 0) {
        CHECK(scene.building_of(static_cast<size_t>(roof)) != scene.building_of(static_cast<size_t>(marker)));
    }
    // A wall, with its extent along the face.
    const int south = scene.find_triangle({30.0, 5.0, -10.0});
    CHECK(south >= 0);
    if (south >= 0) {
        const auto &f = scene.frame_of(static_cast<size_t>(south));
        CHECK(f.kind == render::SurfaceFrame::Kind::kWall);
        CHECK(std::fabs(f.u_max - f.u_min - 40.0) < 1e-6);
        CHECK(f.z_top == -30.0 && f.z_bottom == 0.0);
    }
    CHECK(scene.find_triangle({200.0, 200.0, 0.0}) == -1);
    CHECK(scene.find_triangle({50.0, 0.0, -20.0}) == -1); // inside the tower, on nothing

    // Shadows fall away from the sun (it is to the south-west).
    CHECK(scene.in_shadow({80.0, 25.0, 0.0}, -1));   // north-east of the tower
    CHECK(!scene.in_shadow({20.0, -40.0, 0.0}, -1)); // south-west of it
    CHECK(!scene.in_shadow({150.0, 150.0, 0.0}, -1));
    CHECK(!scene.in_shadow({50.0, 0.0, -31.0}, -1)); // above everything
    if (roof >= 0) {
        CHECK(!scene.in_shadow({50.0, 0.0, -30.0}, roof));
    }
    const int north = scene.find_triangle({70.0, 0.0, -10.0});
    CHECK(north >= 0);
    if (north >= 0 && south >= 0) {
        CHECK(scene.in_shadow({70.0, 0.0, -10.0}, north));  // faces away from the sun
        CHECK(!scene.in_shadow({30.0, 5.0, -10.0}, south)); // faces it
    }
    // Shadowed ground is darker than the same ground in the sun.
    double lit[3], shade[3];
    scene.lit_colour(-1, {80.0, 25.0, 0.0}, 0.05, shade);
    double albedo[3];
    scene.albedo(-1, {80.0, 25.0, 0.0}, 0.05, albedo);
    for (int c = 0; c < 3; ++c) {
        CHECK(std::fabs(shade[c] - albedo[c] * render::kSkyLight[c]) < 1e-12);
    }
    scene.lit_colour(-1, {20.0, -40.0, 0.0}, 0.05, lit);
    scene.albedo(-1, {20.0, -40.0, 0.0}, 0.05, albedo);
    CHECK(lit[1] > albedo[1] * render::kSkyLight[1]);

    // A scene with nothing in it still has a textured, lit ground and no shadows.
    render::RasterScene empty;
    empty.build({});
    CHECK(empty.triangle_count() == 0 && empty.building_count() == 0);
    CHECK(!empty.in_shadow({0.0, 0.0, 0.0}, -1));
    CHECK(empty.find_triangle({0.0, 0.0, 0.0}) == -1);
}

void test_camera() {
    render::CameraConfig cfg;
    cfg.width = 320;
    cfg.height = 240;
    cfg.fov_deg = 73.7;
    const auto cam =
        render::Camera::with_attitude(cfg, {10.0, 20.0, -80.0}, render::camera_quat_from_angles(0, -90, 0));
    // Straight down, top of the image to the north, right to the east.
    CHECK(std::fabs(cam.forward()[2] - 1.0) < 1e-12);
    CHECK(std::fabs(cam.right()[1] - 1.0) < 1e-12);
    CHECK(std::fabs(cam.down()[0] + 1.0) < 1e-12);
    double px = 0.0, py = 0.0;
    CHECK(cam.project({10.0, 20.0, 0.0}, px, py));
    CHECK(std::fabs(px - 160.0) < 1e-9 && std::fabs(py - 120.0) < 1e-9);
    CHECK(!cam.project({10.0, 20.0, -100.0}, px, py)); // behind
    // ray() and project() are inverses.
    const Vec3 d = cam.ray(37, 201);
    CHECK(cam.project({10.0 + d[0] * 50.0, 20.0 + d[1] * 50.0, -80.0 + d[2] * 50.0}, px, py));
    CHECK(std::fabs(px - 37.5) < 1e-9 && std::fabs(py - 201.5) < 1e-9);
    // The rigid-mount constructor is the airframe attitude times the mount tilt.
    cfg.pitch_deg = -30.0;
    const render::Camera rigid(cfg, {0.0, 0.0, -10.0}, yaw_quat(90.0));
    CHECK(std::fabs(rigid.forward()[1] - std::cos(M_PI / 6.0)) < 1e-12);
    CHECK(std::fabs(rigid.forward()[2] - 0.5) < 1e-12);
}

void test_angles() {
    using namespace render;
    CHECK(std::fabs(heading_deg(yaw_quat(90.0)) - 90.0) < 1e-9);
    CHECK(std::fabs(heading_deg(yaw_quat(-90.0)) - 270.0) < 1e-9);
    CHECK(heading_deg({1.0, 0.0, 0.0, 0.0}) == 0.0);

    // Round trip away from the pole...
    const CameraAngles a = angles_from_camera_quat(camera_quat_from_angles(123.0, -60.0, 7.0));
    CHECK(std::fabs(a.yaw_deg - 123.0) < 1e-9 && std::fabs(a.pitch_deg + 60.0) < 1e-9 &&
          std::fabs(a.roll_deg - 7.0) < 1e-9);
    // ... and at it, where yaw is where the top of the image faces.
    const CameraAngles n = angles_from_camera_quat(camera_quat_from_angles(250.0, -90.0, 0.0));
    CHECK(std::fabs(n.yaw_deg - 250.0) < 1e-6 && n.pitch_deg == -90.0 && n.roll_deg == 0.0);
    // Near the pole, but not at it, the angles must still describe the same camera: rebuild the
    // attitude from them and compare its axes with the original's.
    for (const auto &[yaw, pitch, roll] : {std::array<double, 3>{0.0, -89.95, 90.0},
                                           {45.0, -89.999, -30.0},
                                           {250.0, -90.0, 0.0},
                                           {10.0, -60.0, 15.0},
                                           {300.0, 89.99, 45.0}}) {
        const Quat q0 = camera_quat_from_angles(yaw, pitch, roll);
        const CameraAngles back = angles_from_camera_quat(q0);
        const Quat q1 = camera_quat_from_angles(back.yaw_deg, back.pitch_deg, back.roll_deg);
        double worst = 0.0;
        for (const Vec3 &axis : {Vec3{1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 0, 1}}) {
            const Vec3 a0 = skysim::core::quat_rotate(q0, axis);
            const Vec3 a1 = skysim::core::quat_rotate(q1, axis);
            worst = std::max(
                {worst, std::fabs(a0[0] - a1[0]), std::fabs(a0[1] - a1[1]), std::fabs(a0[2] - a1[2])});
        }
        CHECK(worst < 1e-6);
    }
    const CameraAngles up = angles_from_camera_quat(camera_quat_from_angles(30.0, 90.0, 0.0));
    CHECK(std::fabs(up.yaw_deg - 30.0) < 1e-6 && up.pitch_deg == 90.0);

    // Stabilised: the airframe's roll and pitch do not reach the camera, its heading does.
    StillRequest r;
    r.pitch_deg = -60.0;
    r.yaw_deg = 30.0;
    CameraAngles got;
    const Quat q = still_camera_attitude(attitude(100.0, 12.0, -20.0), r, got);
    CHECK(std::fabs(got.yaw_deg - 130.0) < 1e-9 && got.pitch_deg == -60.0 && got.roll_deg == 0.0);
    const CameraAngles back = angles_from_camera_quat(q);
    CHECK(std::fabs(back.yaw_deg - 130.0) < 1e-9 && std::fabs(back.pitch_deg + 60.0) < 1e-9);
    r.yaw_deg = -200.0;
    still_camera_attitude(attitude(100.0, 0.0, 0.0), r, got);
    CHECK(std::fabs(got.yaw_deg - 260.0) < 1e-9);
    // Rigid: they do.
    r.roll_stab = false;
    r.yaw_deg = 0.0;
    still_camera_attitude(attitude(100.0, 0.0, -20.0), r, got);
    CHECK(std::fabs(got.roll_deg) > 5.0);
    still_camera_attitude(attitude(100.0, 0.0, 0.0), r, got);
    CHECK(std::fabs(got.yaw_deg - 100.0) < 1e-6 && std::fabs(got.pitch_deg + 60.0) < 1e-6 &&
          std::fabs(got.roll_deg) < 1e-6);

    const CameraConfig cfg = still_camera_config(r);
    CHECK(cfg.width == 1600 && cfg.height == 1200 && cfg.fov_deg == 73.7 && cfg.max_range_m == 600.0);
    CHECK(cfg.haze_start > 0.0);
}

// TASK-2548: the gimbal a SITL drives on its servo outputs, and the cameras that follow it.
void test_gimbal() {
    using namespace render;
    using skysim::vehicle::gimbal_from_servos;
    using skysim::vehicle::GimbalConfig;
    const GimbalConfig cfg; // the gateway's: pitch on 10 over -90..20, yaw on 11 over -180..180
    std::array<uint16_t, 32> pwm{};
    for (int k = 0; k < 4; ++k) {
        pwm[static_cast<size_t>(k)] = 1500; // motors
    }
    const double deg = M_PI / 180.0;
    // No pulse on output 10: no gimbal, whatever else the vehicle drives.
    CHECK(!gimbal_from_servos(cfg, pwm, 16).present);
    // Nor an output idling below the servo range, as some builds leave an output with no
    // function (ArduCopter 4.8 SITL sends 0 there).
    pwm[9] = 1000;
    CHECK(!gimbal_from_servos(cfg, pwm, 16).present);
    // Measured on ArduCopter 4.8 SITL: DO_MOUNT_CONTROL -60 puts output 10 at 1318 us, yaw 0 at 1500.
    pwm[9] = 1318;
    pwm[10] = 1500;
    auto g = gimbal_from_servos(cfg, pwm, 16);
    CHECK(g.present && std::fabs(g.pitch_rad + 60.0 * deg) < 0.1 * deg && std::fabs(g.yaw_rad) < 1e-9);
    pwm[9] = 1100;
    pwm[10] = 1900;
    g = gimbal_from_servos(cfg, pwm, 16);
    CHECK(g.present && std::fabs(g.pitch_rad + M_PI / 2.0) < 1e-12 && std::fabs(g.yaw_rad - M_PI) < 1e-12);
    // A pulse just past the end stops at the end of the range; one far past it is no mount's.
    pwm[9] = 1940;
    CHECK(std::fabs(gimbal_from_servos(cfg, pwm, 16).pitch_rad - 20.0 * deg) < 1e-12);
    pwm[9] = 2000;
    CHECK(!gimbal_from_servos(cfg, pwm, 16).present);
    // Outputs the frame does not carry, a disabled channel, and no yaw axis.
    CHECK(!gimbal_from_servos(cfg, pwm, 8).present);
    GimbalConfig off = cfg;
    off.pitch_channel = 0;
    CHECK(!gimbal_from_servos(off, pwm, 16).present);
    GimbalConfig no_yaw = cfg;
    no_yaw.yaw_channel = 0;
    pwm[9] = 1318;
    g = gimbal_from_servos(no_yaw, pwm, 16);
    CHECK(g.present && g.yaw_rad == 0.0);

    // A still asked to follow the gimbal is drawn at the gimbal's pitch and yaw, stabilised;
    // one not asked, or on a vehicle without a gimbal, as asked.
    render::StillPose pose;
    pose.quat_ned_frd = attitude(90.0, -12.0, 8.0);
    pose.gimbal = true;
    pose.gimbal_pitch_rad = -60.0 * deg;
    pose.gimbal_yaw_rad = 0.0;
    StillRequest asked;
    asked.pitch_deg = -90.0;
    asked.roll_stab = false;
    asked.follow_gimbal = true;
    bool followed = false;
    CameraAngles got;
    still_camera_attitude(pose.quat_ned_frd, render::gimbal_still_request(pose, asked, followed), got);
    CHECK(followed && std::fabs(got.pitch_deg + 60.0) < 1e-9 && std::fabs(got.yaw_deg - 90.0) < 1e-6 &&
          got.roll_deg == 0.0);
    asked.follow_gimbal = false;
    CHECK(render::gimbal_still_request(pose, asked, followed).pitch_deg == -90.0 && !followed);
    asked.follow_gimbal = true;
    pose.gimbal = false;
    CHECK(render::gimbal_still_request(pose, asked, followed).pitch_deg == -90.0 && !followed);

    // The pilot camera on the gimbal looks along the heading at the gimbal's pitch, the
    // airframe's lean taken out: heading 90 (east), -60 -> forward (0, cos 60, sin 60) in NED.
    render::CameraConfig cam;
    const auto on =
        render::Camera::on_gimbal(cam, {0.0, 0.0, -60.0}, attitude(90.0, -12.0, 8.0), 0.0, -60.0 * deg);
    CHECK(std::fabs(on.forward()[0]) < 1e-9 && std::fabs(on.forward()[1] - 0.5) < 1e-9 &&
          std::fabs(on.forward()[2] - std::sqrt(3.0) / 2.0) < 1e-9);
    CHECK(std::fabs(on.right()[2]) < 1e-9); // no roll
    // Rigid, as before a gimbal: the fixed mount pitch on the leaning airframe.
    const render::Camera rigid(cam, {0.0, 0.0, -60.0}, attitude(90.0, 0.0, 0.0));
    CHECK(std::fabs(rigid.forward()[2] - std::sin(15.0 * M_PI / 180.0)) < 1e-9);
}

void test_request_bounds() {
    render::StillRequest r;
    CHECK(r.invalid().empty());
    const auto bad = [](auto mutate) {
        render::StillRequest x;
        mutate(x);
        return !x.invalid().empty();
    };
    CHECK(bad([](render::StillRequest &x) { x.width = 63; }));
    CHECK(bad([](render::StillRequest &x) { x.height = 4097; }));
    CHECK(bad([](render::StillRequest &x) { x.hfov_deg = 9.9; }));
    CHECK(bad([](render::StillRequest &x) { x.hfov_deg = std::nan(""); }));
    CHECK(bad([](render::StillRequest &x) { x.pitch_deg = 0.1; }));
    CHECK(bad([](render::StillRequest &x) { x.yaw_deg = -361.0; }));
    CHECK(bad([](render::StillRequest &x) { x.range_m = 3001.0; }));
    CHECK(bad([](render::StillRequest &x) { x.quality = 0; }));
}

void test_nadir_centre(const render::RasterScene &scene, skysim::core::ThreadPool &pool) {
    // A vehicle 60 m over the marker: whatever the airframe is doing, a stabilised nadir
    // frame has the marker's roof at its centre — the four pixels round the optical axis.
    const render::StillRequest req = small_request();
    const Vec3 over{1.0, 101.0, -60.0};
    const auto centre_on_marker = [&](const Quat &vehicle, const render::StillRequest &r) {
        render::CameraAngles angles;
        const Quat cam = render::still_camera_attitude(vehicle, r, angles);
        std::vector<int32_t> surface;
        render::render_still(scene, over, cam, r, &pool, true, &surface);
        bool all = true;
        for (int y = r.height / 2 - 1; y <= r.height / 2; ++y) {
            for (int x = r.width / 2 - 1; x <= r.width / 2; ++x) {
                const int32_t id = surface[static_cast<size_t>(y) * r.width + x];
                all = all && id >= 0 &&
                      scene.frame_of(static_cast<size_t>(id)).kind == render::SurfaceFrame::Kind::kRoof &&
                      scene.building_of(static_cast<size_t>(id)) ==
                          scene.building_of(static_cast<size_t>(scene.find_triangle({1.0, 101.0, -3.0})));
            }
        }
        return all;
    };
    CHECK(centre_on_marker({1.0, 0.0, 0.0, 0.0}, req));
    CHECK(centre_on_marker(attitude(37.0, 0.0, 0.0), req));
    CHECK(centre_on_marker(attitude(37.0, 15.0, -12.0), req)); // tilted airframe, gimbal holds
    render::StillRequest rigid = req;
    rigid.roll_stab = false;
    CHECK(centre_on_marker(attitude(37.0, 0.0, 0.0), rigid));
    CHECK(!centre_on_marker(attitude(37.0, 15.0, -12.0), rigid)); // rigid: the tilt moves it

    // Heading turns the picture: the tower (north-west of the marker) sits up and to the
    // left at heading 0, and on the other side of the frame at heading 180.
    const Vec3 between{40.0, 60.0, -150.0};
    render::StillRequest wide = req;
    std::vector<int32_t> surface;
    render::CameraAngles angles;
    render::render_still(scene, between, render::still_camera_attitude({1.0, 0.0, 0.0, 0.0}, wide, angles),
                         wide, &pool, true, &surface);
    const int32_t left = surface[static_cast<size_t>(wide.height / 2) * wide.width + 30];
    CHECK(left >= 0);
    render::render_still(scene, between, render::still_camera_attitude(yaw_quat(180.0), wide, angles), wide,
                         &pool, true, &surface);
    CHECK(surface[static_cast<size_t>(wide.height / 2) * wide.width + 30] == render::kSurfaceGround);
    CHECK(surface[static_cast<size_t>(wide.height / 2) * wide.width + wide.width - 31] >= 0);
}

void test_determinism(const render::RasterScene &scene, const render::RasterScene &reseeded,
                      skysim::core::ThreadPool &pool) {
    const render::StillRequest req = small_request(400, 300);
    const Vec3 pos{20.0, 10.0, -90.0};
    const Quat cam = render::camera_quat_from_angles(30.0, -60.0, 0.0);
    const render::Image a = render::render_still(scene, pos, cam, req, nullptr);
    const render::Image b = render::render_still(scene, pos, cam, req, &pool, true);
    const render::Image c = render::render_still(scene, pos, cam, req, &pool, false);
    CHECK(a.rgb == b.rgb); // threads cannot change a pixel: each band is one thread's
    CHECK(a.rgb == c.rgb);
    const auto ja = render::encode_jpeg(a, req.quality);
    const auto jb = render::encode_jpeg(b, req.quality);
    CHECK(!ja.empty() && ja == jb);
    // Another seed is another world.
    const render::Image d = render::render_still(reseeded, pos, cam, req, &pool);
    size_t differ = 0;
    for (size_t i = 0; i < a.rgb.size(); ++i) {
        differ += a.rgb[i] != d.rgb[i] ? 1 : 0;
    }
    CHECK(differ > a.rgb.size() / 4);
}

// The same world point has the same colour in two overlapping frames.
//
// Two nadir frames at the same height, the second moved by a whole number of pixels'
// worth of ground (160 px east, 100 px north at ground level; 256 and 160 px at the
// tower's roof, 50 m below the camera rather than 80). Every pixel centre then sees
// exactly the world point a pixel of the other frame sees, so the comparison needs no
// interpolation: a difference is the renderer's, not the test's.
void test_consistency(const render::RasterScene &scene, skysim::core::ThreadPool &pool) {
    render::StillRequest req = small_request(640, 480);
    const double alt = 80.0;
    const double gsd = 2.0 * alt * std::tan(req.hfov_deg * M_PI / 360.0) / req.width;
    const Vec3 a_pos{40.0, 0.0, -alt};
    const Vec3 b_pos{40.0 + 100.0 * gsd, 160.0 * gsd, -alt};
    const Quat nadir = render::camera_quat_from_angles(0.0, -90.0, 0.0);
    std::vector<int32_t> sa, sb;
    const render::Image a = render::render_still(scene, a_pos, nadir, req, &pool, true, &sa);
    const render::Image b = render::render_still(scene, b_pos, nadir, req, &pool, true, &sb);

    const auto compare = [&](int dx, int dy, bool roof, double &mean, int &worst, size_t &samples) {
        double sum = 0.0;
        worst = 0;
        samples = 0;
        for (int y = 0; y < req.height; ++y) {
            for (int x = 0; x < req.width; ++x) {
                const int xb = x - dx;
                const int yb = y + dy;
                if (xb < 0 || xb >= req.width || yb < 0 || yb >= req.height) {
                    continue;
                }
                const int32_t ia = sa[static_cast<size_t>(y) * req.width + x];
                const int32_t ib = sb[static_cast<size_t>(yb) * req.width + xb];
                const bool ok =
                    roof ? (ia >= 0 && ia == ib &&
                            scene.frame_of(static_cast<size_t>(ia)).kind == render::SurfaceFrame::Kind::kRoof)
                         : (ia == render::kSurfaceGround && ib == render::kSurfaceGround);
                if (!ok) {
                    continue;
                }
                for (int c = 0; c < 3; ++c) {
                    const int d = std::abs(byte_at(a, x, y, c) - byte_at(b, xb, yb, c));
                    sum += d;
                    worst = std::max(worst, d);
                }
                ++samples;
            }
        }
        mean = samples > 0 ? sum / (3.0 * samples) : 1e9;
    };

    double mean = 0.0;
    int worst = 0;
    size_t samples = 0;
    compare(160, 100, false, mean, worst, samples);
    std::printf("test_still: ground: %zu shared points, mean |diff| %.3f, worst %d\n", samples, mean, worst);
    CHECK(samples > 50000);
    CHECK(mean < 0.01);
    CHECK(worst <= 1); // the same bytes, give or take a rounding of the last bit
    compare(256, 160, true, mean, worst, samples);
    std::printf("test_still: roof:   %zu shared points, mean |diff| %.3f, worst %d\n", samples, mean, worst);
    CHECK(samples > 20000);
    CHECK(mean < 0.01);
    CHECK(worst <= 1);
    // And the texture is not flat: three pixels off, the same comparison is far apart.
    compare(163, 100, false, mean, worst, samples);
    std::printf("test_still: ground, misregistered by 3 px: mean |diff| %.3f\n", mean);
    CHECK(mean > 5.0);

    // Through the JPEG codec too, at the default quality.
    const auto ja = render::encode_jpeg(a, req.quality);
    const auto jb = render::encode_jpeg(b, req.quality);
    int w = 0, h = 0, n = 0;
    unsigned char *da = stbi_load_from_memory(ja.data(), static_cast<int>(ja.size()), &w, &h, &n, 3);
    unsigned char *db = stbi_load_from_memory(jb.data(), static_cast<int>(jb.size()), &w, &h, &n, 3);
    CHECK(da != nullptr && db != nullptr && w == req.width && h == req.height);
    if (da != nullptr && db != nullptr) {
        double sum = 0.0;
        size_t count = 0;
        for (int y = 8; y < req.height - 8; ++y) {
            for (int x = 168; x < req.width - 8; ++x) {
                const int xb = x - 160;
                const int yb = y + 100;
                if (yb >= req.height - 8 ||
                    sa[static_cast<size_t>(y) * req.width + x] != render::kSurfaceGround ||
                    sb[static_cast<size_t>(yb) * req.width + xb] != render::kSurfaceGround) {
                    continue;
                }
                for (int c = 0; c < 3; ++c) {
                    sum += std::abs(da[(static_cast<size_t>(y) * w + x) * 3 + c] -
                                    db[(static_cast<size_t>(yb) * w + xb) * 3 + c]);
                }
                ++count;
            }
        }
        const double jmean = count > 0 ? sum / (3.0 * count) : 1e9;
        std::printf("test_still: ground through JPEG q%d: mean |diff| %.3f\n", req.quality, jmean);
        CHECK(jmean < 3.0);
    }
    stbi_image_free(da);
    stbi_image_free(db);
}

void test_ground_without_tiles(skysim::core::ThreadPool &pool) {
    // No tile loaded: the ground is still there, textured, to the horizon — never sky.
    render::RasterScene empty;
    empty.build({}, 1);
    for (const double pitch : {-90.0, -60.0, -35.0}) { // the top edge still below the horizon
        render::StillRequest req = small_request();
        req.pitch_deg = pitch;
        std::vector<int32_t> surface;
        const render::Image img =
            render::render_still(empty, {0.0, 0.0, -100.0}, render::camera_quat_from_angles(0.0, pitch, 0.0),
                                 req, &pool, true, &surface);
        CHECK(std::count(surface.begin(), surface.end(), render::kSurfaceGround) ==
              static_cast<long>(surface.size()));
        int lo = 255, hi = 0;
        for (size_t i = 0; i < img.rgb.size(); i += 3) {
            lo = std::min<int>(lo, img.rgb[i + 1]);
            hi = std::max<int>(hi, img.rgb[i + 1]);
        }
        CHECK(hi - lo > 40);
    }
    // Level, the top half is sky and the bottom half ground.
    render::StillRequest level = small_request();
    level.pitch_deg = 0.0;
    std::vector<int32_t> surface;
    render::render_still(empty, {0.0, 0.0, -100.0}, render::camera_quat_from_angles(0.0, 0.0, 0.0), level,
                         &pool, true, &surface);
    CHECK(surface[5 * level.width + 10] == render::kSurfaceSky);
    CHECK(surface[static_cast<size_t>(level.height - 5) * level.width + 10] == render::kSurfaceGround);
}

// Faces built straight from triangles (NED), for shapes the cooked fixtures do not have.
skysim::core::World::Triangle tri(Vec3 a, Vec3 b, Vec3 c) {
    skysim::core::World::Triangle t;
    t.a = a;
    t.b = b;
    t.c = c;
    return t;
}

void test_geometry_edges(skysim::core::ThreadPool &pool) {
    // A long, narrow roof (100 x 10 m, 20 m up) split along its diagonal. (99, 9.8) lies in the
    // triangle without the long east edge, yet that edge — 0.2 m away — is the nearest one of
    // the roof's outline, and the parapet must follow it.
    render::RasterScene roof;
    const double z = -20.0;
    roof.build({tri({0, 0, z}, {100, 10, z}, {100, 0, z}), tri({0, 0, z}, {0, 10, z}, {100, 10, z}),
                tri({0, 0, 0}, {100, 0, 0}, {100, 0, z}), tri({0, 0, 0}, {100, 0, z}, {0, 0, z})});
    const int t = roof.find_triangle({99.0, 9.8, z});
    CHECK(t >= 0);
    if (t >= 0) {
        CHECK(roof.frame_of(static_cast<size_t>(t)).kind == render::SurfaceFrame::Kind::kRoof);
        CHECK(std::fabs(roof.edge_distance(static_cast<size_t>(t), {99.0, 9.8, z}) - 0.2) < 1e-9);
        CHECK(std::fabs(roof.edge_distance(static_cast<size_t>(t), {50.0, 5.0, z}) - 5.0) < 1e-9);
    }
    // A wall measures through its own triangle.
    const int w = roof.find_triangle({50.0, 0.0, -5.0});
    CHECK(w >= 0 && roof.frame_of(static_cast<size_t>(w)).kind == render::SurfaceFrame::Kind::kWall);

    // Geometry below the ground plane is underground: a nadir camera sees the ground over it,
    // never the buried face, while a face standing on the ground still shows.
    render::RasterScene buried;
    buried.build({tri({-50, -50, 10}, {50, -50, 10}, {50, 50, 10}),
                  tri({-50, -50, 10}, {50, 50, 10}, {-50, 50, 10}),
                  tri({-5, -5, -1}, {5, -5, -1}, {5, 5, -1}), tri({-5, -5, -1}, {5, 5, -1}, {-5, 5, -1})});
    std::vector<int32_t> surface;
    render::StillRequest req = small_request(128, 96);
    render::render_still(buried, {20.0, 20.0, -60.0}, render::camera_quat_from_angles(0, -90, 0), req, &pool,
                         true, &surface);
    CHECK(surface[static_cast<size_t>(req.height / 2) * req.width + req.width / 2] == render::kSurfaceGround);
    render::render_still(buried, {0.0, 0.0, -60.0}, render::camera_quat_from_angles(0, -90, 0), req, &pool,
                         true, &surface);
    CHECK(surface[static_cast<size_t>(req.height / 2) * req.width + req.width / 2] >= 0); // the 1 m slab

    // A face lying exactly on the ground ties with it, and the tie goes to the face.
    render::RasterScene flat;
    flat.build({tri({-5, -5, 0}, {5, -5, 0}, {5, 5, 0}), tri({-5, -5, 0}, {5, 5, 0}, {-5, 5, 0})});
    render::render_still(flat, {0.0, 0.0, -60.0}, render::camera_quat_from_angles(0, -90, 0), req, &pool,
                         true, &surface);
    CHECK(surface[static_cast<size_t>(req.height / 2) * req.width + req.width / 2] >= 0);
    CHECK(surface[0] == render::kSurfaceGround); // and beyond it, ground
    // ... and looks like the ground it lies on.
    double on_face[3], ground[3];
    flat.albedo(0, {1.0, -2.0, 0.0}, 0.05, on_face);
    flat.albedo(-1, {1.0, -2.0, 0.0}, 0.05, ground);
    CHECK(on_face[0] == ground[0] && on_face[1] == ground[1] && on_face[2] == ground[2]);
}

void test_service(const std::shared_ptr<const render::RasterScene> &scene) {
    CHECK(render::lower_thread_priority()); // only ever lowers, so always allowed

    render::StillService::Config cfg;
    cfg.concurrency = 2;
    cfg.threads = 2;
    render::StillService service(scene, cfg);
    CHECK(service.concurrency() == 2);
    CHECK(service.scene().triangle_count() == scene->triangle_count());

    render::StillPose pose;
    pose.position_ned = {1.0, 101.0, -60.0};
    const render::StillRequest req = small_request();
    const render::StillFrame ok = service.render(pose, req);
    CHECK(ok.status == render::StillFrame::Status::kOk);
    CHECK(ok.jpeg.size() > 1000 && ok.jpeg[0] == 0xFF && ok.jpeg[1] == 0xD8);
    CHECK(ok.camera.pitch_deg == -90.0);
    CHECK(service.in_flight() == 0);
    // Deterministic through the service as well.
    CHECK(service.render(pose, req).jpeg == ok.jpeg);

    // Every slot taken: busy at once, not queued.
    {
        auto first = service.try_acquire();
        auto second = service.try_acquire();
        CHECK(first.has_value() && second.has_value());
        CHECK(!service.try_acquire().has_value());
        CHECK(service.in_flight() == 2);
        const render::StillFrame busy = service.render(pose, req);
        CHECK(busy.status == render::StillFrame::Status::kBusy);
        CHECK(busy.jpeg.empty() && busy.error.find("busy") != std::string::npos);
        // A held lease renders on its own slot.
        const render::StillFrame mine = service.render(*first, pose, req);
        CHECK(mine.status == render::StillFrame::Status::kOk && mine.jpeg == ok.jpeg);
        render::StillService::Lease moved(std::move(*second));
        CHECK(moved.slot() >= 0);
    }
    CHECK(service.in_flight() == 0);

    render::StillRequest bad = req;
    bad.width = 5000;
    const render::StillFrame invalid = service.render(pose, bad);
    CHECK(invalid.status == render::StillFrame::Status::kInvalid && !invalid.error.empty());

    // Concurrency 0 (or no scene) is a still camera that is off.
    cfg.concurrency = 0;
    render::StillService off(scene, cfg);
    CHECK(off.render(pose, req).status == render::StillFrame::Status::kDisabled);
    cfg.concurrency = 2;
    render::StillService none(nullptr, cfg);
    CHECK(none.concurrency() == 0);
    CHECK(none.render(pose, req).status == render::StillFrame::Status::kDisabled);
}

// The HTTP routes, with a real still service behind them and a canned snapshot in front.
void test_http(const std::shared_ptr<const render::RasterScene> &scene) {
    using namespace skysim::api;
    render::StillService::Config cfg;
    cfg.concurrency = 1;
    cfg.threads = 2;
    render::StillService service(scene, cfg);

    VehicleInfo v;
    v.id = 7;
    v.instance = 31;
    v.pos_ned[0] = 1.0;
    v.pos_ned[1] = 101.0;
    v.pos_ned[2] = -60.0;
    const Quat q = attitude(45.0, 8.0, -5.0);
    for (int k = 0; k < 4; ++k) {
        v.quat_ned_frd[k] = q[static_cast<size_t>(k)];
    }
    v.sim_time_s = 12.3456789;
    // Its gimbal at the -60 degrees a 3D mapping survey points it to, looking 20 degrees right.
    v.gimbal = true;
    v.gimbal_pitch_rad = -60.0 * M_PI / 180.0;
    v.gimbal_yaw_rad = 20.0 * M_PI / 180.0;

    CommandQueue queue;
    ControlServer::Snapshots snaps;
    snaps.vehicles = [v] { return std::vector<VehicleInfo>{v}; };
    snaps.metrics = [] { return MetricsInfo{}; };
    std::atomic<int> calls{0};
    snaps.still = [&service, &calls](const VehicleInfo &info, const render::StillRequest &r) {
        ++calls;
        render::StillPose pose;
        pose.position_ned = {info.pos_ned[0], info.pos_ned[1], info.pos_ned[2]};
        pose.quat_ned_frd = {info.quat_ned_frd[0], info.quat_ned_frd[1], info.quat_ned_frd[2],
                             info.quat_ned_frd[3]};
        pose.sim_time_s = info.sim_time_s;
        pose.gimbal = info.gimbal;
        pose.gimbal_pitch_rad = info.gimbal_pitch_rad;
        pose.gimbal_yaw_rad = info.gimbal_yaw_rad;
        return service.render(pose, r);
    };
    ControlServer::RadioConfig radio;
    radio.home = skysim::radio::GeoAnchor{42.14, 24.76, 160.0};
    {
        ControlServer server("127.0.0.1", kPort, queue, snaps, radio);
        httplib::Client client("127.0.0.1", kPort);
        client.set_read_timeout(30, 0);

        auto first = client.Get("/instances/31/still.jpg?w=320&h=240&yaw=10");
        CHECK(first && first->status == 200);
        if (first && first->status == 200) {
            CHECK(first->get_header_value("Content-Type") == "image/jpeg");
            CHECK(first->body.size() > 1000);
            // The exact pose it was rendered from.
            CHECK(first->get_header_value("X-Skysim-Time-Usec") == "12345679");
            const auto geo = skysim::core::geo_from_ned({42.14, 24.76, 160.0}, 1.0, 101.0, -60.0);
            CHECK(std::fabs(std::stod(first->get_header_value("X-Skysim-Lat")) - geo.lat_deg) < 1e-9);
            CHECK(std::fabs(std::stod(first->get_header_value("X-Skysim-Lon")) - geo.lon_deg) < 1e-9);
            CHECK(first->get_header_value("X-Skysim-Alt-Amsl") == "220.000");
            CHECK(first->get_header_value("X-Skysim-Alt-Rel") == "60.000");
            CHECK(std::fabs(std::stod(first->get_header_value("X-Skysim-Yaw")) - 55.0) < 1e-3);
            CHECK(first->get_header_value("X-Skysim-Pitch") == "-90.0000");
            CHECK(first->get_header_value("X-Skysim-Roll") == "0.0000");
            CHECK(first->get_header_value("X-Skysim-Hfov") == "73.7000");
            CHECK(first->get_header_value("X-Skysim-Ned") == "1.000,101.000,-60.000");
            CHECK(first->get_header_value("X-Skysim-Vehicle") == "7");
            int w = 0, h = 0, n = 0;
            unsigned char *px =
                stbi_load_from_memory(reinterpret_cast<const unsigned char *>(first->body.data()),
                                      static_cast<int>(first->body.size()), &w, &h, &n, 3);
            CHECK(px != nullptr && w == 320 && h == 240);
            stbi_image_free(px);
        }
        // Same pose, same request: the same bytes, by instance or by id.
        auto again = client.Get("/vehicles/7/still.jpg?w=320&h=240&yaw=10");
        CHECK(again && again->status == 200 && first && again->body == first->body);
        // Defaults: 1600 x 1200.
        auto full = client.Get("/instances/31/still.jpg");
        CHECK(full && full->status == 200);
        if (full && full->status == 200) {
            int w = 0, h = 0, n = 0;
            CHECK(stbi_info_from_memory(reinterpret_cast<const unsigned char *>(full->body.data()),
                                        static_cast<int>(full->body.size()), &w, &h, &n) == 1);
            CHECK(w == 1600 && h == 1200);
        }
        // Rigid mount: the airframe's tilt shows in the headers.
        auto rigid = client.Get("/instances/31/still.jpg?w=64&h=64&roll_stab=0&pitch=-45");
        CHECK(rigid && rigid->status == 200);
        if (rigid && rigid->status == 200) {
            CHECK(rigid->get_header_value("X-Skysim-Roll") != "0.0000");
        }
        auto stab = client.Get("/instances/31/still.jpg?w=64&h=64&roll_stab=true&pitch=-45");
        CHECK(stab && stab->status == 200 && stab->get_header_value("X-Skysim-Pitch") == "-45.0000");
        CHECK(stab && stab->get_header_value("X-Skysim-Gimbal") == "0");
        // TASK-2548: gimbal=1 draws where the vehicle's gimbal points, at its own heading (45)
        // plus the gimbal's yaw, whatever fixed pitch the caller also sent.
        auto gimbal = client.Get("/instances/31/still.jpg?w=64&h=64&pitch=-90&gimbal=1");
        CHECK(gimbal && gimbal->status == 200);
        if (gimbal && gimbal->status == 200) {
            CHECK(gimbal->get_header_value("X-Skysim-Gimbal") == "1");
            CHECK(gimbal->get_header_value("X-Skysim-Pitch") == "-60.0000");
            CHECK(std::fabs(std::stod(gimbal->get_header_value("X-Skysim-Yaw")) - 65.0) < 1e-3);
            CHECK(gimbal->get_header_value("X-Skysim-Roll") == "0.0000");
        }
        auto bad_gimbal = client.Get("/instances/31/still.jpg?w=64&h=64&gimbal=yes");
        CHECK(bad_gimbal && bad_gimbal->status == 400);

        // Every input bounded, every error JSON.
        const char *bad[] = {"w=63",    "w=4097",      "w=12.5",        "w=",         "h=4097",
                             "hfov=9",  "hfov=121",    "hfov=x",        "pitch=1",    "pitch=-91",
                             "yaw=361", "yaw=-361",    "range=49",      "range=3001", "q=0",
                             "q=101",   "roll_stab=2", "roll_stab=yes", "pitch=nan",  "hfov=inf"};
        const int before = calls.load();
        for (const char *query : bad) {
            auto r = client.Get((std::string("/instances/31/still.jpg?") + query).c_str());
            CHECK(r && r->status == 400);
            if (r) {
                CHECK(r->get_header_value("Content-Type") == "application/json");
                CHECK(r->body.find("\"error\":") != std::string::npos);
            }
        }
        CHECK(calls.load() == before); // rejected before anything was rendered

        auto missing = client.Get("/instances/32/still.jpg");
        CHECK(missing && missing->status == 404 &&
              missing->body.find("no such instance") != std::string::npos);
        auto missing_id = client.Get("/vehicles/99999999999999999999/still.jpg");
        CHECK(missing_id && missing_id->status == 404 &&
              missing_id->body.find("no such vehicle") != std::string::npos);
        // 2^32 + 7 is not vehicle 7: an id too big for the type does not wrap onto another one.
        auto wrapped = client.Get("/vehicles/4294967303/still.jpg?w=64&h=64");
        CHECK(wrapped && wrapped->status == 404);
        auto zeros = client.Get("/instances/0031/still.jpg?w=64&h=64");
        CHECK(zeros && zeros->status == 200 && zeros->get_header_value("X-Skysim-Instance") == "31");

        // Busy: the one slot is taken, so the route answers 503 with Retry-After at once.
        {
            auto lease = service.try_acquire();
            CHECK(lease.has_value());
            const auto t0 = std::chrono::steady_clock::now();
            auto busy = client.Get("/instances/31/still.jpg?w=64&h=64");
            const double waited =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            CHECK(busy && busy->status == 503);
            if (busy) {
                CHECK(busy->get_header_value("Retry-After") == "1");
                CHECK(busy->body.find("busy") != std::string::npos);
            }
            CHECK(waited < 2.0);
        }
        auto after = client.Get("/instances/31/still.jpg?w=64&h=64");
        CHECK(after && after->status == 200);

        // Concurrent requests past the limit: never more than one rendering, the rest 503.
        std::atomic<int> ok{0}, busy{0}, other{0};
        std::vector<std::thread> clients;
        for (int i = 0; i < 6; ++i) {
            clients.emplace_back([&] {
                httplib::Client c("127.0.0.1", kPort);
                c.set_read_timeout(30, 0);
                auto r = c.Get("/instances/31/still.jpg?w=1600&h=1200");
                if (r && r->status == 200) {
                    ++ok;
                } else if (r && r->status == 503) {
                    ++busy;
                } else {
                    ++other;
                }
            });
        }
        for (auto &t : clients) {
            t.join();
        }
        std::printf("test_still: 6 concurrent full-size stills at concurrency 1: %d ok, %d busy\n", ok.load(),
                    busy.load());
        CHECK(ok.load() >= 1 && ok.load() + busy.load() == 6 && other.load() == 0);
    }

    // Without a home there is nowhere to put the frame on the earth: no lat/lon, the rest stays.
    {
        ControlServer server("127.0.0.1", kPort, queue, snaps, ControlServer::RadioConfig{});
        httplib::Client client("127.0.0.1", kPort);
        client.set_read_timeout(30, 0);
        auto r = client.Get("/instances/31/still.jpg?w=64&h=64");
        CHECK(r && r->status == 200 && !r->has_header("X-Skysim-Lat") && r->has_header("X-Skysim-Alt-Rel"));
        CHECK(r && r->get_header_value("X-Skysim-Warning").find("no geo anchor") != std::string::npos);
    }

    // A render that failed is the server's fault: 500, with the reason, as JSON.
    {
        auto failing = snaps;
        failing.still = [](const VehicleInfo &, const render::StillRequest &) {
            render::StillFrame f;
            f.status = render::StillFrame::Status::kFailed;
            f.error = "render failed: std::bad_alloc";
            return f;
        };
        ControlServer server("127.0.0.1", kPort, queue, failing, radio);
        httplib::Client client("127.0.0.1", kPort);
        auto r = client.Get("/instances/31/still.jpg");
        CHECK(r && r->status == 500 && r->body.find("bad_alloc") != std::string::npos);
    }

    // Still camera off: the routes say so.
    {
        snaps.still = nullptr;
        ControlServer server("127.0.0.1", kPort, queue, snaps, radio);
        httplib::Client client("127.0.0.1", kPort);
        auto r = client.Get("/instances/31/still.jpg");
        CHECK(r && r->status == 404 && r->body.find("disabled") != std::string::npos);
    }
    {
        render::StillService off(scene, render::StillService::Config{0, 1, false});
        snaps.still = [&off](const VehicleInfo &, const render::StillRequest &r) {
            return off.render(render::StillPose{}, r);
        };
        ControlServer server("127.0.0.1", kPort, queue, snaps, radio);
        httplib::Client client("127.0.0.1", kPort);
        auto r = client.Get("/vehicles/7/still.jpg");
        CHECK(r && r->status == 404 && r->body.find("disabled") != std::string::npos);
    }
}

// The pilot camera draws the same scene the stills do, with the same shading.
void test_pilot_camera(const std::shared_ptr<const render::RasterScene> &scene) {
    render::RenderService::Config cfg;
    cfg.camera.width = 128;
    cfg.camera.height = 72;
    cfg.scene = scene;
    cfg.scene_tiles = 2;
    cfg.fps = 60.0;
    cfg.threads = 2;
    render::RenderService service(cfg);
    CHECK(service.tiles_loaded() == 2);
    CHECK(service.triangle_count() == scene->triangle_count());
    render::RenderService::Pose pose;
    pose.vehicle_id = 3;
    pose.position_ned = {-60.0, 0.0, -20.0};
    service.publish_poses({pose});
    std::vector<uint8_t> frame;
    for (int i = 0; i < 200 && frame.empty(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        frame = service.frame(3).jpeg;
    }
    CHECK(!frame.empty());

    // The reference ray caster and the rasteriser agree on what is where, and — given the
    // scene for appearance — on its colour.
    render::CameraConfig cam;
    cam.width = 160;
    cam.height = 90;
    cam.pitch_deg = -20.0;
    const Vec3 pos{-40.0, -10.0, -25.0};
    const Quat level{1.0, 0.0, 0.0, 0.0};
    skysim::core::WorldConfig wcfg;
    wcfg.dt_s = 1.0 / 800.0;
    skysim::core::World world(wcfg);
    world.add_ground_plane();
    world.load_tiles(std::filesystem::temp_directory_path() / "skysim_still_tiles");
    std::vector<int32_t> ray_surface, raster_surface;
    const render::Image ray =
        render::Renderer(cam, scene.get()).render(world, 0, pos, level, nullptr, &ray_surface);
    const render::Image ras =
        render::Rasterizer(cam).render(*scene, render::Camera(cam, pos, level), nullptr, &raster_surface);
    size_t agree = 0, close = 0;
    for (size_t i = 0; i < ray_surface.size(); ++i) {
        const bool same = (ray_surface[i] < 0 || raster_surface[i] < 0)
                              ? ray_surface[i] == raster_surface[i]
                              : scene->building_of(ray_surface[i]) == scene->building_of(raster_surface[i]);
        agree += same ? 1 : 0;
        int d = 0;
        for (int c = 0; c < 3; ++c) {
            d = std::max(d, std::abs(ray.rgb[i * 3 + c] - ras.rgb[i * 3 + c]));
        }
        close += (same && d <= 24) ? 1 : 0;
    }
    std::printf(
        "test_still: ray caster vs rasteriser: %zu/%zu pixels on the same surface, %zu close in colour\n",
        agree, ray_surface.size(), close);
    CHECK(agree > ray_surface.size() * 97 / 100);
    CHECK(close > ray_surface.size() * 85 / 100);
    // Without an appearance scene the ray caster still draws: textured ground, grey buildings.
    const render::Image plain = render::Renderer(cam).render(world, 0, pos, level, nullptr);
    CHECK(plain.rgb.size() == ray.rgb.size());
}

void test_thread_pool_modes() {
    std::atomic<int> started{0};
    skysim::core::ThreadPool pool(3, [&started] { ++started; });
    std::vector<int> hits(10, 0);
    std::thread::id caller = std::this_thread::get_id();
    std::atomic<bool> caller_worked{false};
    pool.parallel_for(
        10,
        [&](size_t begin, size_t end) {
            if (std::this_thread::get_id() == caller) {
                caller_worked = true;
            }
            for (size_t i = begin; i < end; ++i) {
                ++hits[i];
            }
        },
        false);
    CHECK(!caller_worked.load()); // caller_runs = false: the caller only waited

    // A chunk that throws, on a worker or on the caller, reaches the caller as an exception
    // once every chunk has finished — and the pool keeps working afterwards.
    for (const bool caller_runs : {false, true}) {
        bool caught = false;
        std::atomic<int> finished{0};
        try {
            pool.parallel_for(
                4,
                [&](size_t begin, size_t) {
                    if (begin == 1 || (caller_runs && begin == 0)) {
                        throw std::runtime_error("chunk failed");
                    }
                    ++finished;
                },
                caller_runs);
        } catch (const std::runtime_error &e) {
            caught = std::string(e.what()) == "chunk failed";
        }
        CHECK(caught);
        CHECK(finished.load() >= 2);
    }
    std::atomic<int> after{0};
    pool.parallel_for(8, [&](size_t begin, size_t end) { after += static_cast<int>(end - begin); });
    CHECK(after.load() == 8);
    CHECK(std::all_of(hits.begin(), hits.end(), [](int h) { return h == 1; }));
    for (int i = 0; i < 100 && started.load() < 3; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    CHECK(started.load() == 3);
}

} // namespace

int main() {
    test_primitives();
    test_texture();
    test_geo();
    test_camera();
    test_angles();
    test_gimbal();
    test_request_bounds();
    test_thread_pool_modes();

    const auto dir = make_tiles();
    size_t tiles = 0;
    const auto scene = render::load_scene(dir.string(), 1, &tiles);
    CHECK(tiles == 2);
    const auto reseeded = render::load_scene(dir.string(), 2);
    skysim::core::ThreadPool pool(3);

    test_scene(*scene);
    test_nadir_centre(*scene, pool);
    test_determinism(*scene, *reseeded, pool);
    test_consistency(*scene, pool);
    test_ground_without_tiles(pool);
    test_geometry_edges(pool);
    test_service(scene);
    test_http(scene);
    test_pilot_camera(scene);

    std::filesystem::remove_all(dir);
    if (g_failures == 0) {
        std::printf("test_still: OK\n");
    }
    return g_failures == 0 ? 0 : 1;
}
