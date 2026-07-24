// Demo-map collision acceptance (ctest side): cook an OBJ building through the real
// cooker path, load it as a world tile, then (a) rest a body on the roof, (b) fly into a
// wall at 25 m/s and verify no tunneling (M5 acceptance: stable at <= 25 m/s), (c) raycast.
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "core/world.h"
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

// One 40x40 m, 30 m tall building centered at ENU (0, 50): NED north in [30, 70],
// east in [-20, 20], roof at NED z = -30. Written in the exact pretile.py box format.
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

} // namespace

int main() {
    const auto dir = std::filesystem::temp_directory_path() / "skysim_test_tiles";
    std::filesystem::create_directories(dir);
    const auto obj = dir / "building.obj";
    write_test_obj(obj);

    skysim::terrain::CookResult res;
    std::string err;
    if (!skysim::terrain::cook_obj_tile(obj, dir / "building.jshape", &res, &err)) {
        std::printf("FAIL cook: %s\n", err.c_str());
        return 1;
    }
    CHECK(res.triangles == 12);
    // ENU building (x:-20..20, y:30..70, z:0..30) => NED north 30..70, east -20..20, z -30..0.
    CHECK(std::abs(res.aabb_min_ned[0] - 30.0) < 0.01 && std::abs(res.aabb_max_ned[0] - 70.0) < 0.01);
    CHECK(std::abs(res.aabb_min_ned[1] + 20.0) < 0.01 && std::abs(res.aabb_max_ned[1] - 20.0) < 0.01);
    CHECK(std::abs(res.aabb_min_ned[2] + 30.0) < 0.01 && std::abs(res.aabb_max_ned[2] - 0.0) < 0.01);

    skysim::core::World w(cfg());
    w.add_ground_plane();
    CHECK(w.load_tiles(dir) == 1);
    CHECK(w.load_tiles("/nonexistent/skysim/tiles") == 0); // must not throw/abort

    // --- (a) Drop onto the roof from 5 m above: must come to rest ON the roof (z=-30). ---
    {
        skysim::core::VehicleBodyParams p;
        p.start_pos_ned = {50.0, 0.0, -35.0}; // above building center
        const uint32_t id = w.add_vehicle(p);
        for (int i = 0; i < 4000; ++i) { // 5 s
            w.step();
        }
        const auto s = w.get_state(id);
        CHECK(std::abs(s.pos_ned[2] + 30.0 + p.half_extents_frd[2]) < 0.05); // resting on roof
        CHECK(std::abs(s.vel_ned[2]) < 0.02);
        CHECK(std::abs(s.accel_body_frd[2] + 9.81) < 0.4); // accelerometer sees -g again
        w.remove_vehicle(id);
    }

    // --- (b) 25 m/s straight at the south wall (north face at NED north=30): no tunnel. ---
    {
        skysim::core::VehicleBodyParams p;
        p.start_pos_ned = {10.0, 0.0, -15.0}; // 20 m from the wall, at mid-building height
        const uint32_t id = w.add_vehicle(p);
        // Give it northward velocity by a strong initial push, then coast into the wall.
        // 25 m/s over dt=1/800 needs one impulse: F = m*v/dt applied for exactly one step.
        const double m = p.mass_kg;
        const double dt = 1.0 / 800.0;
        // counteract gravity during the approach so it flies level into the wall
        w.apply_body_wrench(id, {m * 25.0 / dt, 0.0, -m * 9.81}, {0.0, 0.0, 0.0});
        w.step();
        double max_north = -1e9;
        for (int i = 0; i < 2400; ++i) {                                     // 3 s: impact + settle
            w.apply_body_wrench(id, {0.0, 0.0, -m * 9.81}, {0.0, 0.0, 0.0}); // keep it airborne
            w.step();
            max_north = std::max(max_north, w.get_state(id).pos_ned[0]);
        }
        // Wall plane at north=30; body half-extent 0.18 → center may reach ~29.82 + slop.
        CHECK(max_north < 30.0); // center NEVER crosses the wall plane
        CHECK(max_north > 29.5); // and it really reached the wall
        const auto s = w.get_state(id);
        CHECK(s.pos_ned[0] < 30.0); // still outside after settling
        w.remove_vehicle(id);
    }

    // --- (b2) Contact attribution: a building strike must be distinguishable from a
    // landing. static_contacts lumps both together, so building_contacts is what a
    // crash alert can actually key on. ---
    {
        // Land on the ground plane well clear of the building: static_contacts rises,
        // building_contacts must NOT.
        skysim::core::VehicleBodyParams p;
        p.start_pos_ned = {-200.0, 0.0, -2.0}; // south of the building, 2 m up
        const uint32_t id = w.add_vehicle(p);
        for (int i = 0; i < 2400; ++i) { // 3 s: fall and settle on the ground
            w.step();
        }
        const auto landed = w.get_state(id);
        CHECK(landed.static_contacts > 0);    // it did touch something static
        CHECK(landed.building_contacts == 0); // but it was the ground, not a building
        w.remove_vehicle(id);
    }
    {
        // Drop onto the building roof: both counters rise.
        skysim::core::VehicleBodyParams p;
        p.start_pos_ned = {50.0, 0.0, -33.0}; // just above the roof at z=-30
        const uint32_t id = w.add_vehicle(p);
        for (int i = 0; i < 2400; ++i) {
            w.step();
        }
        const auto hit = w.get_state(id);
        CHECK(hit.building_contacts > 0); // struck the tile mesh
        CHECK(hit.static_contacts >= hit.building_contacts);
        w.remove_vehicle(id);
    }

    // --- (c) Rangefinder-style raycasts against the mesh. ---
    {
        // Down onto the roof from 10 m above it.
        const double d_roof = w.raycast({50.0, 0.0, -40.0}, {0.0, 0.0, 1.0}, 100.0);
        CHECK(std::abs(d_roof - 10.0) < 0.01);
        // North into the wall from 10 m south of it, at mid height.
        const double d_wall = w.raycast({20.0, 0.0, -15.0}, {1.0, 0.0, 0.0}, 100.0);
        CHECK(std::abs(d_wall - 10.0) < 0.01);
        // One-sided mesh: a ray from INSIDE the building hits the far wall's back face or
        // nothing before it — outward winding means back faces don't block from inside.
        const double d_inside = w.raycast({50.0, 0.0, -15.0}, {1.0, 0.0, 0.0}, 5.0);
        CHECK(d_inside < 0.0);
    }

    if (g_failures == 0) {
        std::printf("test_collision: all checks OK\n");
        return 0;
    }
    std::printf("test_collision: %d failure(s)\n", g_failures);
    return 1;
}
