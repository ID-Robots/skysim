// M2: Jolt world behavior through the NED-only World interface — ground rest, free fall,
// hover balance, yaw torque, and same-build determinism (CLAUDE.md invariant 4).
#include <cmath>
#include <cstdio>

#include "core/world.h"

namespace {

int g_failures = 0;

#define CHECK(cond)                                                                                          \
    do {                                                                                                     \
        if (!(cond)) {                                                                                       \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                      \
            ++g_failures;                                                                                    \
        }                                                                                                    \
    } while (0)

constexpr double kG = 9.81;

skysim::core::WorldConfig test_cfg() {
    skysim::core::WorldConfig cfg;
    cfg.dt_s = 1.0 / 800.0;
    cfg.worker_threads = 2; // fixed for determinism comparisons
    return cfg;
}

} // namespace

int main() {
    using namespace skysim::core;

    // --- Rest on ground: stays put, accelerometer reads (0,0,-g). ---
    {
        World w(test_cfg());
        w.add_ground_plane();
        VehicleBodyParams p;
        p.start_pos_ned = {0.0, 0.0, -(p.half_extents_frd[2] + 0.005)};
        const uint32_t id = w.add_vehicle(p);
        for (int i = 0; i < 800; ++i) {
            w.step();
        }
        const BodyState s = w.get_state(id);
        CHECK(std::abs(s.pos_ned[0]) < 0.01 && std::abs(s.pos_ned[1]) < 0.01);
        CHECK(s.pos_ned[2] < 0.0 && s.pos_ned[2] > -0.2); // resting, not sunk/tunneled
        CHECK(std::abs(s.vel_ned[0]) + std::abs(s.vel_ned[1]) + std::abs(s.vel_ned[2]) < 0.02);
        CHECK(std::abs(s.accel_body_frd[2] + kG) < 0.3); // ~ -9.81
        CHECK(std::abs(s.accel_body_frd[0]) < 0.3 && std::abs(s.accel_body_frd[1]) < 0.3);
    }

    // --- Free fall: specific force ~ 0, velocity grows downward (+z NED). ---
    {
        World w(test_cfg());
        w.add_ground_plane();
        VehicleBodyParams p;
        p.start_pos_ned = {0.0, 0.0, -100.0};
        const uint32_t id = w.add_vehicle(p);
        for (int i = 0; i < 80; ++i) { // 0.1 s
            w.step();
        }
        const BodyState s = w.get_state(id);
        CHECK(std::abs(s.accel_body_frd[0]) < 0.2 && std::abs(s.accel_body_frd[1]) < 0.2 &&
              std::abs(s.accel_body_frd[2]) < 0.2);
        CHECK(std::abs(s.vel_ned[2] - kG * 0.1) < 0.05);
        CHECK(s.pos_ned[2] > -100.0);
    }

    // --- Hover: continuous body-z force of -m*g holds altitude and reads -g on the imu. ---
    {
        World w(test_cfg());
        w.add_ground_plane();
        VehicleBodyParams p;
        p.start_pos_ned = {0.0, 0.0, -50.0};
        const uint32_t id = w.add_vehicle(p);
        for (int i = 0; i < 1600; ++i) { // 2 s
            w.apply_body_wrench(id, {0.0, 0.0, -p.mass_kg * kG}, {0.0, 0.0, 0.0});
            w.step();
        }
        const BodyState s = w.get_state(id);
        CHECK(std::abs(s.pos_ned[2] + 50.0) < 0.05);
        CHECK(std::abs(s.vel_ned[2]) < 0.01);
        CHECK(std::abs(s.accel_body_frd[2] + kG) < 0.2);
    }

    // --- Yaw torque: +z body torque yields +z body rate (nose right). ---
    {
        World w(test_cfg());
        VehicleBodyParams p;
        p.start_pos_ned = {0.0, 0.0, -50.0};
        const uint32_t id = w.add_vehicle(p);
        for (int i = 0; i < 80; ++i) {
            w.apply_body_wrench(id, {0.0, 0.0, -p.mass_kg * kG}, {0.0, 0.0, 0.02});
            w.step();
        }
        const BodyState s = w.get_state(id);
        CHECK(s.gyro_frd[2] > 0.01);
        CHECK(std::abs(s.gyro_frd[0]) < 1e-3 && std::abs(s.gyro_frd[1]) < 1e-3);
    }

    // --- Determinism: identical config + inputs => bit-identical trajectory (same build). ---
    {
        auto run = [] {
            World w(test_cfg());
            w.add_ground_plane();
            VehicleBodyParams p;
            p.start_pos_ned = {0.0, 0.0, -3.0};
            const uint32_t id = w.add_vehicle(p);
            for (int i = 0; i < 1200; ++i) { // free fall + ground impact + settle
                w.apply_body_wrench(id, {0.1, 0.05, -10.0}, {0.001, 0.0, 0.002});
                w.step();
            }
            return w.get_state(id);
        };
        const BodyState a = run();
        const BodyState b = run();
        CHECK(a.pos_ned[0] == b.pos_ned[0] && a.pos_ned[1] == b.pos_ned[1] && a.pos_ned[2] == b.pos_ned[2]);
        CHECK(a.vel_ned[0] == b.vel_ned[0] && a.vel_ned[1] == b.vel_ned[1] && a.vel_ned[2] == b.vel_ned[2]);
        CHECK(a.quat_ned_frd[0] == b.quat_ned_frd[0] && a.quat_ned_frd[3] == b.quat_ned_frd[3]);
    }

    // --- Raycast: straight down from 10 m above ground hits at ~10 m. ---
    {
        World w(test_cfg());
        w.add_ground_plane();
        const double d = w.raycast({0.0, 0.0, -10.0}, {0.0, 0.0, 1.0}, 100.0);
        CHECK(std::abs(d - 10.0) < 0.01);
        const double miss = w.raycast({0.0, 0.0, -10.0}, {0.0, 0.0, -1.0}, 100.0);
        CHECK(miss < 0.0);
    }

    if (g_failures == 0) {
        std::printf("test_world: all checks OK\n");
        return 0;
    }
    std::printf("test_world: %d failure(s)\n", g_failures);
    return 1;
}
