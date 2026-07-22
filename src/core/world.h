#pragma once
// Jolt world wrapper. The ONLY file allowed to see Jolt's Y-up axes; everything above
// speaks NED (see CLAUDE.md invariant 2). Init order rules: CLAUDE.md "Jolt-specific".
#include <cstdint>
#include <filesystem>
#include <memory>

#include "core/frames.h"

namespace skysim::core {

struct WorldConfig {
    double dt_s = 1.0 / 400.0;
    uint32_t max_bodies = 4096;
    int worker_threads = -1; // -1 => hardware_concurrency - 1
    uint64_t rng_seed = 1;   // all randomness derives from this (determinism invariant)

    // Wind = steady + gusts. Gusts are a per-axis Ornstein-Uhlenbeck process driven by the
    // world-owned PRNG (DESIGN.md "Vehicle model"): stationary std dev gust_sigma_mps,
    // correlation time gust_tau_s. Advanced once per step(), exact discretization.
    Vec3 wind_steady_ned{0.0, 0.0, 0.0};
    double gust_sigma_mps = 0.0;
    double gust_tau_s = 2.0;
};

struct VehicleBodyParams {
    double mass_kg = 1.5;
    Vec3 half_extents_frd{0.18, 0.18, 0.07}; // box collision shape, FRD half sizes
    Vec3 inertia_diag_frd{0.02, 0.02, 0.04};
    Vec3 start_pos_ned{0.0, 0.0, 0.0}; // body CENTER at spawn (z up-negative)
};

// Truth published after each step. NED / FRD / SI throughout.
struct BodyState {
    Vec3 pos_ned{}; // body center
    Vec3 vel_ned{};
    Quat quat_ned_frd{1.0, 0.0, 0.0, 0.0};
    Vec3 gyro_frd{};       // body rates, rad/s
    Vec3 accel_body_frd{}; // specific force (accelerometer), clamped +/-16 g
    bool on_ground = false;
};

class World {
  public:
    explicit World(const WorldConfig &cfg);
    ~World();
    World(const World &) = delete;
    World &operator=(const World &) = delete;

    // Static environment (add at startup / tick boundaries only).
    void add_ground_plane();                             // flat ground at NED z = 0
    size_t load_tiles(const std::filesystem::path &dir); // load ALL tiles (tests, replay)

    // Streamed tiles (M5): add/remove one cooked .jshape at tick boundaries.
    // Returns 0 on load failure.
    uint32_t add_static_tile(const std::filesystem::path &jshape);
    void remove_static_tile(uint32_t id);
    // Ray from origin_ned along dir_ned (unit); returns hit distance or a negative value.
    // ignore_vehicle_id: skip that vehicle's own body (rangefinders ray from the body center,
    // which would otherwise hit the vehicle's own collision box at distance 0).
    double raycast(const Vec3 &origin_ned, const Vec3 &dir_ned, double max_dist_m,
                   uint32_t ignore_vehicle_id = 0) const;

    // Vehicles (tick boundaries only). Returns body id used by the calls below.
    uint32_t add_vehicle(const VehicleBodyParams &p);
    void remove_vehicle(uint32_t id);

    // Straggler policy (DESIGN.md "interactive"): frozen = kinematic hold — the body keeps
    // its pose, ignores forces and gravity, still collides as an obstacle for others.
    void set_frozen(uint32_t id, bool frozen);

    // Per-tick: queue a body wrench (force/torque in body FRD), then step once by dt.
    void apply_body_wrench(uint32_t id, const Vec3 &force_frd, const Vec3 &torque_frd);
    void step();

    BodyState get_state(uint32_t id) const;

    // Current wind (steady + gust), NED m/s. Constant within a tick; changes only in step().
    Vec3 wind_ned() const;

    double now() const;
    uint64_t tick_index() const;
    double dt() const { return dt_s_; }

  private:
    struct Impl; // keeps Jolt headers out of the rest of the codebase
    std::unique_ptr<Impl> impl_;
    double dt_s_;
};

} // namespace skysim::core
