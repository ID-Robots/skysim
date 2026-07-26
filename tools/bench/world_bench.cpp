// skysim world-step microbenchmark: create N hovering quads, run the full tick pipeline
// (compute_wrench -> apply -> step -> get_state) and report per-stage timing.
// Usage: world_bench [N=100] [ticks=2000] [--collide] [--max-p99-us N]
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "core/world.h"
#include "vehicle/quad.h"

using Clock = std::chrono::steady_clock;
using skysim::core::Vec3;

namespace {
double pct(std::vector<double> &v, double p) {
    if (v.empty()) {
        return 0.0;
    }
    std::sort(v.begin(), v.end());
    return v[std::min(v.size() - 1, static_cast<size_t>(v.size() * p))];
}
} // namespace

int main(int argc, char **argv) {
    int n = 100;
    int ticks = 2000;
    bool collide = false;
    int threads = -1;
    double spacing_override = -1.0; // meters between vehicles; overrides --collide's 0.3
    double max_p99_us = -1.0;
    int positional = 0;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--collide") == 0) {
            collide = true;
        } else if (std::strcmp(argv[i], "--threads") == 0 && i + 1 < argc) {
            threads = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--spacing") == 0 && i + 1 < argc) {
            spacing_override = std::atof(argv[++i]);
        } else if (std::strcmp(argv[i], "--max-p99-us") == 0 && i + 1 < argc) {
            max_p99_us = std::atof(argv[++i]);
        } else if (argv[i][0] == '-') {
            std::fprintf(stderr, "unknown or incomplete option: %s\n", argv[i]);
            return 2;
        } else if (positional++ == 0) {
            n = std::atoi(argv[i]);
        } else if (positional == 2) {
            ticks = std::atoi(argv[i]);
        } else {
            std::fprintf(stderr, "unexpected argument: %s\n", argv[i]);
            return 2;
        }
    }
    if (n <= 0 || ticks <= 0 || max_p99_us == 0.0) {
        std::fprintf(stderr, "N, ticks, and --max-p99-us (when set) must be positive\n");
        return 2;
    }

    skysim::core::WorldConfig cfg;
    cfg.dt_s = 1.0 / 800.0;
    cfg.worker_threads = threads;
    cfg.max_bodies = static_cast<uint32_t>(n + 64);
    skysim::core::World world(cfg);
    world.add_ground_plane();

    struct V {
        uint32_t id;
        skysim::vehicle::QuadParams params;
        std::array<skysim::vehicle::MotorState, 4> motors{};
        skysim::core::BodyState state;
    };
    std::vector<V> fleet(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        skysim::core::VehicleBodyParams bp;
        // Grid layout; --collide packs to 0.3 m (overlap soup); --spacing S sets it explicitly.
        const double spacing = spacing_override > 0.0 ? spacing_override : (collide ? 0.3 : 5.0);
        const int cols = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(n))));
        bp.start_pos_ned = {spacing * (i / cols), spacing * (i % cols), -50.0};
        fleet[static_cast<size_t>(i)].id = world.add_vehicle(bp);
        fleet[static_cast<size_t>(i)].state = world.get_state(fleet[static_cast<size_t>(i)].id);
    }

    // PWM ~ hover for all motors.
    std::array<uint16_t, 16> pwm{};
    pwm.fill(1000);
    for (int k = 0; k < 4; ++k) {
        pwm[k] = 1578;
    }

    std::vector<double> t_wrench, t_step, t_state, t_total;
    t_wrench.reserve(ticks);
    t_step.reserve(ticks);
    t_state.reserve(ticks);
    t_total.reserve(ticks);

    for (int t = 0; t < ticks; ++t) {
        const auto a = Clock::now();
        const Vec3 wind = world.wind_ned();
        for (auto &v : fleet) {
            const auto vel_frd = skysim::core::quat_rotate_inverse(v.state.quat_ned_frd, v.state.vel_ned);
            const double vel_air[3] = {vel_frd[0] - wind[0], vel_frd[1] - wind[1], vel_frd[2] - wind[2]};
            const auto w = skysim::vehicle::compute_wrench(v.params, v.motors, pwm, vel_air, cfg.dt_s);
            world.apply_body_wrench(v.id, {w.force_frd[0], w.force_frd[1], w.force_frd[2]},
                                    {w.torque_frd[0], w.torque_frd[1], w.torque_frd[2]});
        }
        const auto b = Clock::now();
        world.step();
        const auto c = Clock::now();
        for (auto &v : fleet) {
            v.state = world.get_state(v.id);
        }
        const auto d = Clock::now();
        auto us = [](auto x, auto y) { return std::chrono::duration<double, std::micro>(y - x).count(); };
        t_wrench.push_back(us(a, b));
        t_step.push_back(us(b, c));
        t_state.push_back(us(c, d));
        t_total.push_back(us(a, d));
    }

    const double dt_us = cfg.dt_s * 1e6;
    std::printf("skysim world_bench: N=%d ticks=%d collide=%d dt=%.1f us\n", n, ticks, collide, dt_us);
    auto row = [&](const char *name, std::vector<double> &v) {
        std::printf("  %-8s p50=%7.1f  p99=%7.1f  max=%7.1f us\n", name, pct(v, 0.50), pct(v, 0.99),
                    pct(v, 1.0));
    };
    row("wrench", t_wrench);
    row("step", t_step);
    row("getstate", t_state);
    row("TOTAL", t_total);
    const double p99 = pct(t_total, 0.99);
    std::printf("  realtime headroom @ %d veh: tick p99 %.1f us vs dt %.1f us => %.2fx %s\n", n, p99, dt_us,
                dt_us / p99, p99 < dt_us ? "OK" : "OVER BUDGET");
    if (max_p99_us > 0.0) {
        const bool pass = p99 <= max_p99_us;
        std::printf("  performance gate: p99 %.1f us <= %.1f us => %s\n", p99, max_p99_us,
                    pass ? "PASS" : "FAIL");
        return pass ? 0 : 1;
    }
    return 0;
}
