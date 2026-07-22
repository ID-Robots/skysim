// M2: X-quad wrench sanity — mixer signs per ArduPilot QUAD_X (AP_MotorsMatrix.cpp:592-599
// @ Copter-4.7.0), hover thrust matches the tuned stock-SITL hover point.
#include <array>
#include <cmath>
#include <cstdio>

#include "vehicle/quad.h"

namespace {

int g_failures = 0;

#define CHECK(cond)                                                                                          \
    do {                                                                                                     \
        if (!(cond)) {                                                                                       \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                      \
            ++g_failures;                                                                                    \
        }                                                                                                    \
    } while (0)

constexpr double kDt = 1.0 / 800.0;
constexpr double kZero3[3] = {0.0, 0.0, 0.0};

// Run the motors to steady state at the given pwm, return the final wrench.
skysim::vehicle::BodyWrench steady_wrench(const skysim::vehicle::QuadParams &p,
                                          const std::array<uint16_t, 16> &pwm) {
    std::array<skysim::vehicle::MotorState, 4> motors{};
    skysim::vehicle::BodyWrench w{};
    for (int i = 0; i < 2000; ++i) { // 2.5 s >> tau (30 ms)
        w = compute_wrench(p, motors, pwm, kZero3, kDt);
    }
    return w;
}

std::array<uint16_t, 16> all_pwm(uint16_t v) {
    std::array<uint16_t, 16> pwm{};
    pwm.fill(1000);
    pwm[0] = pwm[1] = pwm[2] = pwm[3] = v;
    return pwm;
}

} // namespace

int main() {
    skysim::vehicle::QuadParams p;

    // Hover: at the stock-SITL-measured hover pwm (~1578) total thrust ~= m*g.
    {
        const auto w = steady_wrench(p, all_pwm(1578));
        const double weight = p.mass_kg * 9.81;
        CHECK(std::abs(-w.force_frd[2] - weight) < 0.05 * weight);
        CHECK(std::abs(w.torque_frd[0]) < 1e-9); // symmetric => no roll/pitch/yaw
        CHECK(std::abs(w.torque_frd[1]) < 1e-9);
        CHECK(std::abs(w.torque_frd[2]) < 1e-9);
    }

    // Motors off: no thrust, no torque.
    {
        const auto w = steady_wrench(p, all_pwm(1000));
        CHECK(std::abs(w.force_frd[2]) < 1e-9);
    }

    // Roll right: raise LEFT motors (M2 rear-left = pwm[1], M3 front-left = pwm[2]).
    {
        auto pwm = all_pwm(1500);
        pwm[1] += 100;
        pwm[2] += 100;
        const auto w = steady_wrench(p, pwm);
        CHECK(w.torque_frd[0] > 0.01); // +roll (right side down)
        CHECK(std::abs(w.torque_frd[1]) < 1e-6);
    }

    // Pitch down (how a quad accelerates forward): raise REAR motors
    // (M2 rear-left = pwm[1], M4 rear-right = pwm[3]) => tail up, nose down.
    {
        auto pwm = all_pwm(1500);
        pwm[1] += 100;
        pwm[3] += 100;
        const auto w = steady_wrench(p, pwm);
        CHECK(w.torque_frd[1] < -0.01); // -pitch (nose down)
        CHECK(std::abs(w.torque_frd[0]) < 1e-6);
    }

    // Yaw right (+z FRD): raise the CCW pair (M1 = pwm[0], M2 = pwm[1]).
    {
        auto pwm = all_pwm(1500);
        pwm[0] += 100;
        pwm[1] += 100;
        const auto w = steady_wrench(p, pwm);
        CHECK(w.torque_frd[2] > 1e-4);
    }

    // Drag opposes airspeed.
    {
        std::array<skysim::vehicle::MotorState, 4> motors{};
        const double vel[3] = {5.0, 0.0, 0.0};
        const auto w = compute_wrench(p, motors, all_pwm(1000), vel, kDt);
        CHECK(w.force_frd[0] < -1.0);
    }

    if (g_failures == 0) {
        std::printf("test_quad: all checks OK\n");
        return 0;
    }
    std::printf("test_quad: %d failure(s)\n", g_failures);
    return 1;
}
