#pragma once
// PWM -> thrust/torque with first-order lag. Pure, per-motor, called from parallel force jobs.
// Params live in per-frame TOML (M2); tuned against stock SITL hover/step response.
#include <cstdint>

namespace skysim::vehicle {

struct MotorParams {
    double tau_s = 0.03;       // spin-up lag
    double k_thrust = 7.65e-6; // N/(rad/s)^2 — hover at pwm ~1578, matching measured stock
                               // SITL quad hover (HOVER_PWM=1577.9, see M2 commit message)
    double k_torque = 1.1e-7;  // N.m/(rad/s)^2 (~1.4 cm torque/thrust ratio)
    double omega_max = 1200;   // rad/s at PWM 2000
};

struct MotorState {
    double omega = 0.0;
};

// u = clamp((pwm-1000)/1000, 0, 1); omega' = (u*omega_max - omega)/tau
double step_motor(MotorState &s, const MotorParams &p, uint16_t pwm_us, double dt_s);

} // namespace skysim::vehicle
