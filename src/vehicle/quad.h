#pragma once
// X-quad: 4 motors -> body wrench (FRD). Mixer geometry + drag; wind enters as freestream.
// Motor order/spin must match ArduPilot's X-frame convention — verify at M2 against
// stock SITL (mavproxy `motortest`) before tuning anything else.
#include <array>
#include "vehicle/motor_model.h"

namespace skysim::vehicle {

struct QuadParams {
    double mass_kg = 1.5;
    double arm_m = 0.25;
    std::array<double, 3> inertia_diag{0.02, 0.02, 0.04};
    std::array<double, 3> lin_drag{0.3, 0.3, 0.4};   // N per m/s   TODO(M2): tune
    MotorParams motor;
};

struct BodyWrench { double force_frd[3]; double torque_frd[3]; };

BodyWrench compute_wrench(const QuadParams& p, std::array<MotorState, 4>& motors,
                          const std::array<uint16_t, 16>& pwm,
                          const double vel_air_frd[3], double dt_s);

}  // namespace skysim::vehicle
