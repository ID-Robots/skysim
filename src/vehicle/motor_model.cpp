// See motor_model.h. Implicit-Euler discretization of omega' = (u*omega_max - omega)/tau —
// unconditionally stable for any dt, matches the continuous model to first order.
#include "vehicle/motor_model.h"

#include <algorithm>

namespace skysim::vehicle {

double step_motor(MotorState &s, const MotorParams &p, uint16_t pwm_us, double dt_s) {
    const double u = std::clamp((static_cast<double>(pwm_us) - 1000.0) / 1000.0, 0.0, 1.0);
    const double target = u * p.omega_max;
    const double alpha = dt_s / (p.tau_s + dt_s);
    s.omega += alpha * (target - s.omega);
    return s.omega;
}

} // namespace skysim::vehicle
