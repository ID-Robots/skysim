// See quad.h. Motor layout verified against the pinned ArduPilot source:
// AP_MotorsMatrix::setup_quad_matrix MOTOR_FRAME_TYPE_X, AP_MotorsMatrix.cpp:592-599
// @ Copter-4.7.0. MotorDef = {angle deg CW from nose, yaw factor, test order}; entries are
// added in SERVO order, so pwm[i] drives the (i+1)-th entry:
//   M1 { +45, CCW}  front-right   M2 {-135, CCW}  rear-left
//   M3 { -45, CW }  front-left    M4 {+135, CW }  rear-right
// Yaw factor CCW (+1): a CCW-spinning prop reacts with a clockwise (nose-right, +yaw in
// FRD z-down) torque on the frame when throttled up.
#include "vehicle/quad.h"

#include <cmath>

namespace skysim::vehicle {

namespace {
struct MotorGeom {
    double angle_rad;
    double yaw_dir; // +1: contributes +yaw (CCW prop), -1: -yaw (CW prop)
};
constexpr double kDeg = M_PI / 180.0;
const MotorGeom kQuadX[4] = {
    {45.0 * kDeg, +1.0},
    {-135.0 * kDeg, +1.0},
    {-45.0 * kDeg, -1.0},
    {135.0 * kDeg, -1.0},
};
} // namespace

BodyWrench compute_wrench(const QuadParams &p, std::array<MotorState, 4> &motors,
                          const std::array<uint16_t, 16> &pwm, const double vel_air_frd[3], double dt_s) {
    BodyWrench w{};
    for (int i = 0; i < 4; ++i) {
        const double omega = step_motor(motors[i], p.motor, pwm[i], dt_s);
        const double thrust = p.motor.k_thrust * omega * omega; // N, along -z body (up)
        const double torque = p.motor.k_torque * omega * omega * kQuadX[i].yaw_dir;

        const double x = p.arm_m * std::cos(kQuadX[i].angle_rad); // FRD: +x fwd
        const double y = p.arm_m * std::sin(kQuadX[i].angle_rad); // FRD: +y right

        w.force_frd[2] += -thrust;
        // r x F with r=(x,y,0), F=(0,0,-T): front-right thrust rolls left, pitches nose up.
        w.torque_frd[0] += -y * thrust;
        w.torque_frd[1] += x * thrust;
        w.torque_frd[2] += torque;
    }
    for (int k = 0; k < 3; ++k) {
        w.force_frd[k] -= p.lin_drag[k] * vel_air_frd[k];
    }
    return w;
}

} // namespace skysim::vehicle
