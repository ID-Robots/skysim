// M2: NED/FRD <-> Jolt conversion invariants (core/frames.h). Pure math, no Jolt.
#include <cmath>
#include <cstdio>

#include "core/frames.h"

namespace {

int g_failures = 0;

#define CHECK(cond)                                                                                          \
    do {                                                                                                     \
        if (!(cond)) {                                                                                       \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                      \
            ++g_failures;                                                                                    \
        }                                                                                                    \
    } while (0)

bool near3(const skysim::core::Vec3 &a, const skysim::core::Vec3 &b, double tol = 1e-12) {
    return std::abs(a[0] - b[0]) < tol && std::abs(a[1] - b[1]) < tol && std::abs(a[2] - b[2]) < tol;
}

} // namespace

int main() {
    using namespace skysim::core;

    // Axis mapping: north->x, up->y, east->z.
    CHECK(near3(ned_to_jolt({1, 0, 0}), {1, 0, 0}));  // north
    CHECK(near3(ned_to_jolt({0, 1, 0}), {0, 0, 1}));  // east -> jolt z
    CHECK(near3(ned_to_jolt({0, 0, 1}), {0, -1, 0})); // down -> jolt -y

    // Round trips.
    const Vec3 v{1.5, -2.25, 3.125};
    CHECK(near3(jolt_to_ned(ned_to_jolt(v)), v));
    CHECK(near3(ned_to_jolt(jolt_to_ned(v)), v));

    // Handedness preserved: cross(north, east) == down must map to cross of images.
    // north x east = down: jolt images (1,0,0) x (0,0,1) = (0*1-0*0, 0*0-1*1, 0) = (0,-1,0) ✓
    CHECK(near3(ned_to_jolt({0, 0, 1}), {0, -1, 0}));

    // Quaternion conjugation: rotating a vector must commute with the basis change:
    // ned_to_jolt(q_ned * v) == q_jolt * ned_to_jolt(v) for arbitrary q, v.
    const Quat q_ned = quat_normalize({0.9, 0.2, -0.3, 0.15});
    const Quat q_jolt = quat_ned_to_jolt(q_ned);
    const Vec3 lhs = ned_to_jolt(quat_rotate(q_ned, v));
    const Vec3 rhs = quat_rotate(q_jolt, ned_to_jolt(v));
    CHECK(near3(lhs, rhs, 1e-9));

    // Quaternion round trip.
    const Quat back = quat_jolt_to_ned(q_jolt);
    CHECK(std::abs(back[0] - q_ned[0]) < 1e-12 && std::abs(back[1] - q_ned[1]) < 1e-12 &&
          std::abs(back[2] - q_ned[2]) < 1e-12 && std::abs(back[3] - q_ned[3]) < 1e-12);

    // Known attitude: 90 deg yaw right (nose to east). q = (cos45, 0, 0, sin45) in NED.
    const Quat yaw90{std::sqrt(0.5), 0, 0, std::sqrt(0.5)};
    CHECK(near3(quat_rotate(yaw90, {1, 0, 0}), {0, 1, 0}, 1e-12)); // fwd -> east
    // At rest under gravity the accelerometer must read (0,0,-g) regardless of yaw.
    const Vec3 f_ned{0, 0, -9.81};
    CHECK(near3(quat_rotate_inverse(yaw90, f_ned), {0, 0, -9.81}, 1e-9));

    // 90 deg roll right: gravity specific force appears along -y (left wing... right wing down).
    const Quat roll90{std::sqrt(0.5), std::sqrt(0.5), 0, 0};
    CHECK(near3(quat_rotate_inverse(roll90, f_ned), {0, -9.81, 0}, 1e-9));

    if (g_failures == 0) {
        std::printf("test_frames: all checks OK\n");
        return 0;
    }
    std::printf("test_frames: %d failure(s)\n", g_failures);
    return 1;
}
