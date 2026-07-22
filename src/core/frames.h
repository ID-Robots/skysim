#pragma once
// NED (world) / FRD (body) <-> Jolt (Y-up) conversions as pure array math, so they are
// unit-testable without Jolt types. world.cpp is the only consumer on the Jolt side
// (CLAUDE.md invariant 2: Jolt axes never leak above core/world.h).
//
// Basis change (jolt_from_ned): jolt.x = north, jolt.y = -down (up), jolt.z = east.
// This is R_x(+90 deg), a proper rotation (det +1), so quaternions map by conjugation:
// q_jolt = q_m * q_ned * conj(q_m), with q_m = (cos45, sin45, 0, 0).
#include <array>
#include <cmath>

namespace skysim::core {

using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>; // w, x, y, z

inline Vec3 ned_to_jolt(const Vec3 &n) { return {n[0], -n[2], n[1]}; }
inline Vec3 jolt_to_ned(const Vec3 &j) { return {j[0], j[2], -j[1]}; }

// Body-local FRD (x fwd, y right, z down) <-> Jolt body local (x fwd, y up, z right):
// the same basis change applied to body axes, so the identical component swap applies.
inline Vec3 frd_to_jolt_local(const Vec3 &f) { return ned_to_jolt(f); }
inline Vec3 jolt_local_to_frd(const Vec3 &j) { return jolt_to_ned(j); }

inline Quat quat_multiply(const Quat &a, const Quat &b) {
    return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

inline Quat quat_conjugate(const Quat &q) { return {q[0], -q[1], -q[2], -q[3]}; }

inline Quat quat_normalize(const Quat &q) {
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    return {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
}

// Rotate vector by quaternion (active rotation, body->world for an attitude quat).
inline Vec3 quat_rotate(const Quat &q, const Vec3 &v) {
    const Quat p{0.0, v[0], v[1], v[2]};
    const Quat r = quat_multiply(quat_multiply(q, p), quat_conjugate(q));
    return {r[1], r[2], r[3]};
}

inline Vec3 quat_rotate_inverse(const Quat &q, const Vec3 &v) { return quat_rotate(quat_conjugate(q), v); }

namespace detail {
inline const Quat kJoltFromNed{std::sqrt(0.5), std::sqrt(0.5), 0.0, 0.0}; // R_x(+90 deg)
}

// Attitude quaternions: q_ned rotates body-FRD into world-NED; q_jolt rotates
// Jolt-body-local into Jolt-world. Same physical rotation, conjugated basis.
inline Quat quat_ned_to_jolt(const Quat &q_ned) {
    return quat_multiply(quat_multiply(detail::kJoltFromNed, q_ned), quat_conjugate(detail::kJoltFromNed));
}

inline Quat quat_jolt_to_ned(const Quat &q_jolt) {
    return quat_multiply(quat_multiply(quat_conjugate(detail::kJoltFromNed), q_jolt), detail::kJoltFromNed);
}

} // namespace skysim::core
