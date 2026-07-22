// See docs/PROTOCOL.md. Layouts verified against SIM_JSON.h @ Copter-4.7.0.
#include "protocol/packets.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>

namespace skysim::protocol {

// Invariant 6 (CLAUDE.md): wire format is little-endian and we memcpy fields straight out.
static_assert(std::endian::native == std::endian::little, "skysim assumes a little-endian host (x86/ARM64)");

// Both variants share the 8-byte header, so ServoPacket16 offsets serve for both.
static_assert(offsetof(ServoPacket16, magic) == offsetof(ServoPacket32, magic));
static_assert(offsetof(ServoPacket16, frame_rate) == offsetof(ServoPacket32, frame_rate));
static_assert(offsetof(ServoPacket16, frame_count) == offsetof(ServoPacket32, frame_count));
static_assert(offsetof(ServoPacket16, pwm) == offsetof(ServoPacket32, pwm));

ParsedServos parse_servo_datagram(std::span<const std::byte> datagram,
                                  std::optional<uint32_t> last_frame_count) {
    ParsedServos out{};

    uint16_t expected_magic = 0;
    uint8_t channels = 0;
    switch (datagram.size()) {
    case sizeof(ServoPacket16):
        expected_magic = kServoMagic16;
        channels = 16;
        break;
    case sizeof(ServoPacket32):
        expected_magic = kServoMagic32;
        channels = 32;
        break;
    default:
        out.status = FrameStatus::kBadSize;
        return out;
    }

    uint16_t magic = 0;
    std::memcpy(&magic, datagram.data() + offsetof(ServoPacket16, magic), sizeof(magic));
    if (magic != expected_magic) {
        out.status = FrameStatus::kBadMagic;
        return out;
    }

    std::memcpy(&out.frame_rate_hz, datagram.data() + offsetof(ServoPacket16, frame_rate),
                sizeof(out.frame_rate_hz));
    std::memcpy(&out.frame_count, datagram.data() + offsetof(ServoPacket16, frame_count),
                sizeof(out.frame_count));
    out.channel_count = channels;
    std::memcpy(out.pwm.data(), datagram.data() + offsetof(ServoPacket16, pwm), channels * sizeof(uint16_t));

    if (!last_frame_count.has_value()) {
        out.status = FrameStatus::kOk;
    } else if (out.frame_count == *last_frame_count) {
        out.status = FrameStatus::kDuplicate;
    } else if (out.frame_count < *last_frame_count) {
        out.status = FrameStatus::kReboot;
    } else if (out.frame_count == *last_frame_count + 1) {
        out.status = FrameStatus::kOk;
    } else {
        out.status = FrameStatus::kGap;
    }
    return out;
}

// ArduPilot's strstr parser rules (docs/PROTOCOL.md): compact JSON, no spaces anywhere,
// "velocity" must appear before any key containing it as a substring (we emit it once),
// single line, '\n'-terminated. Field order matches the pybullet reference implementation.
//
// Non-finite guard: PROTOCOL.md makes "no NaN/Inf in the reply" a hard interop rule —
// glibc prints "nan"/"inf", which ArduPilot's skip-to-digit parser walks straight over,
// desyncing the field scan (required-field reject => permanent lockstep stall, or silent
// value theft from the next field). The emitter is the last line of defense: substitute
// 0.0 so the reply stays parseable; upstream owns not producing non-finites.
namespace {
double fin(double v) { return std::isfinite(v) ? v : 0.0; }
} // namespace

size_t build_state_json(const VehicleTruth &t, char *buf, size_t buf_size) {
    size_t off = 0;
    auto emit = [&](const char *fmt, auto... args) {
        if (off >= buf_size) {
            return false;
        }
        const int n = std::snprintf(buf + off, buf_size - off, fmt, args...);
        if (n < 0 || static_cast<size_t>(n) >= buf_size - off) {
            return false;
        }
        off += static_cast<size_t>(n);
        return true;
    };

    // A quaternion with any non-finite component is unusable as a whole: fall back to identity.
    const bool quat_ok = std::isfinite(t.quat_wxyz[0]) && std::isfinite(t.quat_wxyz[1]) &&
                         std::isfinite(t.quat_wxyz[2]) && std::isfinite(t.quat_wxyz[3]);
    const double qw = quat_ok ? t.quat_wxyz[0] : 1.0;
    const double qx = quat_ok ? t.quat_wxyz[1] : 0.0;
    const double qy = quat_ok ? t.quat_wxyz[2] : 0.0;
    const double qz = quat_ok ? t.quat_wxyz[3] : 0.0;

    // Euler attitude is emitted IN ADDITION to the quaternion (which ArduPilot prefers,
    // SIM_JSON.cpp:430-435) because of an upstream quirk at Copter-4.7.0: the rangefinder
    // copy loop (SIM_JSON.cpp:465-470) still uses pre-lat/lon/alt keytable bit indices, so
    // rangefinder_m[0] is only updated when the EULER_ATT bit is present. See PROTOCOL.md.
    const double sinp = std::clamp(2.0 * (qw * qy - qx * qz), -1.0, 1.0);
    const double roll = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
    const double pitch = std::asin(sinp);
    const double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    bool ok = emit("{\"timestamp\":%.6f,\"imu\":{\"gyro\":[%.6f,%.6f,%.6f],\"accel_body\":[%.6f,%.6f,%.6f]},",
                   fin(t.timestamp_s), fin(t.gyro_rps[0]), fin(t.gyro_rps[1]), fin(t.gyro_rps[2]),
                   fin(t.accel_body[0]), fin(t.accel_body[1]), fin(t.accel_body[2])) &&
              emit("\"position\":[%.6f,%.6f,%.6f],\"velocity\":[%.6f,%.6f,%.6f],", fin(t.pos_ned_m[0]),
                   fin(t.pos_ned_m[1]), fin(t.pos_ned_m[2]), fin(t.vel_ned_mps[0]), fin(t.vel_ned_mps[1]),
                   fin(t.vel_ned_mps[2])) &&
              emit("\"attitude\":[%.7f,%.7f,%.7f],", roll, pitch, yaw) &&
              emit("\"quaternion\":[%.7f,%.7f,%.7f,%.7f]", qw, qx, qy, qz);
    const size_t n_rng = std::min<size_t>(t.rangefinder_count, t.rangefinder_m.size());
    for (size_t i = 0; ok && i < n_rng; ++i) {
        ok = emit(",\"rng_%zu\":%.4f", i + 1, fin(t.rangefinder_m[i]));
    }
    if (!ok || !emit("%s", "}\n")) {
        return 0;
    }
    return off;
}

} // namespace skysim::protocol
