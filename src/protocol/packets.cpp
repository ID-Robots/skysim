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

constexpr uint64_t kPow10[] = {1ull,      10ull,      100ull,      1000ull,     10000ull,
                               100000ull, 1000000ull, 10000000ull, 100000000ull};

// Append a string literal; returns false on overflow.
inline bool append_lit(char *buf, size_t cap, size_t &off, const char *s, size_t len) {
    if (off + len > cap) {
        return false;
    }
    std::memcpy(buf + off, s, len);
    off += len;
    return true;
}
template <size_t N> inline bool append_lit(char *buf, size_t cap, size_t &off, const char (&s)[N]) {
    return append_lit(buf, cap, off, s, N - 1); // drop the trailing NUL
}

// Fixed-precision formatting equivalent to snprintf("%.*f") but ~30x faster: no format
// parsing, no locale, integer-based digit emission. Non-finite -> 0.0 (PROTOCOL.md: no
// NaN/Inf on the wire). prec in [0,8]. Rounds half away from zero (indistinguishable from
// snprintf's round-half-to-even for physical sim data; ArduPilot re-parses with strtod).
inline bool append_fixed(char *buf, size_t cap, size_t &off, double v, int prec) {
    if (off + 40 > cap) { // sign + up to ~24 int digits + '.' + up to 8 frac
        return false;
    }
    if (!std::isfinite(v)) {
        v = 0.0;
    }
    char *p = buf + off;
    if (std::signbit(v)) {
        *p++ = '-';
        v = -v;
    }
    const uint64_t scale = kPow10[prec];
    const uint64_t scaled = static_cast<uint64_t>(v * static_cast<double>(scale) + 0.5);
    uint64_t ip = scaled / scale;
    const uint64_t fp = scaled % scale;

    char tmp[24];
    int ti = 0;
    if (ip == 0) {
        tmp[ti++] = '0';
    }
    while (ip != 0) {
        tmp[ti++] = static_cast<char>('0' + ip % 10);
        ip /= 10;
    }
    while (ti != 0) {
        *p++ = tmp[--ti];
    }
    if (prec > 0) {
        *p++ = '.';
        for (int d = prec - 1; d >= 0; --d) {
            *p++ = static_cast<char>('0' + (fp / kPow10[d]) % 10);
        }
    }
    off = static_cast<size_t>(p - buf);
    return true;
}
} // namespace

size_t build_state_json(const VehicleTruth &t, char *buf, size_t buf_size) {
    size_t off = 0;
    const size_t cap = buf_size;

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

    bool ok =
        append_lit(buf, cap, off, "{\"timestamp\":") && append_fixed(buf, cap, off, t.timestamp_s, 6) &&
        append_lit(buf, cap, off, ",\"imu\":{\"gyro\":[") && append_fixed(buf, cap, off, t.gyro_rps[0], 6) &&
        append_lit(buf, cap, off, ",") && append_fixed(buf, cap, off, t.gyro_rps[1], 6) &&
        append_lit(buf, cap, off, ",") && append_fixed(buf, cap, off, t.gyro_rps[2], 6) &&
        append_lit(buf, cap, off, "],\"accel_body\":[") && append_fixed(buf, cap, off, t.accel_body[0], 6) &&
        append_lit(buf, cap, off, ",") && append_fixed(buf, cap, off, t.accel_body[1], 6) &&
        append_lit(buf, cap, off, ",") && append_fixed(buf, cap, off, t.accel_body[2], 6) &&
        append_lit(buf, cap, off, "]},\"position\":[") && append_fixed(buf, cap, off, t.pos_ned_m[0], 6) &&
        append_lit(buf, cap, off, ",") && append_fixed(buf, cap, off, t.pos_ned_m[1], 6) &&
        append_lit(buf, cap, off, ",") && append_fixed(buf, cap, off, t.pos_ned_m[2], 6) &&
        append_lit(buf, cap, off, "],\"velocity\":[") && append_fixed(buf, cap, off, t.vel_ned_mps[0], 6) &&
        append_lit(buf, cap, off, ",") && append_fixed(buf, cap, off, t.vel_ned_mps[1], 6) &&
        append_lit(buf, cap, off, ",") && append_fixed(buf, cap, off, t.vel_ned_mps[2], 6) &&
        append_lit(buf, cap, off, "],\"attitude\":[") && append_fixed(buf, cap, off, roll, 7) &&
        append_lit(buf, cap, off, ",") && append_fixed(buf, cap, off, pitch, 7) &&
        append_lit(buf, cap, off, ",") && append_fixed(buf, cap, off, yaw, 7) &&
        append_lit(buf, cap, off, "],\"quaternion\":[") && append_fixed(buf, cap, off, qw, 7) &&
        append_lit(buf, cap, off, ",") && append_fixed(buf, cap, off, qx, 7) &&
        append_lit(buf, cap, off, ",") && append_fixed(buf, cap, off, qy, 7) &&
        append_lit(buf, cap, off, ",") && append_fixed(buf, cap, off, qz, 7) &&
        append_lit(buf, cap, off, "]");

    const size_t n_rng = std::min<size_t>(t.rangefinder_count, t.rangefinder_m.size());
    for (size_t i = 0; ok && i < n_rng; ++i) {
        ok = append_lit(buf, cap, off, ",\"rng_", 6) && append_lit(buf, cap, off, &"123456"[i], 1) &&
             append_lit(buf, cap, off, "\":", 2) && append_fixed(buf, cap, off, t.rangefinder_m[i], 4);
    }
    if (ok && t.battery_valid) {
        // ArduPilot reads these only when both bits are present (SIM_JSON.cpp), and
        // integrates current into consumed_mah — which is what makes capacity_remaining_pct
        // fall, and therefore what makes BATT_LOW_MAH / BATT_CRT_MAH able to fire.
        ok = append_lit(buf, cap, off, ",\"battery\":{\"voltage\":") &&
             append_fixed(buf, cap, off, t.battery_voltage_v, 4) &&
             append_lit(buf, cap, off, ",\"current\":") && append_fixed(buf, cap, off, t.battery_current_a, 4) &&
             append_lit(buf, cap, off, "}");
    }
    if (ok && t.no_lockstep) {
        // Compact boolean (no space): ArduPilot's parser reads "true" only without a leading
        // space, else strtoull (docs/PROTOCOL.md parser rules).
        ok = append_lit(buf, cap, off, ",\"no_lockstep\":1");
    }
    if (!ok || !append_lit(buf, cap, off, "}\n")) {
        return 0;
    }
    return off;
}

} // namespace skysim::protocol
