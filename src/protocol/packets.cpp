// See docs/PROTOCOL.md. Layouts verified against SIM_JSON.h @ Copter-4.7.0.
#include "protocol/packets.h"

#include <bit>
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

    bool ok = emit("{\"timestamp\":%.6f,\"imu\":{\"gyro\":[%.6f,%.6f,%.6f],\"accel_body\":[%.6f,%.6f,%.6f]},",
                   t.timestamp_s, t.gyro_rps[0], t.gyro_rps[1], t.gyro_rps[2], t.accel_body[0],
                   t.accel_body[1], t.accel_body[2]) &&
              emit("\"position\":[%.6f,%.6f,%.6f],\"velocity\":[%.6f,%.6f,%.6f],", t.pos_ned_m[0],
                   t.pos_ned_m[1], t.pos_ned_m[2], t.vel_ned_mps[0], t.vel_ned_mps[1], t.vel_ned_mps[2]) &&
              emit("\"quaternion\":[%.7f,%.7f,%.7f,%.7f]", t.quat_wxyz[0], t.quat_wxyz[1], t.quat_wxyz[2],
                   t.quat_wxyz[3]);
    if (ok && t.rangefinder_m.has_value()) {
        const auto &rng = *t.rangefinder_m;
        for (size_t i = 0; ok && i < rng.size(); ++i) {
            ok = emit(",\"rng_%zu\":%.4f", i + 1, rng[i]);
        }
    }
    if (!ok || !emit("}\n")) {
        return 0;
    }
    return off;
}

} // namespace skysim::protocol
