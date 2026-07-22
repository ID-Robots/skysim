// See docs/PROTOCOL.md. Layouts verified against SIM_JSON.h @ Copter-4.7.0.
#include "protocol/packets.h"

#include <bit>
#include <cstddef>
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

} // namespace skysim::protocol
