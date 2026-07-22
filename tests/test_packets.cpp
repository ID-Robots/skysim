// M0: static ABI checks + classification tests against a captured real servo datagram.
// Fixture: tests/fixtures/servo_frame.bin — first datagram from arducopter (Copter-4.7.0)
// --model json, captured per docs/PROTOCOL.md. SKYSIM_SOURCE_DIR is set by CMake.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <optional>
#include <vector>

#include "protocol/packets.h"

using skysim::protocol::FrameStatus;
using skysim::protocol::kServoMagic16;
using skysim::protocol::kServoMagic32;
using skysim::protocol::parse_servo_datagram;
using skysim::protocol::ParsedServos;
using skysim::protocol::ServoPacket16;
using skysim::protocol::ServoPacket32;

static_assert(sizeof(ServoPacket16) == 40);
static_assert(sizeof(ServoPacket32) == 72);
static_assert(kServoMagic16 == 18458);
static_assert(kServoMagic32 == 29569);

namespace {

int g_failures = 0;

#define CHECK(cond)                                                                                          \
    do {                                                                                                     \
        if (!(cond)) {                                                                                       \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                      \
            ++g_failures;                                                                                    \
        }                                                                                                    \
    } while (0)

std::vector<std::byte> load_fixture(const char *path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        return {};
    }
    std::vector<char> raw((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    std::vector<std::byte> bytes(raw.size());
    std::memcpy(bytes.data(), raw.data(), raw.size());
    return bytes;
}

// Return a copy of the fixture with frame_count patched (little-endian at offset 4).
std::vector<std::byte> with_frame_count(const std::vector<std::byte> &pkt, uint32_t fc) {
    std::vector<std::byte> out = pkt;
    std::memcpy(out.data() + 4, &fc, sizeof(fc));
    return out;
}

} // namespace

int main() {
    const std::vector<std::byte> fix = load_fixture(SKYSIM_SOURCE_DIR "/tests/fixtures/servo_frame.bin");
    if (fix.empty()) {
        std::printf("FAIL: could not load tests/fixtures/servo_frame.bin\n");
        return 1;
    }
    CHECK(fix.size() == sizeof(ServoPacket16));

    // --- Real captured frame: first-ever datagram parses Ok, header fields as observed. ---
    const ParsedServos first = parse_servo_datagram(fix, std::nullopt);
    CHECK(first.status == FrameStatus::kOk);
    CHECK(first.frame_count == 0);
    CHECK(first.frame_rate_hz == 1200); // SIM_RATE_HZ_DEFAULT, SITL.cpp:46-52 @ Copter-4.7.0
    CHECK(first.channel_count == 16);

    // --- Classification against last seen frame_count (docs/PROTOCOL.md semantics). ---
    // Duplicate: ArduPilot really re-sends frame_count=0 at connection start (observed at
    // capture); the sim must not re-step physics but must still reply.
    CHECK(parse_servo_datagram(fix, 0u).status == FrameStatus::kDuplicate);
    // Ok: exactly the next frame.
    CHECK(parse_servo_datagram(with_frame_count(fix, 8), 7u).status == FrameStatus::kOk);
    // Gap: frames were dropped — log, step anyway.
    CHECK(parse_servo_datagram(with_frame_count(fix, 42), 7u).status == FrameStatus::kGap);
    CHECK(parse_servo_datagram(with_frame_count(fix, 42), 7u).frame_count == 42);
    // Reboot: count went backwards ⇒ ArduPilot restarted; protocol state reset, pose kept.
    CHECK(parse_servo_datagram(with_frame_count(fix, 3), 7u).status == FrameStatus::kReboot);

    // --- Corruption handling. ---
    std::vector<std::byte> bad_magic = fix;
    bad_magic[0] = std::byte{0xFF};
    CHECK(parse_servo_datagram(bad_magic, std::nullopt).status == FrameStatus::kBadMagic);

    std::vector<std::byte> truncated(fix.begin(), fix.end() - 1);
    CHECK(parse_servo_datagram(truncated, std::nullopt).status == FrameStatus::kBadSize);
    std::vector<std::byte> oversize = fix;
    oversize.push_back(std::byte{0});
    CHECK(parse_servo_datagram(oversize, std::nullopt).status == FrameStatus::kBadSize);

    // A 72-byte datagram must carry the 32-channel magic, not the 16-channel one.
    std::vector<std::byte> wrong_variant(sizeof(ServoPacket32), std::byte{0});
    std::memcpy(wrong_variant.data(), fix.data(), 8); // 16-ch header on a 32-ch-sized packet
    CHECK(parse_servo_datagram(wrong_variant, std::nullopt).status == FrameStatus::kBadMagic);

    // --- Synthetic 32-channel variant (SERVO_32_ENABLE=1 path) + pwm payload round-trip. ---
    ServoPacket32 p32{};
    p32.magic = kServoMagic32;
    p32.frame_rate = 400;
    p32.frame_count = 1001;
    for (int i = 0; i < 32; ++i) {
        p32.pwm[i] = static_cast<uint16_t>(1000 + i * 10);
    }
    std::vector<std::byte> raw32(sizeof(p32));
    std::memcpy(raw32.data(), &p32, sizeof(p32));
    const ParsedServos s32 = parse_servo_datagram(raw32, 1000u);
    CHECK(s32.status == FrameStatus::kOk);
    CHECK(s32.frame_rate_hz == 400);
    CHECK(s32.frame_count == 1001);
    CHECK(s32.channel_count == 32);
    CHECK(s32.pwm[0] == 1000);
    CHECK(s32.pwm[31] == 1310);

    // Same pwm round-trip through the 16-channel struct.
    ServoPacket16 p16{};
    p16.magic = kServoMagic16;
    p16.frame_rate = 1200;
    p16.frame_count = 7;
    for (int i = 0; i < 16; ++i) {
        p16.pwm[i] = static_cast<uint16_t>(2000 - i);
    }
    std::vector<std::byte> raw16(sizeof(p16));
    std::memcpy(raw16.data(), &p16, sizeof(p16));
    const ParsedServos s16 = parse_servo_datagram(raw16, 6u);
    CHECK(s16.status == FrameStatus::kOk);
    CHECK(s16.channel_count == 16);
    CHECK(s16.pwm[0] == 2000);
    CHECK(s16.pwm[15] == 1985);

    if (g_failures == 0) {
        std::printf("test_packets: all checks OK\n");
        return 0;
    }
    std::printf("test_packets: %d failure(s)\n", g_failures);
    return 1;
}
