#pragma once
// ArduPilot SITL JSON backend wire types. See docs/PROTOCOL.md.
// ABI: layouts must match $ARDUPILOT_ROOT/libraries/SITL/SIM_JSON.h byte-for-byte.
// Verified against Copter-4.7.0: SIM_JSON.h:45-50 (16ch), SIM_JSON.h:52-57 (32ch).

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace skysim::protocol {

inline constexpr uint16_t kServoMagic16 = 18458; // SIM_JSON.h:46 @ Copter-4.7.0
inline constexpr uint16_t kServoMagic32 = 29569; // SIM_JSON.h:53 @ Copter-4.7.0

#pragma pack(push, 1)
struct ServoPacket16 {
    uint16_t magic;       // kServoMagic16
    uint16_t frame_rate;  // Hz requested by ArduPilot (SIM_RATE_HZ)
    uint32_t frame_count; // monotonic per boot; reset ⇒ ArduPilot rebooted
    uint16_t pwm[16];     // ~1000..2000
};

// Sent instead of ServoPacket16 when SERVO_32_ENABLE=1; dispatch on size + magic.
struct ServoPacket32 {
    uint16_t magic;       // kServoMagic32
    uint16_t frame_rate;  // Hz requested by ArduPilot (SIM_RATE_HZ)
    uint32_t frame_count; // monotonic per boot; reset ⇒ ArduPilot rebooted
    uint16_t pwm[32];     // ~1000..2000
};
#pragma pack(pop)
static_assert(sizeof(ServoPacket16) == 40, "ABI drift: ServoPacket16");
static_assert(sizeof(ServoPacket32) == 72, "ABI drift: ServoPacket32");

// Result of validating a raw datagram against the expected sequence state.
enum class FrameStatus : uint8_t { kOk, kDuplicate, kGap, kReboot, kBadMagic, kBadSize };

struct ParsedServos {
    FrameStatus status;
    uint32_t frame_count = 0;
    uint16_t frame_rate_hz = 0;
    uint8_t channel_count = 0;      // 16 or 32; 0 when status is kBadMagic/kBadSize
    std::array<uint16_t, 32> pwm{}; // [0, channel_count) valid
};

// Pure function: parse+classify one datagram given the last seen frame_count.
// Classification: size ∉ {40, 72} ⇒ kBadSize; magic mismatch for that size ⇒ kBadMagic;
// no last_frame_count ⇒ kOk; count == last ⇒ kDuplicate (still reply — see PROTOCOL.md);
// count == last+1 ⇒ kOk; count > last+1 ⇒ kGap (step anyway); count < last ⇒ kReboot.
// No allocation, no logging (caller logs).
ParsedServos parse_servo_datagram(std::span<const std::byte> datagram,
                                  std::optional<uint32_t> last_frame_count);

// Truth state for one vehicle, world NED / body FRD, SI units. Filled by the tick.
struct VehicleTruth {
    double timestamp_s;   // physics time, strictly monotonic per vehicle
    double gyro_rps[3];   // body FRD
    double accel_body[3]; // specific force: at rest ≈ {0, 0, -9.81}
    double pos_ned_m[3];
    double vel_ned_mps[3];
    double quat_wxyz[4];
    // Optional sensors: rng_1..rng_<count> emitted iff rangefinder_count > 0.
    std::array<double, 6> rangefinder_m{};
    uint8_t rangefinder_count = 0;
};

// Serialize one reply datagram: single JSON object, '\n'-terminated, snprintf into buf,
// no heap. Returns bytes written, or 0 if buf too small (caller asserts; size buffers at 1024).
size_t build_state_json(const VehicleTruth &t, char *buf, size_t buf_size);

} // namespace skysim::protocol
