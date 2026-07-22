#pragma once
// ArduPilot SITL JSON backend wire types. See docs/PROTOCOL.md.
// ABI: layouts must match $ARDUPILOT_ROOT/libraries/SITL/SIM_JSON.h byte-for-byte.
// M0 task: verify magics/layouts against the pinned tree; add 32-channel variant if present.

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace skysim::protocol {

inline constexpr uint16_t kServoMagic16 = 18458;  // verified against pinned SIM_JSON.h at M0
// inline constexpr uint16_t kServoMagic32 = 29569;  // VERIFY at M0 before enabling

#pragma pack(push, 1)
struct ServoPacket16 {
    uint16_t magic;        // kServoMagic16
    uint16_t frame_rate;   // Hz requested by ArduPilot (SIM_RATE_HZ)
    uint32_t frame_count;  // monotonic per boot; reset ⇒ ArduPilot rebooted
    uint16_t pwm[16];      // ~1000..2000
};
#pragma pack(pop)
static_assert(sizeof(ServoPacket16) == 40, "ABI drift: ServoPacket16");

// Result of validating a raw datagram against the expected sequence state.
enum class FrameStatus : uint8_t { kOk, kDuplicate, kGap, kReboot, kBadMagic, kBadSize };

struct ParsedServos {
    FrameStatus status;
    uint32_t frame_count = 0;
    uint16_t frame_rate_hz = 0;
    std::array<uint16_t, 16> pwm{};
};

// Pure function: parse+classify one datagram given the last seen frame_count.
// No allocation, no logging (caller logs).
ParsedServos parse_servo_datagram(std::span<const std::byte> datagram,
                                  std::optional<uint32_t> last_frame_count);

// Truth state for one vehicle, world NED / body FRD, SI units. Filled by the tick.
struct VehicleTruth {
    double timestamp_s;      // physics time, strictly monotonic per vehicle
    double gyro_rps[3];      // body FRD
    double accel_body[3];    // specific force: at rest ≈ {0, 0, -9.81}
    double pos_ned_m[3];
    double vel_ned_mps[3];
    double quat_wxyz[4];
    // Optional sensor extras (emitted only if enabled for the vehicle):
    std::optional<std::array<double, 6>> rangefinder_m;
};

// Serialize one reply datagram: single JSON object, '\n'-terminated, snprintf into buf,
// no heap. Returns bytes written, or 0 if buf too small (caller asserts; size buffers at 1024).
size_t build_state_json(const VehicleTruth& t, char* buf, size_t buf_size);

}  // namespace skysim::protocol
