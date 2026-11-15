#pragma once
// The survey still camera's request and answer, shared by the control plane (which parses the
// query and writes the headers) and the renderer (which draws the frame). Header-only and
// free of renderer types, so the HTTP layer can use it without linking the renderer.
//
// Names, defaults and bounds are the 3D-mapping contract's (GET /instances/<n>/still.jpg):
// w, h, hfov, pitch, yaw, roll_stab, range, q.
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace skysim::render {

struct StillRequest {
    static constexpr int kDefaultWidth = 1600;
    static constexpr int kDefaultHeight = 1200;
    static constexpr int kMinSize = 64;
    static constexpr int kMaxSize = 4096;
    static constexpr double kDefaultHfovDeg = 73.7; // a 24 mm-equivalent lens
    static constexpr double kMinHfovDeg = 10.0;
    static constexpr double kMaxHfovDeg = 120.0;
    static constexpr double kDefaultPitchDeg = -90.0; // straight down
    static constexpr double kMinPitchDeg = -90.0;
    static constexpr double kMaxPitchDeg = 0.0;
    static constexpr double kMaxYawDeg = 360.0; // either way
    static constexpr double kDefaultRangeM = 600.0;
    static constexpr double kMinRangeM = 50.0;
    static constexpr double kMaxRangeM = 3000.0;
    static constexpr int kDefaultQuality = 90;
    static constexpr int kMinQuality = 1;
    static constexpr int kMaxQuality = 100;

    int width{kDefaultWidth};
    int height{kDefaultHeight};
    double hfov_deg{kDefaultHfovDeg};
    double pitch_deg{kDefaultPitchDeg};
    double yaw_deg{0.0};  // relative to the vehicle's heading
    bool roll_stab{true}; // gimbal-stabilised: vehicle roll and pitch do not tilt the camera
    // gimbal=1 (TASK-2548): look where the vehicle's own gimbal points — its pitch, and its yaw
    // added to the heading, stabilised — and use pitch/yaw/roll_stab above only for a vehicle
    // that has none. What a survey camera on a gimbal photographs.
    bool follow_gimbal{false};
    double range_m{kDefaultRangeM};
    int quality{kDefaultQuality};

    // Empty when every field is in bounds, else what is wrong with the first one that is not.
    std::string invalid() const {
        if (width < kMinSize || width > kMaxSize) {
            return "w must be an integer between 64 and 4096";
        }
        if (height < kMinSize || height > kMaxSize) {
            return "h must be an integer between 64 and 4096";
        }
        if (!(hfov_deg >= kMinHfovDeg && hfov_deg <= kMaxHfovDeg)) {
            return "hfov must be between 10 and 120";
        }
        if (!(pitch_deg >= kMinPitchDeg && pitch_deg <= kMaxPitchDeg)) {
            return "pitch must be between -90 and 0";
        }
        if (!(yaw_deg >= -kMaxYawDeg && yaw_deg <= kMaxYawDeg)) {
            return "yaw must be between -360 and 360";
        }
        if (!(range_m >= kMinRangeM && range_m <= kMaxRangeM)) {
            return "range must be between 50 and 3000";
        }
        if (quality < kMinQuality || quality > kMaxQuality) {
            return "q must be an integer between 1 and 100";
        }
        return {};
    }
};

// The vehicle's pose as one tick published it: everything a frame is drawn from comes from
// the same snapshot, so the picture and its pose headers can never be from different ticks.
struct StillPose {
    std::array<double, 3> position_ned{0.0, 0.0, 0.0};
    std::array<double, 4> quat_ned_frd{1.0, 0.0, 0.0, 0.0}; // w, x, y, z; FRD -> NED
    double sim_time_s{0.0};
    // The vehicle's gimbal in the same tick (vehicle/gimbal.h): earth-frame pitch, yaw
    // relative to the heading. Absent on a vehicle that drives none.
    bool gimbal{false};
    double gimbal_pitch_deg{0.0};
    double gimbal_yaw_deg{0.0};
};

// Camera axes as NED Euler angles (Z-Y-X), degrees: yaw 0..360 clockwise from north, pitch
// -90 (down) .. 90, roll about the optical axis.
struct CameraAngles {
    double yaw_deg{0.0};
    double pitch_deg{0.0};
    double roll_deg{0.0};
};

struct StillFrame {
    enum class Status {
        kOk,
        kBusy,     // every render slot taken: 503 + Retry-After
        kInvalid,  // a request field out of bounds: 400
        kDisabled, // no still camera on this server: 404
        kFailed,   // the render itself failed (out of memory, encoder): 500
    };
    Status status{Status::kDisabled};
    std::vector<uint8_t> jpeg;
    CameraAngles camera;
    bool gimbal{false}; // drawn where the vehicle's gimbal points (X-Skysim-Gimbal)
    std::string error;
};

} // namespace skysim::render
