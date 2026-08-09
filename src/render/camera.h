#pragma once

#include "core/frames.h"

#include <cmath>
#include <cstdint>

namespace skysim::render {

using core::Quat;
using core::Vec3;

// Deliberately small. This is a situational view for a pilot flying by stick —
// "is there a building in front of me", not an inspection image — and every
// pixel costs a ray through the physics world. 256x144 at 10 Hz is about 370k
// rays a second per vehicle, which a handful of cores can hold while the
// simulation keeps its tick.
struct CameraConfig {
    int width{256};
    int height{144};
    double fov_deg{78.0};   // horizontal, close to a typical drone camera
    double pitch_deg{-15.0}; // nose-down tilt of the mount, degrees
    double max_range_m{350.0}; // draw distance; rays cost, and haze hides the cut
};

// Rays for a pinhole camera rigidly mounted to the airframe.
//
// The vehicle's attitude comes in as the NED->FRD quaternion the physics uses,
// so the mount tilt is applied in body axes and the result rotated into world
// axes — the same order the real gimbal sits in.
class Camera {
  public:
    Camera(const CameraConfig &cfg, const Vec3 &position_ned, const Quat &quat_ned_frd)
        : cfg_(cfg), position_(position_ned), quat_(core::quat_normalize(quat_ned_frd)) {
        const double aspect = static_cast<double>(cfg.width) / static_cast<double>(cfg.height);
        tan_half_fov_x_ = std::tan((cfg.fov_deg * M_PI / 180.0) * 0.5);
        tan_half_fov_y_ = tan_half_fov_x_ / aspect;
        const double p = cfg.pitch_deg * M_PI / 180.0;
        cos_pitch_ = std::cos(p);
        sin_pitch_ = std::sin(p);
    }

    const Vec3 &position() const { return position_; }

    // Unit ray direction in NED for a pixel. x right, y down, both 0-based.
    Vec3 ray(int x, int y) const {
        // Pixel centre in [-1, 1], y flipped so row 0 is the top of the image.
        const double sx = ((x + 0.5) / cfg_.width) * 2.0 - 1.0;
        const double sy = 1.0 - ((y + 0.5) / cfg_.height) * 2.0;

        // Camera-local FRD: forward is +x, right +y, down +z.
        const double fwd = 1.0;
        const double right = sx * tan_half_fov_x_;
        const double down = -sy * tan_half_fov_y_;

        // Mount tilt about the body's right axis (pitch). Negative pitch looks down.
        const Vec3 tilted{fwd * cos_pitch_ + down * sin_pitch_, right, -fwd * sin_pitch_ + down * cos_pitch_};

        return normalize(core::quat_rotate(quat_, tilted));
    }

  private:
    static Vec3 normalize(const Vec3 &v) {
        const double n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (n <= 0.0) {
            return {1.0, 0.0, 0.0};
        }
        return {v[0] / n, v[1] / n, v[2] / n};
    }

    CameraConfig cfg_;
    Vec3 position_;
    Quat quat_;
    double tan_half_fov_x_{1.0};
    double tan_half_fov_y_{1.0};
    double cos_pitch_{1.0};
    double sin_pitch_{0.0};
};

} // namespace skysim::render
