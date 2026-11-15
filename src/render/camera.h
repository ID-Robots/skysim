#pragma once

#include "core/frames.h"

#include <cmath>
#include <cstdint>

namespace skysim::render {

using core::Quat;
using core::Vec3;

// Deliberately small by default. This is a situational view for a pilot flying by stick —
// "is there a building in front of me", not an inspection image. 256x144 at 10 Hz is what
// a handful of cores hold for a fleet while the simulation keeps its tick. Survey stills
// (render/still.h) use the same camera at whatever size they ask for.
struct CameraConfig {
    int width{256};
    int height{144};
    double fov_deg{78.0};      // horizontal, close to a typical drone camera
    double pitch_deg{-15.0};   // nose-down tilt of the mount, degrees (rigid-mount constructor)
    double max_range_m{350.0}; // draw distance for buildings; haze hides the cut
    // Where haze starts, as a fraction of max_range_m. 0 for the pilot view, whose haze is
    // what makes distance readable. A survey still sets it well out, because a point must
    // have the same colour seen from 80 m and from 200 m or photos stop matching.
    double haze_start{0.0};
};

// A pinhole camera: position plus an attitude that rotates the camera's own axes — x along
// the optical axis, y towards the image's right edge, z towards its bottom edge — into NED.
//
// Pixel (0, 0) is the top-left pixel, and pixel centres sit at half-integer coordinates, so
// the optical axis passes through (width / 2, height / 2) exactly. ray() and project() are
// inverses of each other; the rasteriser projects with the same numbers ray() unprojects.
class Camera {
  public:
    // Rigidly mounted on an airframe with this attitude (the NED->FRD quaternion the physics
    // uses), tilted cfg.pitch_deg about the body's right axis — the order a real mount sits in.
    Camera(const CameraConfig &cfg, const Vec3 &position_ned, const Quat &quat_ned_frd)
        : Camera(Attitude{}, cfg, position_ned,
                 core::quat_multiply(core::quat_normalize(quat_ned_frd), mount_pitch(cfg.pitch_deg))) {}

    // A camera whose attitude is given directly (a gimbal, a survey pose).
    static Camera with_attitude(const CameraConfig &cfg, const Vec3 &position_ned, const Quat &camera_quat) {
        return Camera(Attitude{}, cfg, position_ned, core::quat_normalize(camera_quat));
    }

    // On a self-stabilising gimbal (TASK-2548): the airframe's heading plus the gimbal's yaw,
    // the gimbal's earth-frame pitch, no roll — the airframe's lean does not reach the camera.
    // Radians.
    static Camera on_gimbal(const CameraConfig &cfg, const Vec3 &position_ned, const Quat &quat_ned_frd,
                            double yaw_rel_rad, double pitch_rad) {
        return with_attitude(cfg, position_ned, stabilised(quat_ned_frd, yaw_rel_rad, pitch_rad));
    }

    // Heading of an airframe attitude, radians clockwise from north.
    static double heading_rad(const Quat &quat_ned_frd) {
        const Quat q = core::quat_normalize(quat_ned_frd);
        return std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
    }

    // Yaw about down, then pitch about the resulting right axis, in NED. Radians.
    static Quat stabilised(const Quat &quat_ned_frd, double yaw_rel_rad, double pitch_rad) {
        const double yaw_half = (heading_rad(quat_ned_frd) + yaw_rel_rad) * 0.5;
        const Quat yaw{std::cos(yaw_half), 0.0, 0.0, std::sin(yaw_half)};
        const Quat pitch{std::cos(pitch_rad * 0.5), 0.0, std::sin(pitch_rad * 0.5), 0.0};
        return core::quat_multiply(yaw, pitch);
    }

    const CameraConfig &config() const { return cfg_; }
    const Vec3 &position() const { return position_; }
    const Quat &attitude() const { return quat_; }
    const Vec3 &forward() const { return fwd_; }
    const Vec3 &right() const { return right_; }
    const Vec3 &down() const { return down_; }
    double tan_half_x() const { return tan_half_x_; }
    double tan_half_y() const { return tan_half_y_; }
    // Pixels per unit of tangent, the same in x and y (square pixels).
    double focal_px() const { return cfg_.width * 0.5 / tan_half_x_; }

    // Unit ray direction in NED through the centre of pixel (x, y).
    Vec3 ray(int x, int y) const { return ray_at(x + 0.5, y + 0.5); }

    // Unit ray direction through a continuous image position.
    Vec3 ray_at(double px, double py) const {
        const double cx = (px / cfg_.width * 2.0 - 1.0) * tan_half_x_;
        const double cy = (py / cfg_.height * 2.0 - 1.0) * tan_half_y_;
        return normalize({fwd_[0] + right_[0] * cx + down_[0] * cy, fwd_[1] + right_[1] * cx + down_[1] * cy,
                          fwd_[2] + right_[2] * cx + down_[2] * cy});
    }

    // Image position of a world point; false when it is behind the camera.
    bool project(const Vec3 &p, double &px, double &py) const {
        const Vec3 v{p[0] - position_[0], p[1] - position_[1], p[2] - position_[2]};
        const double z = dot(v, fwd_);
        if (!(z > 1e-9)) {
            return false;
        }
        const double f = focal_px() / z;
        px = cfg_.width * 0.5 + dot(v, right_) * f;
        py = cfg_.height * 0.5 + dot(v, down_) * f;
        return true;
    }

    // How wide a pixel is on a surface: the square root of the area one pixel covers on the
    // plane through the hit point with this normal, `distance_m` along the unit ray `dir`.
    //
    // From the Jacobian of the ray-plane intersection, not from distance alone: on a plane
    // facing the camera squarely a pinhole's pixels all cover the same ground — a nadir frame
    // has one ground sample distance edge to edge — while a rule like distance / cos would
    // make the same spot of ground coarser at a frame's corner than at its centre, and the
    // texture filtered to it a different colour in the next, overlapping, photo.
    double footprint(const Vec3 &dir, double distance_m, const Vec3 &normal) const {
        const double along = dot(dir, fwd_);
        if (!(along > 1e-9)) {
            return 1e3;
        }
        // The un-normalised ray (unit forward component) and how far along it the hit is.
        const Vec3 r{dir[0] / along, dir[1] / along, dir[2] / along};
        const double s = distance_m * along / focal_px();
        const double nr = dot(normal, r);
        if (std::fabs(nr) < 1e-9) {
            return 1e3;
        }
        const double kr = dot(normal, right_) / nr;
        const double kd = dot(normal, down_) / nr;
        const Vec3 a{(right_[0] - r[0] * kr) * s, (right_[1] - r[1] * kr) * s, (right_[2] - r[2] * kr) * s};
        const Vec3 b{(down_[0] - r[0] * kd) * s, (down_[1] - r[1] * kd) * s, (down_[2] - r[2] * kd) * s};
        const Vec3 c{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
        return std::min(std::sqrt(std::sqrt(dot(c, c))), 1e3);
    }

    // Rotation about the body's right (y) axis; negative pitches the optical axis down.
    static Quat mount_pitch(double pitch_deg) {
        const double half = pitch_deg * M_PI / 360.0;
        return {std::cos(half), 0.0, std::sin(half), 0.0};
    }

  private:
    struct Attitude {}; // selects the constructor taking the camera's own attitude

    Camera(Attitude, const CameraConfig &cfg, const Vec3 &position_ned, const Quat &camera_quat)
        : cfg_(cfg), position_(position_ned), quat_(camera_quat) {
        const double aspect = static_cast<double>(cfg.width) / static_cast<double>(cfg.height);
        tan_half_x_ = std::tan((cfg.fov_deg * M_PI / 180.0) * 0.5);
        tan_half_y_ = tan_half_x_ / aspect;
        fwd_ = normalize(core::quat_rotate(quat_, {1.0, 0.0, 0.0}));
        right_ = normalize(core::quat_rotate(quat_, {0.0, 1.0, 0.0}));
        down_ = normalize(core::quat_rotate(quat_, {0.0, 0.0, 1.0}));
    }

    static double dot(const Vec3 &a, const Vec3 &b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

    static Vec3 normalize(const Vec3 &v) {
        const double n = std::sqrt(dot(v, v));
        if (n <= 0.0) {
            return {1.0, 0.0, 0.0};
        }
        return {v[0] / n, v[1] / n, v[2] / n};
    }

    CameraConfig cfg_;
    Vec3 position_;
    Quat quat_;
    Vec3 fwd_{1.0, 0.0, 0.0};
    Vec3 right_{0.0, 1.0, 0.0};
    Vec3 down_{0.0, 0.0, 1.0};
    double tan_half_x_{1.0};
    double tan_half_y_{1.0};
};

} // namespace skysim::render
