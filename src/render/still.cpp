#include "render/still.h"

#include "render/jpeg.h"
#include "render/raster.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <exception>
#include <string>

#if defined(__linux__)
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace skysim::render {
namespace {

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

double wrap360(double deg) {
    double d = std::fmod(deg, 360.0);
    if (d < 0.0) {
        d += 360.0;
    }
    return d >= 360.0 ? 0.0 : d + 0.0; // + 0.0 turns -0 into 0
}

Quat axis_quat(int axis, double deg) {
    const double half = deg * kDegToRad * 0.5;
    Quat q{std::cos(half), 0.0, 0.0, 0.0};
    q[static_cast<size_t>(axis + 1)] = std::sin(half);
    return q;
}

} // namespace

double heading_deg(const Quat &quat_ned_frd) {
    const Quat q = core::quat_normalize(quat_ned_frd);
    return wrap360(std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])) *
                   kRadToDeg);
}

Quat camera_quat_from_angles(double yaw_deg, double pitch_deg, double roll_deg) {
    return core::quat_multiply(core::quat_multiply(axis_quat(2, yaw_deg), axis_quat(1, pitch_deg)),
                               axis_quat(0, roll_deg));
}

CameraAngles angles_from_camera_quat(const Quat &camera_quat) {
    const Quat q = core::quat_normalize(camera_quat);
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const double sinp = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
    CameraAngles a;
    a.pitch_deg = std::asin(sinp) * kRadToDeg;
    // Only at the singularity itself (within ~1e-6 rad of it): anywhere else the regular
    // conversion is well defined in doubles, and taking this branch early would report
    // angles for a different optical axis.
    if (std::fabs(sinp) > 1.0 - 1e-12) {
        // Looking straight down (or up): yaw is where the top of the image points.
        const Vec3 down = core::quat_rotate(q, {0.0, 0.0, 1.0});
        const double sign = sinp < 0.0 ? -1.0 : 1.0;
        a.yaw_deg = wrap360(std::atan2(sign * down[1], sign * down[0]) * kRadToDeg);
        a.roll_deg = 0.0;
        return a;
    }
    a.yaw_deg = wrap360(std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) * kRadToDeg);
    a.roll_deg = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)) * kRadToDeg;
    return a;
}

Quat still_camera_attitude(const Quat &quat_ned_frd, const StillRequest &req, CameraAngles &angles) {
    if (req.roll_stab) {
        angles.yaw_deg = wrap360(heading_deg(quat_ned_frd) + req.yaw_deg);
        angles.pitch_deg = req.pitch_deg;
        angles.roll_deg = 0.0;
        return camera_quat_from_angles(angles.yaw_deg, angles.pitch_deg, 0.0);
    }
    const Quat mount = core::quat_multiply(axis_quat(2, req.yaw_deg), axis_quat(1, req.pitch_deg));
    const Quat q = core::quat_multiply(core::quat_normalize(quat_ned_frd), mount);
    angles = angles_from_camera_quat(q);
    return q;
}

Quat still_attitude(const StillPose &pose, const StillRequest &req, CameraAngles &angles, bool &followed) {
    followed = req.follow_gimbal && pose.gimbal;
    if (!followed) {
        return still_camera_attitude(pose.quat_ned_frd, req, angles);
    }
    const Quat q = Camera::stabilised(pose.quat_ned_frd, pose.gimbal_yaw_rad, pose.gimbal_pitch_rad);
    // Degrees only for the headers. No roll by construction: say exactly 0, not a rounding of it.
    angles = angles_from_camera_quat(q);
    angles.roll_deg = 0.0;
    return q;
}

CameraConfig still_camera_config(const StillRequest &req) {
    CameraConfig cfg;
    cfg.width = req.width;
    cfg.height = req.height;
    cfg.fov_deg = req.hfov_deg;
    cfg.pitch_deg = req.pitch_deg;
    cfg.max_range_m = req.range_m;
    // Clear air out to half the draw distance: a survey frame must give a point the same
    // colour seen from 80 m overhead and 200 m off at an angle.
    cfg.haze_start = 0.5;
    return cfg;
}

Image render_still(const RasterScene &scene, const Vec3 &position_ned, const Quat &camera_quat,
                   const StillRequest &req, core::ThreadPool *pool, bool caller_runs,
                   std::vector<int32_t> *surface) {
    const CameraConfig cfg = still_camera_config(req);
    const Camera camera = Camera::with_attitude(cfg, position_ned, camera_quat);
    return Rasterizer(cfg).render(scene, camera, pool, surface, caller_runs);
}

bool lower_thread_priority() {
#if defined(__linux__)
    // Linux keeps the nice value per thread, and this thread's id is what names it.
    const auto tid = static_cast<id_t>(syscall(SYS_gettid));
    errno = 0;
    const int current = getpriority(PRIO_PROCESS, tid);
    if (errno != 0) {
        return false;
    }
    return setpriority(PRIO_PROCESS, tid, std::min(current + 10, 19)) == 0;
#else
    return false;
#endif
}

StillService::Lease::~Lease() {
    if (service_ != nullptr) {
        service_->release(slot_);
    }
}

StillService::StillService(std::shared_ptr<const RasterScene> scene, const Config &cfg)
    : scene_(std::move(scene)), cfg_(cfg) {
    cfg_.concurrency = std::max(cfg_.concurrency, 0);
    cfg_.threads = std::max(cfg_.threads, 1);
    if (!scene_) {
        cfg_.concurrency = 0;
    }
    busy_.assign(static_cast<size_t>(cfg_.concurrency), false);
    const bool background = cfg_.background_priority;
    for (int i = 0; i < cfg_.concurrency; ++i) {
        pools_.push_back(std::make_unique<core::ThreadPool>(cfg_.threads, [background] {
            if (background) {
                lower_thread_priority();
            }
        }));
    }
}

int StillService::in_flight() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<int>(std::count(busy_.begin(), busy_.end(), true));
}

std::optional<StillService::Lease> StillService::try_acquire() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < busy_.size(); ++i) {
        if (!busy_[i]) {
            busy_[i] = true;
            return Lease(this, static_cast<int>(i));
        }
    }
    return std::nullopt;
}

void StillService::release(int slot) {
    std::lock_guard<std::mutex> lock(mutex_);
    busy_[static_cast<size_t>(slot)] = false;
}

StillFrame StillService::render(const StillPose &pose, const StillRequest &req) {
    if (cfg_.concurrency == 0) {
        StillFrame frame;
        frame.status = StillFrame::Status::kDisabled;
        frame.error = "still camera disabled";
        return frame;
    }
    std::optional<Lease> lease = try_acquire();
    if (!lease) {
        StillFrame frame;
        frame.status = StillFrame::Status::kBusy;
        frame.error = "busy: " + std::to_string(cfg_.concurrency) + " stills already rendering; retry";
        return frame;
    }
    return render(*lease, pose, req);
}

StillFrame StillService::render(Lease &lease, const StillPose &pose, const StillRequest &req) {
    StillFrame frame;
    frame.error = req.invalid();
    if (!frame.error.empty()) {
        frame.status = StillFrame::Status::kInvalid;
        return frame;
    }
    // Validated as asked; drawn where the gimbal points when it was asked to follow one. The
    // gimbal's own pitch is bounded by its range (vehicle/gimbal.h), not by the request's.
    const Quat camera = still_attitude(pose, req, frame.camera, frame.gimbal);
    core::ThreadPool *pool = pools_[static_cast<size_t>(lease.slot())].get();
    try {
        const Image image =
            render_still(*scene_, pose.position_ned, camera, req, pool, /*caller_runs=*/false);
        // The encode is the other big cost of a frame; it runs on a worker too.
        pool->parallel_for(
            1, [&](size_t, size_t) { frame.jpeg = encode_jpeg(image, req.quality); }, /*caller_runs=*/false);
    } catch (const std::exception &e) {
        // A 4096 x 4096 frame on a box short of memory: a failed request, not a dead server.
        frame.jpeg.clear();
        frame.error = std::string("render failed: ") + e.what();
    }
    if (frame.jpeg.empty()) {
        frame.status = StillFrame::Status::kFailed;
        if (frame.error.empty()) {
            frame.error = "render failed: encoder produced nothing";
        }
    } else {
        frame.status = StillFrame::Status::kOk;
    }
    return frame;
}

} // namespace skysim::render
