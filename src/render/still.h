#pragma once

#include "core/thread_pool.h"
#include "render/camera.h"
#include "render/dock_marker.h"
#include "render/image.h"
#include "render/scene.h"
#include "render/still_request.h"

#include <memory>
#include <mutex>
#include <optional>
#include <vector>

namespace skysim::render {

// The survey still camera: one frame, rendered on demand at whatever size, field of view and
// gimbal angle a caller asks for, from a vehicle's pose as one tick published it.
//
// The pilot camera (RenderService) streams small frames at a fixed rate whether anyone looks
// or not; a survey wants the opposite — a large, sharp, exactly-posed frame at the moment a
// shutter fires, and nothing in between. Both draw the same scene with the same shading.

// Heading of an FRD->NED attitude, degrees clockwise from north in [0, 360).
double heading_deg(const Quat &quat_ned_frd);

// Camera attitude (camera axes -> NED) from Z-Y-X Euler angles in degrees.
Quat camera_quat_from_angles(double yaw_deg, double pitch_deg, double roll_deg);

// ... and back. Defined at pitch +-90 too, where yaw and roll are one degree of freedom: roll
// is reported 0 and yaw is the direction the top edge of the image faces.
CameraAngles angles_from_camera_quat(const Quat &camera_quat);

// The still camera's attitude for a vehicle attitude and a request. Stabilised (roll_stab):
// yaw = vehicle heading + request yaw, pitch as asked, roll 0, whatever the airframe is
// doing. Rigid: the airframe's attitude, then the mount's yaw and pitch on top of it.
Quat still_camera_attitude(const Quat &quat_ned_frd, const StillRequest &req, CameraAngles &angles);

CameraConfig still_camera_config(const StillRequest &req);

// The camera attitude a still is drawn with (TASK-2548), and its axes in ``angles`` (degrees,
// for the response headers): the request's (still_camera_attitude), unless it asks to follow
// the gimbal and the vehicle has one — then the heading plus the gimbal's yaw at the gimbal's
// pitch, stabilised, computed in radians (Camera::stabilised). ``followed`` says which.
Quat still_attitude(const StillPose &pose, const StillRequest &req, CameraAngles &angles, bool &followed);

// Draw one still from an explicit camera pose. caller_runs and markers (the dock markers on
// the ground) as in Rasterizer::render.
Image render_still(const RasterScene &scene, const Vec3 &position_ned, const Quat &camera_quat,
                   const StillRequest &req, core::ThreadPool *pool, bool caller_runs = true,
                   std::vector<int32_t> *surface = nullptr, const GroundMarkers *markers = nullptr);

// Lower the calling thread's scheduling priority (nice +10). Background render workers call
// it so that, on a box short of cores, the physics tick wins every contest for a CPU.
// Returns whether it took effect (Linux only).
bool lower_thread_priority();

// Renders stills for the control plane, at most `concurrency` at a time.
//
// Every slot owns a worker pool, created up front: a request never waits for threads to
// start, and two renders never share a pool (ThreadPool::parallel_for is single-caller). A
// request that finds every slot taken is answered busy at once rather than queued — a queue
// would hold an HTTP worker and the caller's socket for as long as the backlog takes, and
// the caller is better placed to decide whether to retry.
//
// The calling thread does no rendering: the frame and its JPEG are made on the slot's
// workers, which run at background priority (lower_thread_priority) when configured to.
class StillService {
  public:
    struct Config {
        int concurrency{2}; // SKYSIM_STILL_CONCURRENCY; 0 turns the still camera off
        int threads{2};     // workers per render
        bool background_priority{true};
    };

    // A claim on one render slot, released when it goes out of scope.
    class Lease {
      public:
        Lease(Lease &&other) noexcept : service_(other.service_), slot_(other.slot_) {
            other.service_ = nullptr;
        }
        Lease(const Lease &) = delete;
        Lease &operator=(const Lease &) = delete;
        Lease &operator=(Lease &&) = delete;
        ~Lease();
        int slot() const { return slot_; }

      private:
        friend class StillService;
        Lease(StillService *service, int slot) : service_(service), slot_(slot) {}
        StillService *service_;
        int slot_;
    };

    StillService(std::shared_ptr<const RasterScene> scene, const Config &cfg);

    int concurrency() const { return cfg_.concurrency; }
    int in_flight() const;
    const RasterScene &scene() const { return *scene_; }

    // A free slot, or nothing when all are taken.
    std::optional<Lease> try_acquire();

    // Takes a slot for the length of the render; kBusy when there is none. `markers`: the dock
    // markers on the ground (render/dock_marker.h), drawn like the pilot camera draws them — for a
    // vehicle's still, the docks its own cameras see (DockBoard::seen_by).
    StillFrame render(const StillPose &pose, const StillRequest &req, const GroundMarkers *markers = nullptr);
    StillFrame render(Lease &lease, const StillPose &pose, const StillRequest &req,
                      const GroundMarkers *markers = nullptr);

  private:
    void release(int slot);

    std::shared_ptr<const RasterScene> scene_;
    Config cfg_;
    std::vector<std::unique_ptr<core::ThreadPool>> pools_;
    mutable std::mutex mutex_;
    std::vector<bool> busy_;
};

} // namespace skysim::render
