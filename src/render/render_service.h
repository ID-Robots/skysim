#pragma once

#include "core/thread_pool.h"
#include "core/world.h"
#include "render/camera.h"
#include "render/frame_store.h"
#include "render/raster.h"
#include "render/image.h"

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace skysim::render {

// Renders camera frames on its own thread, against its own copy of the world.
//
// The obvious design — render on the tick thread — cannot work, and the numbers
// say so plainly: one row of a 256-wide frame costs about 1.2 ms of raycasting,
// and the entire 800 Hz physics tick budget is 1.25 ms. Drawing even a single
// row per tick spends the whole budget, and the first attempt at this (four rows
// a tick) had skysim freezing and thawing the vehicle tens of thousands of times
// while ArduPilot refused to arm for "Main loop slow".
//
// So this owns a second World, built from the same tile directory. Buildings and
// ground are static geometry loaded from files, so a private copy is not a copy
// of anything that changes — and it can be raycast freely while the real world
// steps, with no lock between them and no way for a slow frame to slow a vehicle.
//
// The cost is that other vehicles do not appear in the picture. That is the whole
// trade: ground, sky and buildings, drawn without ever touching the simulation.
class RenderService {
  public:
    struct Config {
        CameraConfig camera;
        std::string tiles_dir;
        double fps{10.0};
        int quality{70};
        int threads{4};
    };

    // One vehicle's pose, as the tick thread last published it.
    struct Pose {
        uint32_t vehicle_id{0};
        core::Vec3 position_ned{};
        core::Quat quat_ned_frd{1.0, 0.0, 0.0, 0.0};
        // When the tick that produced this pose happened, carried through to the
        // frame so a consumer can tell a fresh picture from a stalled one.
        double sim_time_s{0.0};
    };

    explicit RenderService(const Config &cfg);
    ~RenderService();
    RenderService(const RenderService &) = delete;
    RenderService &operator=(const RenderService &) = delete;

    // Called from the tick thread once per tick. Cheap: copies a handful of
    // doubles under a mutex and returns.
    void publish_poses(std::vector<Pose> poses);

    // Latest encoded frame for a vehicle, with the sim time it was drawn at so a consumer can
    // tell a new picture from the one it already has. Empty until one has been drawn.
    FrameStore::Frame frame(uint32_t vehicle_id) const { return frames_.get(vehicle_id); }

    size_t tiles_loaded() const { return tiles_loaded_; }
    size_t triangle_count() const { return scene_.triangle_count(); }

  private:
    void run();

    Config cfg_;
    std::unique_ptr<core::World> world_;
    std::unique_ptr<Rasterizer> rasterizer_;
    RasterScene scene_;
    std::unique_ptr<core::ThreadPool> pool_;
    FrameStore frames_;
    size_t tiles_loaded_{0};

    mutable std::mutex poses_mutex_;
    std::vector<Pose> poses_;

    std::atomic<bool> stop_{false};
    std::thread thread_;
};

} // namespace skysim::render
