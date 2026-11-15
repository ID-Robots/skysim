#include "render/render_service.h"

#include "render/jpeg.h"

#include <chrono>
#include <cstdio>

namespace skysim::render {

RenderService::RenderService(const Config &cfg) : cfg_(cfg) {
    // The geometry is pulled out once. The static world does not change under this service —
    // it loads its own tiles and never streams — so the per-frame cost is transforming what
    // is in shot, not walking the world again.
    if (cfg_.scene) {
        scene_ = cfg_.scene;
        tiles_loaded_ = cfg_.scene_tiles;
    } else {
        scene_ = load_scene(cfg_.tiles_dir, cfg_.texture_seed, &tiles_loaded_);
    }

    rasterizer_ = std::make_unique<Rasterizer>(cfg_.camera);
    pool_ = std::make_unique<core::ThreadPool>(cfg_.threads);
    thread_ = std::thread([this] { run(); });
}

RenderService::~RenderService() {
    stop_.store(true, std::memory_order_relaxed);
    if (thread_.joinable()) {
        thread_.join();
    }
}

void RenderService::publish_poses(std::vector<Pose> poses) {
    std::lock_guard<std::mutex> lock(poses_mutex_);
    poses_ = std::move(poses);
}

void RenderService::run() {
    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::duration<double>(cfg_.fps > 0.0 ? 1.0 / cfg_.fps : 1.0);

    while (!stop_.load(std::memory_order_relaxed)) {
        const auto started = clock::now();

        std::vector<Pose> poses;
        {
            std::lock_guard<std::mutex> lock(poses_mutex_);
            poses = poses_;
        }

        // The pose list is the whole fleet, so anything not in it has despawned.
        // Dropping their frames here rather than hooking despawn keeps one owner
        // of the question "which vehicles exist" — this list, once a tick.
        std::vector<uint32_t> live;
        live.reserve(poses.size());
        for (const Pose &pose : poses) {
            live.push_back(pose.vehicle_id);
        }
        frames_.retain(live);

        for (const Pose &pose : poses) {
            if (stop_.load(std::memory_order_relaxed)) {
                break;
            }
            const Camera camera = pose.gimbal
                                      ? Camera::on_gimbal(cfg_.camera, pose.position_ned, pose.quat_ned_frd,
                                                          pose.gimbal_yaw_deg, pose.gimbal_pitch_deg)
                                      : Camera(cfg_.camera, pose.position_ned, pose.quat_ned_frd);
            const Image image = rasterizer_->render(*scene_, camera, pool_.get());
            auto jpeg = encode_jpeg(image, cfg_.quality);
            if (!jpeg.empty()) {
                frames_.publish(pose.vehicle_id, std::move(jpeg), pose.sim_time_s);
            }
        }

        // Pace to the target rate. If a pass took longer than the period the
        // sleep is skipped and the rate simply drops, which is the correct
        // failure: frames get older, nothing else is affected.
        const auto elapsed = clock::now() - started;
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

} // namespace skysim::render
