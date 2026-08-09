#include "render/render_service.h"

#include "render/jpeg.h"

#include <chrono>
#include <cstdio>

namespace skysim::render {

RenderService::RenderService(const Config &cfg) : cfg_(cfg) {
    core::WorldConfig wcfg;
    // This world is never stepped, so its timestep is irrelevant; it exists only
    // to hold static geometry that rays can be cast at.
    wcfg.dt_s = 1.0 / 800.0;
    wcfg.worker_threads = 0;
    world_ = std::make_unique<core::World>(wcfg);
    world_->add_ground_plane();
    if (!cfg_.tiles_dir.empty()) {
        tiles_loaded_ = world_->load_tiles(cfg_.tiles_dir);
    }

    // Pull the geometry out once. The static world does not change under this
    // service — it loads its own tiles and never streams — so the per-frame cost
    // is transforming what is in shot, not walking the world again.
    scene_.build(world_->collect_static_triangles());

    // The physics world was scaffolding: it parsed the tiles so the triangles
    // could be lifted out. Nothing raycasts it once the rasteriser has the
    // geometry, and keeping a second Jolt world alive costs ~10 MB for the life
    // of the process.
    world_.reset();

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
            const Camera camera(cfg_.camera, pose.position_ned, pose.quat_ned_frd);
            const Image image = rasterizer_->render(scene_, camera, pool_.get());
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
