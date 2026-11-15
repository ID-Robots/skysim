#include "render/render_service.h"

#include "render/jpeg.h"
#include "render/still.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace skysim::render {
namespace {

// The same pose as the one a frame was drawn from, so the frame already encoded for it can be
// sent again instead of drawn again: a vehicle parked on the ground waiting for its autopilot —
// the commonest thing a dev server's camera looks at — then costs next to nothing to watch.
//
// "The same" to within float noise, not bit for bit: a vehicle resting on the ground is never
// put to sleep (World disables it), and the contact solver moves it by about 1e-13 m between
// ticks, which made two frames in three of a parked vehicle "new". A nanometre is a few
// millionths of a pixel even at the nearest ground the camera sees (0.1 m off a parked quad's
// camera, 640 px wide), so nothing a viewer could see is skipped. The comparison is always
// against the pose last drawn, so a slow creep adds up and gets drawn.
bool same_pose(const RenderService::Pose &a, const RenderService::Pose &b) {
    constexpr double kPositionM = 1e-9;
    constexpr double kQuat = 1e-12;
    for (size_t k = 0; k < 3; ++k) {
        if (!(std::fabs(a.position_ned[k] - b.position_ned[k]) <= kPositionM)) {
            return false;
        }
    }
    for (size_t k = 0; k < 4; ++k) {
        if (!(std::fabs(a.quat_ned_frd[k] - b.quat_ned_frd[k]) <= kQuat)) {
            return false;
        }
    }
    return true;
}

double ms_between(std::chrono::steady_clock::time_point a, std::chrono::steady_clock::time_point b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
}

// Stats are reported over windows of this length, so /metrics follows a change within seconds.
constexpr auto kStatsWindow = std::chrono::seconds(2);

} // namespace

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

    threads_ = std::max(cfg_.threads, 1);
    rasterizer_ = std::make_unique<Rasterizer>(cfg_.camera);
    // The service's own thread draws a share of every frame, so a pool of threads - 1 workers
    // puts exactly `threads` CPUs on it — not threads + 1, which is what a pool of `threads`
    // did, five render threads on a 2-vCPU task by default.
    if (threads_ > 1) {
        const bool background = cfg_.background_priority;
        pool_ = std::make_unique<core::ThreadPool>(threads_ - 1, [background] {
            if (background) {
                lower_thread_priority();
            }
        });
    }
    stats_.target_fps = cfg_.fps;
    stats_.threads = threads_;
    thread_ = std::thread([this] { run(); });
}

RenderService::~RenderService() {
    {
        std::lock_guard<std::mutex> lock(demand_mutex_);
        stop_.store(true, std::memory_order_relaxed);
    }
    wake_.notify_all();
    if (thread_.joinable()) {
        thread_.join();
    }
}

void RenderService::publish_poses(std::vector<Pose> poses) {
    std::lock_guard<std::mutex> lock(poses_mutex_);
    poses_ = std::move(poses);
}

FrameStore::Frame RenderService::frame(uint32_t vehicle_id, uint64_t after_seq, std::chrono::milliseconds wait) {
    const auto now = Clock::now();
    bool fresh_demand = false;
    {
        std::lock_guard<std::mutex> lock(demand_mutex_);
        auto [it, inserted] = demand_.try_emplace(vehicle_id, now);
        it->second = now;
        if (inserted) {
            ++demand_epoch_;
            fresh_demand = true;
        }
    }
    if (fresh_demand) {
        wake_.notify_all();
    }
    if (wait.count() <= 0) {
        return frames_.get(vehicle_id);
    }
    bool in_fleet = false;
    {
        std::lock_guard<std::mutex> lock(poses_mutex_);
        in_fleet = std::any_of(poses_.begin(), poses_.end(),
                               [vehicle_id](const Pose &p) { return p.vehicle_id == vehicle_id; });
    }
    if (!in_fleet) {
        return frames_.get(vehicle_id);
    }
    return frames_.wait_newer(vehicle_id, after_seq, now + wait);
}

RenderService::Stats RenderService::stats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

void RenderService::record_pass(const Window &pass, Clock::time_point now) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.frames_drawn += pass.drawn;
    stats_.frames_reused += pass.reused;
    stats_.watched = pass.watched;
    for (Window *w : {&stats_window_, &log_window_}) {
        if (w->passes == 0) {
            w->started = pass.started;
        }
        w->passes += pass.passes;
        w->drawn += pass.drawn;
        w->reused += pass.reused;
        w->render_ms_sum += pass.render_ms_sum;
        w->render_ms_max = std::max(w->render_ms_max, pass.render_ms_max);
        w->encode_ms_sum += pass.encode_ms_sum;
        w->watched = pass.watched;
        w->last_started = pass.started;
    }
    if (now - stats_window_.started >= kStatsWindow) {
        const Window &w = stats_window_;
        // Passes per second between the first pass's start and the last one's: every watched
        // vehicle gets one frame a pass, so this is the rate each stream sees.
        const double span_s = std::chrono::duration<double>(w.last_started - w.started).count();
        stats_.fps = w.passes > 1 && span_s > 0.0 ? static_cast<double>(w.passes - 1) / span_s : 0.0;
        if (w.drawn > 0) {
            stats_.render_ms = w.render_ms_sum / static_cast<double>(w.drawn);
            stats_.render_ms_max = w.render_ms_max;
            stats_.encode_ms = w.encode_ms_sum / static_cast<double>(w.drawn);
        }
        stats_window_ = Window{};
    }
    if (cfg_.log_every_s > 0.0 &&
        now - log_window_.started >= std::chrono::duration<double>(cfg_.log_every_s)) {
        log_window(log_window_);
        last_log_ = now;
        log_window_ = Window{};
    }
}

void RenderService::go_idle(Clock::time_point now) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    // Nothing is being drawn now. The cost of the last frames stays readable, the rate does not.
    stats_.fps = 0.0;
    stats_.watched = 0;
    stats_window_ = Window{};
    // At most one camera line per log period: a viewer polling camera.jpg every few seconds
    // takes the camera in and out of idle each time, and must not fill the log doing it.
    if (cfg_.log_every_s > 0.0 && now - last_log_ >= std::chrono::duration<double>(cfg_.log_every_s)) {
        last_log_ = now;
        if (log_window_.passes > 0) {
            log_window(log_window_);
        }
        std::printf("skysim: camera: idle, nobody watching (%llu frame(s) drawn, %llu re-sent so far)\n",
                    static_cast<unsigned long long>(stats_.frames_drawn),
                    static_cast<unsigned long long>(stats_.frames_reused));
        std::fflush(stdout);
    }
    log_window_ = Window{};
}

void RenderService::log_window(const Window &w) const {
    const double span_s = std::chrono::duration<double>(w.last_started - w.started).count();
    const double fps = w.passes > 1 && span_s > 0.0 ? static_cast<double>(w.passes - 1) / span_s : 0.0;
    const double drawn = static_cast<double>(std::max<uint64_t>(w.drawn, 1));
    std::printf("skysim: camera: %zu vehicle(s) watched, %.1f fps each (target %.1f), render %.1f ms/frame "
                "(max %.1f), encode %.1f ms, %llu drawn + %llu re-sent, %d thread(s)\n",
                w.watched, fps, cfg_.fps, w.render_ms_sum / drawn, w.render_ms_max, w.encode_ms_sum / drawn,
                static_cast<unsigned long long>(w.drawn), static_cast<unsigned long long>(w.reused), threads_);
    std::fflush(stdout);
}

void RenderService::run() {
    if (cfg_.background_priority) {
        lower_thread_priority();
    }
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(cfg_.fps > 0.0 ? 1.0 / cfg_.fps : 1.0));
    const auto window = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(std::max(cfg_.demand_window_s, 0.0)));

    // The last frame drawn per vehicle and the pose it was drawn from.
    struct Drawn {
        Pose pose;
        std::vector<uint8_t> jpeg;
    };
    std::unordered_map<uint32_t, Drawn> last;
    std::vector<uint32_t> live;
    std::vector<uint32_t> lapsed;
    std::vector<const Pose *> wanted;
    bool drawing = false;

    while (!stop_.load(std::memory_order_relaxed)) {
        const auto started = Clock::now();

        std::vector<Pose> poses;
        {
            std::lock_guard<std::mutex> lock(poses_mutex_);
            poses = poses_;
        }

        // The pose list is the whole fleet, so anything not in it has despawned.
        // Dropping their frames here rather than hooking despawn keeps one owner
        // of the question "which vehicles exist" — this list, once a tick.
        live.clear();
        for (const Pose &pose : poses) {
            live.push_back(pose.vehicle_id);
        }
        frames_.retain(live);
        for (auto it = last.begin(); it != last.end();) {
            it = std::find(live.begin(), live.end(), it->first) == live.end() ? last.erase(it) : std::next(it);
        }

        // Who is being watched. A vehicle nobody has asked about for a demand window leaves
        // the list and its frame goes with it.
        uint64_t seen_epoch = 0;
        lapsed.clear();
        wanted.clear();
        {
            std::lock_guard<std::mutex> lock(demand_mutex_);
            for (auto it = demand_.begin(); it != demand_.end();) {
                if (started - it->second >= window) {
                    lapsed.push_back(it->first);
                    it = demand_.erase(it);
                } else {
                    ++it;
                }
            }
            for (const Pose &pose : poses) {
                if (demand_.count(pose.vehicle_id) != 0) {
                    wanted.push_back(&pose);
                }
            }
            seen_epoch = demand_epoch_;
        }
        for (const uint32_t id : lapsed) {
            frames_.erase(id);
            last.erase(id);
        }

        if (wanted.empty()) {
            if (drawing) {
                drawing = false;
                go_idle(started);
            }
            // Nothing to draw: sleep until somebody asks. With nobody asking at all that is an
            // untimed wait — the whole idle cost of the camera. A request for a vehicle that is
            // not in the fleet (yet: a stream opened before its spawn) looks again at the camera
            // rate until it lapses.
            std::unique_lock<std::mutex> lock(demand_mutex_);
            const auto woken = [&] { return stop_.load(std::memory_order_relaxed) || demand_epoch_ != seen_epoch; };
            if (demand_.empty()) {
                wake_.wait(lock, woken);
            } else {
                wake_.wait_for(lock, period, woken);
            }
            continue;
        }
        drawing = true;

        Window pass;
        pass.started = started;
        pass.watched = wanted.size();
        for (const Pose *pose : wanted) {
            if (stop_.load(std::memory_order_relaxed)) {
                break;
            }
            Drawn &d = last[pose->vehicle_id];
            if (!d.jpeg.empty() && same_pose(d.pose, *pose)) {
                frames_.publish(pose->vehicle_id, d.jpeg, pose->sim_time_s);
                ++pass.reused;
                continue;
            }
            const auto t0 = Clock::now();
            const Camera camera(cfg_.camera, pose->position_ned, pose->quat_ned_frd);
            const Image image = rasterizer_->render(*scene_, camera, pool_.get());
            const auto t1 = Clock::now();
            std::vector<uint8_t> jpeg = encode_jpeg(image, cfg_.quality);
            const auto t2 = Clock::now();
            const double render_ms = ms_between(t0, t1);
            ++pass.drawn;
            pass.render_ms_sum += render_ms;
            pass.render_ms_max = std::max(pass.render_ms_max, render_ms);
            pass.encode_ms_sum += ms_between(t1, t2);
            if (!jpeg.empty()) {
                d.pose = *pose;
                d.jpeg = jpeg;
                frames_.publish(pose->vehicle_id, std::move(jpeg), pose->sim_time_s);
            }
        }
        pass.passes = 1;
        record_pass(pass, Clock::now());

        // Pace to the target rate. If a pass took longer than the period there is no wait and
        // the rate simply drops, which is the correct failure: frames get older, nothing else
        // is affected (these threads run at background priority).
        std::unique_lock<std::mutex> lock(demand_mutex_);
        wake_.wait_until(lock, started + period, [this] { return stop_.load(std::memory_order_relaxed); });
    }
}

} // namespace skysim::render
