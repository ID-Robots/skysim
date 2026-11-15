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
    // A gimbal that moves while the vehicle holds still is a different picture (TASK-2548).
    constexpr double kGimbalRad = 1e-9;
    return a.gimbal == b.gimbal && std::fabs(a.gimbal_pitch_rad - b.gimbal_pitch_rad) <= kGimbalRad &&
           std::fabs(a.gimbal_yaw_rad - b.gimbal_yaw_rad) <= kGimbalRad;
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

bool RenderService::in_fleet(uint32_t vehicle_id) const {
    std::lock_guard<std::mutex> lock(poses_mutex_);
    return std::any_of(poses_.begin(), poses_.end(),
                       [vehicle_id](const Pose &p) { return p.vehicle_id == vehicle_id; });
}

void RenderService::note_demand(std::unordered_map<uint32_t, Clock::time_point> &demand, uint32_t vehicle_id,
                                Clock::time_point now) {
    bool fresh_demand = false;
    {
        std::lock_guard<std::mutex> lock(demand_mutex_);
        auto [it, inserted] = demand.try_emplace(vehicle_id, now);
        it->second = now;
        if (inserted) {
            ++demand_epoch_;
            fresh_demand = true;
        }
    }
    if (fresh_demand) {
        wake_.notify_all();
    }
}

bool RenderService::want_video(uint32_t vehicle_id) {
    if (!in_fleet(vehicle_id)) {
        return false;
    }
    note_demand(video_demand_, vehicle_id, Clock::now());
    return true;
}

FrameStore::Frame RenderService::frame(uint32_t vehicle_id, uint64_t after_seq,
                                       std::chrono::milliseconds wait) {
    const auto now = Clock::now();
    // Only a vehicle in the fleet goes on the draw list. Anything else would never be drawn,
    // and recording it would let requests for made-up ids grow the list and wake the render
    // thread at whatever rate they arrive. A stream opened before its vehicle spawns keeps
    // asking, so it is on the list within one wait of the spawn.
    if (!in_fleet(vehicle_id)) {
        return frames_.get(vehicle_id);
    }
    note_demand(demand_, vehicle_id, now);
    if (wait.count() <= 0) {
        return frames_.get(vehicle_id);
    }
    return frames_.wait_newer(vehicle_id, after_seq, now + wait);
}

RenderService::Stats RenderService::stats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

double RenderService::Window::fps() const {
    return active_s > 0.0 ? static_cast<double>(intervals) / active_s : 0.0;
}

void RenderService::record_pass(const Window &pass, Clock::time_point now) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.frames_drawn += pass.drawn;
    stats_.frames_reused += pass.reused;
    stats_.watched = pass.watched;
    // The rate is passes per second of drawing: the time from one pass's start to the next,
    // within a run of passes. Idle gaps between runs are not part of it, so a viewer that comes
    // and goes is reported at the rate it saw, not diluted by the time nobody was looking.
    const double step_s = in_burst_ ? std::chrono::duration<double>(pass.started - burst_prev_).count() : 0.0;
    for (Window *w : {&stats_window_, &log_window_}) {
        if (w->passes == 0) {
            w->started = pass.started;
        }
        w->passes += pass.passes;
        if (in_burst_) {
            ++w->intervals;
            w->active_s += step_s;
        }
        w->drawn += pass.drawn;
        w->reused += pass.reused;
        w->render_ms_sum += pass.render_ms_sum;
        w->render_ms_max = std::max(w->render_ms_max, pass.render_ms_max);
        w->encode_ms_sum += pass.encode_ms_sum;
        w->encoded += pass.encoded;
        w->watched = std::max(w->watched, pass.watched);
    }
    in_burst_ = true;
    burst_prev_ = pass.started;
    idle_line_pending_ = false;
    if (now - stats_window_.started >= kStatsWindow) {
        const Window &w = stats_window_;
        // Every watched vehicle gets one frame a pass, so this is the rate each stream sees.
        stats_.fps = w.fps();
        if (w.drawn > 0) {
            stats_.render_ms = w.render_ms_sum / static_cast<double>(w.drawn);
            stats_.render_ms_max = w.render_ms_max;
        }
        if (w.encoded > 0) {
            stats_.encode_ms = w.encode_ms_sum / static_cast<double>(w.encoded);
        }
        stats_window_ = Window{};
    }
    if (cfg_.log_every_s > 0.0 && now - log_window_.started >= log_period() &&
        now - last_log_ >= log_period()) {
        log_window(log_window_);
        last_log_ = now;
        log_window_ = Window{};
    }
}

RenderService::Clock::duration RenderService::log_period() const {
    return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(cfg_.log_every_s));
}

void RenderService::go_idle() {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    // Nothing is being drawn now. The cost of the last frames stays readable, the rate does not.
    stats_.fps = 0.0;
    stats_.watched = 0;
    stats_window_ = Window{};
    in_burst_ = false;
    // The frames drawn since the last log line, and the fact that drawing stopped, are logged
    // by flush_idle_log once a log period has passed since that line — at most one camera line
    // per period, however often a viewer polling camera.jpg takes the camera in and out of idle.
    idle_line_pending_ = cfg_.log_every_s > 0.0;
}

RenderService::Clock::time_point RenderService::idle_log_due() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return idle_line_pending_ ? last_log_ + log_period() : Clock::time_point::max();
}

void RenderService::flush_idle_log(Clock::time_point now) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    if (!idle_line_pending_) {
        return;
    }
    idle_line_pending_ = false;
    last_log_ = now;
    if (log_window_.passes > 0) {
        log_window(log_window_);
    }
    log_window_ = Window{};
    char line[160];
    std::snprintf(line, sizeof(line),
                  "skysim: camera: idle, nobody watching (%llu frame(s) drawn, %llu re-sent so far)",
                  static_cast<unsigned long long>(stats_.frames_drawn),
                  static_cast<unsigned long long>(stats_.frames_reused));
    emit(line);
}

void RenderService::emit(const char *line) const {
    if (cfg_.log_sink) {
        cfg_.log_sink(line);
        return;
    }
    std::printf("%s\n", line);
    std::fflush(stdout);
}

void RenderService::log_window(const Window &w) const {
    const double drawn = static_cast<double>(std::max<uint64_t>(w.drawn, 1));
    const double encoded = static_cast<double>(std::max<uint64_t>(w.encoded, 1));
    char line[256];
    std::snprintf(line, sizeof(line),
                  "skysim: camera: %zu vehicle(s) watched, %.1f fps each (target %.1f), render %.1f ms/frame "
                  "(max %.1f), encode %.1f ms, %llu drawn + %llu re-sent, %d thread(s)",
                  w.watched, w.fps(), cfg_.fps, w.render_ms_sum / drawn, w.render_ms_max,
                  w.encode_ms_sum / encoded, static_cast<unsigned long long>(w.drawn),
                  static_cast<unsigned long long>(w.reused), threads_);
    emit(line);
}

void RenderService::run() {
    if (cfg_.background_priority) {
        lower_thread_priority();
    }
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(cfg_.fps > 0.0 ? 1.0 / cfg_.fps : 1.0));
    const auto window = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(std::max(cfg_.demand_window_s, 0.0)));

    // The last picture drawn per vehicle, the pose it was drawn from, and its JPEG once one has
    // been made (only a JPEG viewer needs it).
    struct Drawn {
        Pose pose;
        std::shared_ptr<const Image> image;
        std::vector<uint8_t> jpeg;
    };
    // A vehicle on this pass's draw list, and who wants it.
    struct Wanted {
        const Pose *pose;
        bool jpeg;
        bool video;
    };
    std::unordered_map<uint32_t, Drawn> last;
    std::vector<uint32_t> live;
    std::vector<uint32_t> lapsed;
    std::vector<uint32_t> lapsed_video;
    std::vector<Wanted> wanted;
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
        lapsed_video.clear();
        wanted.clear();
        bool any_demand = false;
        {
            std::lock_guard<std::mutex> lock(demand_mutex_);
            const auto expire = [&](std::unordered_map<uint32_t, Clock::time_point> &demand,
                                    std::vector<uint32_t> &out) {
                for (auto it = demand.begin(); it != demand.end();) {
                    if (started - it->second >= window) {
                        out.push_back(it->first);
                        it = demand.erase(it);
                    } else {
                        ++it;
                    }
                }
            };
            expire(demand_, lapsed);
            expire(video_demand_, lapsed_video);
            for (const Pose &pose : poses) {
                const bool jpeg = demand_.count(pose.vehicle_id) != 0;
                const bool video = video_demand_.count(pose.vehicle_id) != 0;
                if (jpeg || video) {
                    wanted.push_back({&pose, jpeg, video});
                }
            }
            any_demand = !demand_.empty() || !video_demand_.empty();
            seen_epoch = demand_epoch_;
            // A lapsed JPEG demand takes the vehicle's JPEG with it; the picture goes once
            // nobody wants the vehicle at all.
            for (const uint32_t id : lapsed) {
                if (video_demand_.count(id) == 0) {
                    last.erase(id);
                }
            }
            for (const uint32_t id : lapsed_video) {
                if (demand_.count(id) == 0) {
                    last.erase(id);
                }
            }
        }
        for (const uint32_t id : lapsed) {
            frames_.erase(id);
            auto it = last.find(id);
            if (it != last.end()) {
                it->second.jpeg.clear();
            }
        }

        if (wanted.empty()) {
            if (drawing) {
                drawing = false;
                go_idle();
            }
            const auto log_due = idle_log_due();
            if (log_due <= started) {
                flush_idle_log(started);
            }
            // Nothing to draw: sleep until somebody asks. With nobody asking at all that is an
            // untimed wait — the whole idle cost of the camera — or one that ends when the idle
            // log line is due. A vehicle asked for that has since left the fleet is looked at
            // again at the camera rate until its demand lapses.
            std::unique_lock<std::mutex> lock(demand_mutex_);
            const auto woken = [&] { return stop_.load(std::memory_order_relaxed) || demand_epoch_ != seen_epoch; };
            auto deadline = log_due > started ? log_due : Clock::time_point::max();
            if (any_demand) {
                deadline = std::min(deadline, started + period);
            }
            if (deadline == Clock::time_point::max()) {
                wake_.wait(lock, woken);
            } else {
                wake_.wait_until(lock, deadline, woken);
            }
            continue;
        }
        drawing = true;

        Window pass;
        pass.started = started;
        pass.watched = wanted.size();
        for (const Wanted &w : wanted) {
            if (stop_.load(std::memory_order_relaxed)) {
                break;
            }
            const Pose *pose = w.pose;
            Drawn &d = last[pose->vehicle_id];
            if (d.image && same_pose(d.pose, *pose)) {
                ++pass.reused;
            } else {
                const auto t0 = Clock::now();
                const Camera camera =
                    pose->gimbal ? Camera::on_gimbal(cfg_.camera, pose->position_ned, pose->quat_ned_frd,
                                                     pose->gimbal_yaw_rad, pose->gimbal_pitch_rad)
                                 : Camera(cfg_.camera, pose->position_ned, pose->quat_ned_frd);
                d.image = std::make_shared<const Image>(rasterizer_->render(*scene_, camera, pool_.get()));
                d.pose = *pose;
                d.jpeg.clear();
                const double render_ms = ms_between(t0, Clock::now());
                ++pass.drawn;
                pass.render_ms_sum += render_ms;
                pass.render_ms_max = std::max(pass.render_ms_max, render_ms);
            }
            const auto drawn_at = Clock::now();
            if (w.jpeg) {
                if (d.jpeg.empty()) {
                    d.jpeg = encode_jpeg(*d.image, cfg_.quality);
                    ++pass.encoded;
                    pass.encode_ms_sum += ms_between(drawn_at, Clock::now());
                }
                if (!d.jpeg.empty()) {
                    // Stamped with the moment the picture was ready, before its JPEG was made —
                    // the instant an H.264 frame's RTP timestamp names — so the two feeds'
                    // latencies are measured from the same point.
                    const auto unix_us =
                        std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::system_clock::now().time_since_epoch() - (Clock::now() - drawn_at))
                            .count();
                    frames_.publish(pose->vehicle_id, d.jpeg, pose->sim_time_s, unix_us);
                }
            }
            if (w.video && cfg_.video_sink && !d.image->empty()) {
                cfg_.video_sink(pose->vehicle_id, d.image, pose->sim_time_s, drawn_at);
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
