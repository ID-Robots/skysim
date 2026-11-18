#include "render/render_service.h"

#include "render/jpeg.h"
#include "render/resize.h"
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

// Moved enough to count as moving, for the landing camera's "at rest" (Config::settle_s): far
// coarser than same_pose, which decides whether a picture can be re-sent. A parked vehicle's
// contact jitter and an autopilot's idle servo noise stay well inside these.
bool moved(const RenderService::Pose &a, const RenderService::Pose &b) {
    constexpr double kMoveM = 0.02;
    constexpr double kTurnRad = 0.5 * M_PI / 180.0;
    const double dn = a.position_ned[0] - b.position_ned[0];
    const double de = a.position_ned[1] - b.position_ned[1];
    const double dd = a.position_ned[2] - b.position_ned[2];
    if (dn * dn + de * de + dd * dd > kMoveM * kMoveM) {
        return true;
    }
    double dot = 0.0;
    for (size_t k = 0; k < 4; ++k) {
        dot += a.quat_ned_frd[k] * b.quat_ned_frd[k];
    }
    // The angle between two unit quaternions' rotations is 2 acos |dot|.
    if (std::fabs(dot) < std::cos(kTurnRad * 0.5)) {
        return true;
    }
    return a.gimbal != b.gimbal || std::fabs(a.gimbal_pitch_rad - b.gimbal_pitch_rad) > kTurnRad ||
           std::fabs(a.gimbal_yaw_rad - b.gimbal_yaw_rad) > kTurnRad;
}

// On the ground: the dock and the landing zone are measured from the ground plane (NED z = 0, where
// the markers lie), and a quad resting on it has its centre 0.07 m up (its collision box's half
// height; World), give or take a tilt. Only a vehicle this low can be at rest, and one above it is
// off the ground: a hover over the dock, even at 0.2 m, keeps the landing mode however still it
// holds.
constexpr double kRestHeightM = 0.12;

// The longest the render thread sleeps with vehicles in the fleet: poses keep arriving without
// waking it, and a vehicle that takes off from rest (2 fps, or 0.1) or enters the landing zone
// must not wait a rest period for its new mode — nor must GET .../camera report a stale one.
constexpr auto kModeRefresh = std::chrono::milliseconds(250);

// `m` with the given fields of `patch`, each checked against the bounds in camera_mode.h. False
// (and `m` unchanged) with `error` naming the first field out of bounds.
bool apply_mode_patch(CameraMode &m, const CameraModePatch &patch, std::string *error) {
    if (!check_mode_patch(patch, error)) {
        return false;
    }
    m.width = patch.width.value_or(m.width);
    m.height = patch.height.value_or(m.height);
    m.fps = patch.fps.value_or(m.fps);
    m.bitrate_kbps = patch.bitrate_kbps.value_or(m.bitrate_kbps);
    return true;
}

bool usable(const CameraMode &m) { return m.fps > 0.0; }

// A mode's size resolved against the picture: 0 means the picture's own.
CameraMode sized(CameraMode m, std::pair<int, int> picture) {
    if (m.width <= 0 || m.height <= 0) {
        m.width = picture.first;
        m.height = picture.second;
    }
    return m;
}

// The draw size has the picture's aspect ratio to the nearest pixel (0 = the picture's own).
bool same_aspect(const CameraMode &m, std::pair<int, int> picture) {
    const CameraMode r = sized(m, picture);
    const double height = static_cast<double>(r.width) * picture.second / picture.first;
    return std::fabs(static_cast<double>(r.height) - height) <= 0.5 + 1e-9;
}

// A draw size given by one side only: the other is the picture's aspect ratio of it. (Either side
// 0 alone would otherwise read as "the picture's size" for both.)
CameraMode complete(CameraMode m, std::pair<int, int> picture) {
    if (m.width > 0 && m.height <= 0) {
        m.height =
            static_cast<int>(std::lround(static_cast<double>(m.width) * picture.second / picture.first));
    } else if (m.height > 0 && m.width <= 0) {
        m.width =
            static_cast<int>(std::lround(static_cast<double>(m.height) * picture.first / picture.second));
    }
    return m;
}

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
    if (cfg_.ground_cache_bytes > 0 && scene_) {
        ground_ = std::make_unique<GroundCache>(scene_, cfg_.ground_cache_bytes);
    }
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
    Stats s;
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        s = stats_;
    }
    if (ground_) {
        s.ground = ground_->stats();
    }
    return s;
}

double RenderService::Window::fps() const {
    return vehicle_s > 0.0 ? static_cast<double>(delivered) / vehicle_s : 0.0;
}

// --- per-vehicle camera modes (TASK-2661) --------------------------------------------------

std::pair<int, int> RenderService::picture_of(const Setting *s) const {
    return s != nullptr && s->picture ? *s->picture
                                      : std::pair<int, int>{cfg_.camera.width, cfg_.camera.height};
}

CameraMode RenderService::normal_of(const Setting *s) const {
    return s != nullptr && s->normal ? *s->normal : fleet_mode();
}

std::optional<CameraMode> RenderService::landing_of(const Setting *s) const {
    if (s != nullptr && !s->landing_enabled) {
        return std::nullopt;
    }
    if (s != nullptr && s->landing) {
        return *s->landing;
    }
    if (usable(cfg_.landing)) {
        return cfg_.landing;
    }
    return std::nullopt;
}

RenderService::VehicleCamera RenderService::describe(const Setting *s) const {
    VehicleCamera c;
    const std::pair<int, int> picture = picture_of(s);
    c.picture_width = picture.first;
    c.picture_height = picture.second;
    c.picture_own = s != nullptr && s->picture.has_value();
    c.normal = sized(normal_of(s), picture);
    c.normal_own = s != nullptr && s->normal.has_value();
    const std::optional<CameraMode> landing = landing_of(s);
    c.landing_available = landing.has_value();
    c.landing_enabled = s == nullptr || s->landing_enabled;
    c.landing = sized(landing.value_or(s != nullptr && s->landing ? *s->landing : cfg_.landing), picture);
    c.landing_own = s != nullptr && s->landing.has_value();
    c.landing_radius_m = cfg_.landing_radius_m;
    c.landing_height_m = cfg_.landing_height_m;
    c.landing_active = s != nullptr && s->landing_active && landing.has_value();
    c.rest_fps = cfg_.rest_fps;
    c.resting = s != nullptr && s->resting && !c.landing_active && cfg_.rest_fps > 0.0;
    c.now = c.landing_active ? c.landing : c.normal;
    if (c.resting) {
        c.now.fps = cfg_.rest_fps;
    }
    if (s != nullptr) {
        c.watched = s->watched;
        c.fps = s->fps;
        c.render_ms = s->render_ms;
        c.frames_drawn = s->drawn;
        c.frames_reused = s->reused;
    }
    return c;
}

std::optional<RenderService::VehicleCamera>
RenderService::configure(uint32_t vehicle_id, const CameraPatch &patch, std::string *error) {
    std::lock_guard<std::mutex> lock(settings_mutex_);
    if (retired_.count(vehicle_id) != 0) {
        if (error != nullptr) {
            error->clear(); // not a bad request: no such vehicle
        }
        return std::nullopt;
    }
    const auto it = settings_.find(vehicle_id);
    Setting next = it != settings_.end() ? it->second : Setting{};
    if (!apply(next, patch, error)) {
        return std::nullopt;
    }
    if (picture_of(&next) != picture_of(it != settings_.end() ? &it->second : nullptr)) {
        // A new picture size: the JPEG kept for camera.jpg is the old size, and a reader of it
        // must wait for one of the new size rather than be handed it with the new intrinsics.
        frames_.erase(vehicle_id);
    }
    Setting &stored = settings_[vehicle_id];
    stored.picture = next.picture;
    stored.normal = next.normal;
    stored.landing = next.landing;
    stored.landing_enabled = next.landing_enabled;
    return describe(&stored);
}

bool RenderService::check(const CameraPatch &patch, std::string *error) const {
    Setting fresh;
    return apply(fresh, patch, error);
}

bool RenderService::apply(Setting &next, const CameraPatch &patch, std::string *error) const {
    if (patch.reset) {
        next.picture.reset();
        next.normal.reset();
        next.landing.reset();
        next.landing_enabled = true;
    }
    if (patch.picture_width || patch.picture_height) {
        if (!check_picture_patch(patch, error)) {
            return false;
        }
        const std::pair<int, int> now = picture_of(&next);
        next.picture = std::pair<int, int>{patch.picture_width.value_or(now.first),
                                           patch.picture_height.value_or(now.second)};
    }
    if (!patch.normal.empty()) {
        CameraMode m = normal_of(&next);
        if (!apply_mode_patch(m, patch.normal, error)) {
            return false;
        }
        next.normal = complete(m, picture_of(&next));
    }
    if (!patch.landing.empty()) {
        // From the vehicle's own landing mode, else the fleet's, else a full-size drawing at its
        // normal rate: a landing mode is for drawing every pixel.
        CameraMode m =
            next.landing ? *next.landing
                         : (usable(cfg_.landing) ? cfg_.landing : CameraMode{0, 0, normal_of(&next).fps, 0});
        if (!apply_mode_patch(m, patch.landing, error)) {
            return false;
        }
        next.landing = complete(m, picture_of(&next));
        next.landing_enabled = true; // giving it a landing camera is asking for one
    }
    if (patch.landing_enabled) {
        next.landing_enabled = *patch.landing_enabled;
    }
    // A mode drawn at another aspect ratio than the picture's would be stretched to it, and its
    // pixels would no longer be the picture's pinhole (the intrinsics route's fx = fy): the draw
    // height must be the draw width's share of the picture's, to the nearest pixel.
    const std::pair<int, int> picture = picture_of(&next);
    const std::optional<CameraMode> landing = landing_of(&next);
    if (!same_aspect(normal_of(&next), picture) || (landing && !same_aspect(*landing, picture))) {
        if (error != nullptr) {
            *error = "a mode's draw size must have the picture's aspect ratio (" +
                     std::to_string(picture.first) + "x" + std::to_string(picture.second) +
                     ": a draw height of the width times " + std::to_string(picture.second) + "/" +
                     std::to_string(picture.first) + ", to the nearest pixel)";
        }
        return false;
    }
    return true;
}

RenderService::VehicleCamera RenderService::camera(uint32_t vehicle_id) const {
    std::lock_guard<std::mutex> lock(settings_mutex_);
    const auto it = settings_.find(vehicle_id);
    return describe(it != settings_.end() ? &it->second : nullptr);
}

CameraMode RenderService::mode_now(uint32_t vehicle_id) const { return camera(vehicle_id).now; }

CameraMode RenderService::stream_mode(uint32_t vehicle_id) const {
    const VehicleCamera c = camera(vehicle_id);
    const CameraMode &mode = c.landing_active ? c.landing : c.normal;
    return {c.picture_width, c.picture_height, mode.fps, mode.bitrate_kbps};
}

void RenderService::forget(uint32_t vehicle_id) {
    std::lock_guard<std::mutex> lock(settings_mutex_);
    settings_.erase(vehicle_id);
    // A camera POST that read the fleet before this despawn and lands after it must not bring the
    // settings back: ids are never reused, so a gone one stays gone.
    retired_.insert(vehicle_id);
}

void RenderService::record_pass(const Window &pass, Clock::time_point now) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_.frames_drawn += pass.drawn;
    stats_.frames_reused += pass.reused;
    stats_.watched = pass.watched;
    stats_.landing = pass.landing;
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
            w->delivered += pass.drawn + pass.reused;
            // Each step weighted by the vehicles watched in it: one that joins for a pass adds a
            // pass's worth of time, not a whole window's.
            w->vehicle_s += step_s * static_cast<double>(pass.watched);
        }
        w->landing = std::max(w->landing, pass.landing);
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
    stats_.landing = 0;
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
    char line[320];
    std::snprintf(
        line, sizeof(line),
        "skysim: camera: %zu vehicle(s) watched (%zu landing), %.1f fps each (target %.1f), render %.1f "
        "ms/frame (max %.1f), encode %.1f ms, %llu drawn + %llu re-sent, %d thread(s)",
        w.watched, w.landing, w.fps(), cfg_.fps, w.render_ms_sum / drawn, w.render_ms_max,
        w.encode_ms_sum / encoded, static_cast<unsigned long long>(w.drawn),
        static_cast<unsigned long long>(w.reused), threads_);
    emit(line);
}

void RenderService::run() {
    if (cfg_.background_priority) {
        lower_thread_priority();
    }
    const auto window = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(std::max(cfg_.demand_window_s, 0.0)));
    const auto settle = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(std::max(cfg_.settle_s, 0.0)));
    // A frame due this soon is drawn now rather than slept for.
    constexpr auto kSlack = std::chrono::milliseconds(2);

    // The last picture drawn per vehicle, the pose it was drawn from, and its JPEG once one has
    // been made (only a JPEG viewer needs it).
    struct Drawn {
        Pose pose;
        std::shared_ptr<const GroundMarkers> markers; // the dock markers it was drawn with
        std::shared_ptr<const Image> image;           // at the picture size
        int drawn_width{0};                           // the size it was drawn at
        int drawn_height{0};
        std::vector<uint8_t> jpeg;
    };
    // How a vehicle is drawn now: its mode (size resolved), its picture size, and the rate its
    // frames are due at (the mode's, or the rest rate).
    struct Draw {
        CameraMode mode;
        std::pair<int, int> picture;
        double fps;
    };
    // Per vehicle in the fleet: whether it is moving (for the landing camera), whether it is on
    // the landing camera, when its next frame is due, and its own rate over the last seconds.
    struct Track {
        bool seen{false};
        Pose anchor; // the pose its last move was measured from
        Clock::time_point moved_at{};
        bool landing{false};
        bool due_set{false};
        Clock::time_point next_due{};
        double due_fps{0.0}; // the rate next_due was set at: a new rate starts a new cadence now
        bool counting{false};
        Clock::time_point count_since{};
        uint64_t counted{0};
        double fps{0.0};
        double render_ms{0.0};
        uint64_t drawn{0};
        uint64_t reused{0};
    };
    // A vehicle on the draw list, who wants it, and how it is drawn now.
    struct Wanted {
        const Pose *pose;
        bool jpeg;
        bool video;
        Draw draw;
    };
    std::unordered_map<uint32_t, Drawn> last;
    std::unordered_map<uint32_t, Track> tracks;
    std::unordered_map<uint32_t, Draw> modes;
    std::vector<uint32_t> live;
    std::vector<uint32_t> lapsed;
    std::vector<uint32_t> lapsed_video;
    std::vector<Wanted> wanted;
    std::vector<const Wanted *> due;
    struct Place {
        bool inside;
        bool outside;
        bool at_rest;
        bool airborne;
    };
    std::unordered_map<uint32_t, Place> places;
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
        const auto gone = [&live](uint32_t id) {
            return std::find(live.begin(), live.end(), id) == live.end();
        };
        for (auto it = last.begin(); it != last.end();) {
            it = gone(it->first) ? last.erase(it) : std::next(it);
        }
        for (auto it = tracks.begin(); it != tracks.end();) {
            it = gone(it->first) ? tracks.erase(it) : std::next(it);
        }

        // Where each vehicle is against its dock, and whether it has been at rest there: what the
        // landing camera goes by. Docks are read before the settings lock is taken.
        places.clear();
        for (const Pose &pose : poses) {
            Track &t = tracks[pose.vehicle_id];
            if (!t.seen) {
                // At rest from the start: a vehicle spawned on its dock begins with its normal camera.
                t.seen = true;
                t.anchor = pose;
                t.moved_at = started - settle;
            } else if (moved(t.anchor, pose)) {
                t.anchor = pose;
                t.moved_at = started;
            }
            double dock_n = pose.home_north_m;
            double dock_e = pose.home_east_m;
            if (cfg_.docks) {
                if (const auto dock = cfg_.docks->get(pose.vehicle_id)) {
                    dock_n = dock->pose.north_m;
                    dock_e = dock->pose.east_m;
                }
            }
            const double dn = pose.position_ned[0] - dock_n;
            const double de = pose.position_ned[1] - dock_e;
            const double distance = std::sqrt(dn * dn + de * de);
            const double height = -pose.position_ned[2];
            const double margin = std::max(cfg_.landing_margin_m, 0.0);
            places[pose.vehicle_id] = {
                distance <= cfg_.landing_radius_m && height <= cfg_.landing_height_m,
                !(distance <= cfg_.landing_radius_m + margin && height <= cfg_.landing_height_m + margin),
                started - t.moved_at >= settle && height <= kRestHeightM, height > kRestHeightM};
        }

        // Each vehicle's mode now. Runtime-only entries of vehicles that have gone are dropped
        // here; a vehicle's own settings stay until forget() (they can arrive before its first pose).
        size_t landing_now = 0;
        modes.clear();
        {
            std::lock_guard<std::mutex> lock(settings_mutex_);
            for (auto it = settings_.begin(); it != settings_.end();) {
                const Setting &st = it->second;
                const bool own = st.picture || st.normal || st.landing || !st.landing_enabled;
                it = !own && gone(it->first) ? settings_.erase(it) : std::next(it);
            }
            for (const Pose &pose : poses) {
                Track &t = tracks[pose.vehicle_id];
                const Place &place = places[pose.vehicle_id];
                Setting &st = settings_[pose.vehicle_id];
                const std::optional<CameraMode> landing = landing_of(&st);
                if (!landing) {
                    t.landing = false;
                } else if (!t.landing && place.inside && place.airborne) {
                    // Off the ground near its dock. Not before: a parked vehicle whose autopilot
                    // starts driving its gimbal moves, but it is not landing.
                    t.landing = true;
                } else if (t.landing && (place.outside || place.at_rest)) {
                    t.landing = false;
                }
                st.landing_active = t.landing;
                landing_now += t.landing ? 1 : 0;
                // At rest on the ground: its normal mode, at the rest rate.
                st.resting = !t.landing && place.at_rest && cfg_.rest_fps > 0.0;
                const std::pair<int, int> picture = picture_of(&st);
                const CameraMode mode = sized(t.landing ? *landing : normal_of(&st), picture);
                modes[pose.vehicle_id] = {mode, picture, st.resting ? cfg_.rest_fps : mode.fps};
            }
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
                    wanted.push_back({&pose, jpeg, video, modes[pose.vehicle_id]});
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
        // A vehicle off the draw list starts again at once when it is back on it, and its rate is
        // nobody's business meanwhile.
        for (auto &[id, t] : tracks) {
            const bool on = std::any_of(wanted.begin(), wanted.end(),
                                        [id = id](const Wanted &w) { return w.pose->vehicle_id == id; });
            if (!on) {
                t.due_set = false;
                t.counting = false;
                t.fps = 0.0;
            }
        }
        {
            // ... and what GET .../camera reports for them says so (their frame counts stay).
            std::lock_guard<std::mutex> lock(settings_mutex_);
            for (auto &[id, st] : settings_) {
                if (std::none_of(wanted.begin(), wanted.end(),
                                 [id = id](const Wanted &w) { return w.pose->vehicle_id == id; })) {
                    st.watched = false;
                    st.fps = 0.0;
                }
            }
        }

        if (wanted.empty()) {
            {
                std::lock_guard<std::mutex> lock(settings_mutex_);
                for (auto &[id, st] : settings_) {
                    st.watched = false;
                    st.fps = 0.0;
                }
            }
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
                const double fps = cfg_.fps > 0.0 ? cfg_.fps : 1.0;
                deadline = std::min(deadline, started + std::chrono::duration_cast<Clock::duration>(
                                                            std::chrono::duration<double>(1.0 / fps)));
            }
            if (!poses.empty()) {
                // Nobody watching, but the fleet's modes are still followed (no drawing), so
                // GET .../camera and an RTSP DESCRIBE say the mode a vehicle is in now.
                deadline = std::min(deadline, started + kModeRefresh);
            }
            if (deadline == Clock::time_point::max()) {
                wake_.wait(lock, woken);
            } else {
                wake_.wait_until(lock, deadline, woken);
            }
            continue;
        }
        drawing = true;

        // Whose frame is due. Each vehicle keeps its own cadence; the soonest one not yet due is
        // what the thread sleeps until (or until somebody new asks).
        due.clear();
        auto next_wake = Clock::time_point::max();
        for (const Wanted &w : wanted) {
            Track &t = tracks[w.pose->vehicle_id];
            if (t.due_set && t.due_fps != w.draw.fps) {
                t.due_set = false; // off rest, into landing, a new setting: due now, at the new rate
            }
            if (!t.due_set || t.next_due <= started + kSlack) {
                due.push_back(&w);
            } else {
                next_wake = std::min(next_wake, t.next_due);
            }
        }
        if (due.empty()) {
            std::unique_lock<std::mutex> lock(demand_mutex_);
            wake_.wait_until(lock, std::min(next_wake, started + kModeRefresh), [&] {
                return stop_.load(std::memory_order_relaxed) || demand_epoch_ != seen_epoch;
            });
            continue;
        }

        Window pass;
        pass.started = started;
        pass.watched = wanted.size();
        for (const Wanted &w : wanted) {
            pass.landing += tracks[w.pose->vehicle_id].landing ? 1 : 0;
        }
        for (const Wanted *wp : due) {
            if (stop_.load(std::memory_order_relaxed)) {
                break;
            }
            const Wanted &w = *wp;
            const Pose *pose = w.pose;
            Track &t = tracks[pose->vehicle_id];
            // The docks this vehicle's camera sees — its own alone, unless every camera is to see
            // every dock (DockBoard::seen_by, TASK-2609). The same pointer until that list changes,
            // so a parked vehicle's picture stays current — and re-sent — until a dock in it moves.
            const std::shared_ptr<const GroundMarkers> markers =
                cfg_.docks ? cfg_.docks->seen_by(pose->vehicle_id) : nullptr;
            Drawn &d = last[pose->vehicle_id];
            const CameraMode &mode = w.draw.mode;
            const std::pair<int, int> &picture = w.draw.picture;
            if (d.image && same_pose(d.pose, *pose) && d.markers == markers && d.drawn_width == mode.width &&
                d.drawn_height == mode.height && d.image->width == picture.first &&
                d.image->height == picture.second) {
                ++pass.reused;
                ++t.reused;
            } else {
                const auto t0 = Clock::now();
                CameraConfig cam = cfg_.camera;
                cam.width = mode.width;
                cam.height = mode.height;
                const Camera camera = pose->gimbal
                                          ? Camera::on_gimbal(cam, pose->position_ned, pose->quat_ned_frd,
                                                              pose->gimbal_yaw_rad, pose->gimbal_pitch_rad)
                                          : Camera(cam, pose->position_ned, pose->quat_ned_frd);
                const Rasterizer raster(cam);
                Image image =
                    raster.render(*scene_, camera, pool_.get(), nullptr, true, markers.get(), ground_.get());
                if (mode.width != picture.first || mode.height != picture.second) {
                    // Drawn smaller (or larger) than the picture: every frame of a vehicle is its
                    // picture's size, whatever mode drew it.
                    image = resize_bilinear(image, picture.first, picture.second, pool_.get());
                }
                d.image = std::make_shared<const Image>(std::move(image));
                d.drawn_width = mode.width;
                d.drawn_height = mode.height;
                d.pose = *pose;
                d.markers = markers;
                d.jpeg.clear();
                if (ground_) {
                    ground_->evict(); // between frames: no sampler of the cache is in use
                }
                const double render_ms = ms_between(t0, Clock::now());
                ++pass.drawn;
                pass.render_ms_sum += render_ms;
                pass.render_ms_max = std::max(pass.render_ms_max, render_ms);
                ++t.drawn;
                t.render_ms = render_ms;
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
                    // Only while the picture is still this vehicle's size: a POST that changed it
                    // while this frame was drawn has erased the stored JPEG (configure()), and an
                    // old-size frame published now would be served with the new intrinsics. Checked
                    // and published under the settings lock, which configure() changes it under.
                    std::lock_guard<std::mutex> lock(settings_mutex_);
                    const auto it = settings_.find(pose->vehicle_id);
                    if (picture_of(it != settings_.end() ? &it->second : nullptr) == picture) {
                        frames_.publish(pose->vehicle_id, d.jpeg, pose->sim_time_s, unix_us);
                    }
                }
            }
            if (w.video && cfg_.video_sink && !d.image->empty()) {
                // The stream's picture, frame rate and bitrate: the mode's rate even at rest, so resting
                // changes how often a frame comes, not the stream's encoder.
                cfg_.video_sink(pose->vehicle_id, d.image, pose->sim_time_s, drawn_at,
                                CameraMode{picture.first, picture.second, mode.fps, mode.bitrate_kbps});
            }

            // Its next frame: one period on from when this one was due, so a late frame does not
            // push the cadence back; a vehicle more than a period behind is due again at once,
            // which is the rate simply dropping (frames get older, nothing else is affected).
            const auto period = std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(1.0 / std::max(w.draw.fps, 0.1)));
            t.next_due = t.due_set ? std::max(t.next_due + period, started) : started + period;
            t.due_set = true;
            t.due_fps = w.draw.fps;
            if (!t.counting) {
                t.counting = true;
                t.count_since = started;
                t.counted = 0;
            } else {
                ++t.counted;
                const double span = std::chrono::duration<double>(started - t.count_since).count();
                if (started - t.count_since >= kStatsWindow) {
                    t.fps = static_cast<double>(t.counted) / span;
                    t.count_since = started;
                    t.counted = 0;
                }
            }
        }
        pass.passes = 1;
        record_pass(pass, Clock::now());
        {
            std::lock_guard<std::mutex> lock(settings_mutex_);
            for (const Wanted &w : wanted) {
                const Track &t = tracks[w.pose->vehicle_id];
                Setting &st = settings_[w.pose->vehicle_id];
                st.watched = true;
                st.fps = t.fps;
                st.render_ms = t.render_ms;
                st.drawn = t.drawn;
                st.reused = t.reused;
            }
        }
    }
}

} // namespace skysim::render
