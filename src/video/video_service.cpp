#include "video/video_service.h"

#include "render/still.h"
#include "video/h264_encoder.h"

#include <algorithm>
#include <cmath>

namespace skysim::video {
namespace {

constexpr auto kStatsWindow = std::chrono::seconds(2);

std::chrono::steady_clock::duration seconds(double s) {
    return std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(s));
}

} // namespace

EncoderSettings apply_patch(EncoderSettings s, const EncoderPatch &patch, double max_fps) {
    const double fps_ceiling = std::max(max_fps, kMinFps);
    if (patch.bitrate_kbps && std::isfinite(*patch.bitrate_kbps)) {
        s.bitrate_kbps =
            static_cast<int>(std::lround(std::clamp(*patch.bitrate_kbps, static_cast<double>(kMinBitrateKbps),
                                                    static_cast<double>(kMaxBitrateKbps))));
    }
    if (patch.fps && std::isfinite(*patch.fps)) {
        s.fps = std::clamp(*patch.fps, kMinFps, fps_ceiling);
    }
    if (patch.keyframe_interval_s && std::isfinite(*patch.keyframe_interval_s)) {
        s.keyframe_interval_s =
            std::clamp(*patch.keyframe_interval_s, kMinKeyframeIntervalS, kMaxKeyframeIntervalS);
    }
    s.bitrate_kbps = std::clamp(s.bitrate_kbps, kMinBitrateKbps, kMaxBitrateKbps);
    s.fps = std::clamp(s.fps, kMinFps, fps_ceiling);
    s.keyframe_interval_s = std::clamp(s.keyframe_interval_s, kMinKeyframeIntervalS, kMaxKeyframeIntervalS);
    return s;
}

VideoService::VideoService(const Config &cfg, Hooks hooks) : cfg_(cfg), hooks_(std::move(hooks)) {
    cfg_.defaults = apply_patch(cfg_.defaults, {}, cfg_.max_fps);
    cfg_.threads = std::max(cfg_.threads, 1);
    window_start_ = Clock::now();
    RtspServer::Hooks rh;
    rh.describe = [this](int instance) { return describe(instance); };
    rh.playing = [this](int instance, bool on) { playing(instance, on); };
    rh.keyframe = [this](int instance) {
        std::lock_guard<std::mutex> lock(mutex_);
        stream_for(instance).force_idr = true;
    };
    // DESCRIBE's parameter sets for the default rate, made here rather than on the RTSP thread
    // when the first receiver asks.
    parameter_sets(cfg_.width, cfg_.height, cfg_.defaults.fps);
    rtsp_ = std::make_unique<RtspServer>(cfg_.rtsp, std::move(rh));
    for (int i = 0; i < cfg_.threads; ++i) {
        workers_.emplace_back([this] { worker(); });
    }
    house_ = std::thread([this] { housekeeping(); });
}

VideoService::~VideoService() { stop(); }

void VideoService::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopped_) {
            return;
        }
        stopped_ = true;
        stopping_ = true;
    }
    work_cv_.notify_all();
    house_cv_.notify_all();
    for (auto &t : workers_) {
        t.join();
    }
    house_.join();
    // Last: the workers publish into it, and its hooks only touch state that outlives it.
    rtsp_.reset();
}

int VideoService::port() const { return rtsp_ ? rtsp_->port() : 0; }

VideoService::Stream &VideoService::stream_for(int instance) {
    auto [it, inserted] = streams_.try_emplace(instance);
    if (inserted) {
        it->second.instance = instance;
        it->second.settings = cfg_.defaults;
        // Defaults below the camera's rate are a cap like a POSTed one; at it, the stream follows
        // the camera wherever its mode takes it.
        it->second.fps_set = cfg_.defaults.fps < cfg_.max_fps - 1e-9;
    }
    return it->second;
}

void VideoService::set_settings(Stream &s, const EncoderSettings &next) {
    if (std::fabs(next.fps - s.settings.fps) > 1e-9) {
        s.have_due = false; // the worker sees the new rate and builds a new encoder for it
    }
    s.settings = next;
}

EncoderSettings VideoService::effective(const Stream &s) const {
    EncoderSettings e = s.settings;
    const double cam = std::max(camera_fps(s), kMinFps);
    e.fps = s.fps_set ? std::clamp(s.settings.fps, kMinFps, cam) : cam;
    if (s.camera_bitrate_kbps > 0) {
        e.bitrate_kbps = std::clamp(s.camera_bitrate_kbps, kMinBitrateKbps, kMaxBitrateKbps);
    }
    return e;
}

void VideoService::refresh_camera(int instance, uint32_t vehicle_id) {
    if (vehicle_id == 0 || !hooks_.camera) {
        return;
    }
    const render::CameraMode mode = hooks_.camera(vehicle_id);
    if (mode.width <= 0 || mode.height <= 0 || !(mode.fps > 0.0)) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    Stream &s = stream_for(instance);
    if (s.settings_owner != 0 && s.settings_owner != vehicle_id) {
        return; // another vehicle holds the instance now; its own pictures will say
    }
    if (std::fabs(mode.fps - s.camera_fps) > 1e-9) {
        s.have_due = false;
    }
    s.camera_width = mode.width;
    s.camera_height = mode.height;
    s.camera_fps = mode.fps;
    s.camera_bitrate_kbps = mode.bitrate_kbps;
}

void VideoService::adopt(Stream &s, uint32_t vehicle_id) {
    if (vehicle_id == 0 || vehicle_id == s.settings_owner) {
        return;
    }
    // A new vehicle at this instance: the last one's settings were for it, not this one.
    if (s.settings_owner != 0) {
        set_settings(s, cfg_.defaults);
        s.fps_set = cfg_.defaults.fps < cfg_.max_fps - 1e-9;
        s.camera_width = 0; // the new vehicle's camera says on its first picture or refresh
        s.camera_height = 0;
        s.camera_fps = 0.0;
        s.camera_bitrate_kbps = 0;
    }
    s.settings_owner = vehicle_id;
}

void VideoService::bind_vehicle(Stream &s, uint32_t vehicle_id) {
    adopt(s, vehicle_id);
    if (vehicle_id == s.vehicle_id) {
        return;
    }
    if (s.vehicle_id != 0) {
        auto it = by_vehicle_.find(s.vehicle_id);
        if (it != by_vehicle_.end() && it->second == s.instance) {
            by_vehicle_.erase(it);
        }
    }
    s.vehicle_id = vehicle_id;
    if (vehicle_id != 0) {
        by_vehicle_[vehicle_id] = s.instance;
    }
    // A different camera: the receivers need an IDR from it, and no picture of the last one.
    s.force_idr = true;
    s.have_due = false;
    s.pending.reset();
}

VideoService::StreamState VideoService::state_of(const Stream &s) const {
    StreamState st;
    st.settings = effective(s);
    st.width = camera_width(s) & ~1;
    st.height = camera_height(s) & ~1;
    st.max_fps = camera_fps(s);
    st.clients = s.clients;
    return st;
}

VideoService::StreamState VideoService::encoder(int instance, uint32_t vehicle_id,
                                                const EncoderPatch *patch) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        adopt(stream_for(instance), vehicle_id);
    }
    refresh_camera(instance, vehicle_id);
    StreamState st;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        Stream &s = stream_for(instance);
        adopt(s, vehicle_id);
        if (patch != nullptr) {
            // Clamped against the camera's rate now, like every POST: a cap, which a faster mode
            // later (the landing camera's, say) does not lift.
            const EncoderSettings next = apply_patch(s.settings, *patch, camera_fps(s));
            if (patch->fps && std::isfinite(*patch->fps)) {
                s.fps_set = true;
            }
            set_settings(s, next);
        }
        st = state_of(s);
    }
    // A new rate has new parameter sets: made here, on the caller's (HTTP) thread, so the next
    // DESCRIBE finds them ready instead of opening an encoder on the RTSP thread.
    parameter_sets(st.width, st.height, st.settings.fps);
    return st;
}

size_t VideoService::request_keyframe(int instance, uint32_t vehicle_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    Stream &s = stream_for(instance);
    adopt(s, vehicle_id);
    s.force_idr = true;
    return s.clients;
}

std::optional<VideoService::ParameterSets> VideoService::parameter_sets(int width, int height, double fps) {
    const auto key = std::make_tuple(width & ~1, height & ~1, fps);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = parameter_sets_.find(key);
        if (it != parameter_sets_.end()) {
            return it->second;
        }
    }
    // Made outside the lock: opening an encoder takes milliseconds. The stream's own encoder
    // writes the same SPS and PPS — they depend on the size and rate, not on the bitrate.
    EncoderConfig ec;
    ec.width = width;
    ec.height = height;
    ec.fps = fps;
    ec.bitrate_kbps = cfg_.defaults.bitrate_kbps;
    ec.preset = cfg_.preset;
    const auto enc = H264Encoder::create(ec);
    if (!enc) {
        return std::nullopt;
    }
    ParameterSets ps{enc->sps(), enc->pps(), enc->width(), enc->height()};
    std::lock_guard<std::mutex> lock(mutex_);
    parameter_sets_.try_emplace(key, ps);
    return ps;
}

std::optional<RtspServer::StreamInfo> VideoService::describe(int instance) {
    const uint32_t id = hooks_.resolve ? hooks_.resolve(instance) : 0;
    if (id == 0) {
        return std::nullopt;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        adopt(stream_for(instance), id);
    }
    // The SDP names the picture the vehicle's camera draws now; should it change size later, the
    // stream's next IDR carries the new SPS (the sprop-parameter-sets are only where it starts).
    refresh_camera(instance, id);
    EncoderSettings settings;
    int width = 0;
    int height = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        Stream &s = stream_for(instance);
        adopt(s, id);
        settings = effective(s);
        width = camera_width(s);
        height = camera_height(s);
    }
    // Normally cached already (at startup for the fleet's mode, by the control route for any
    // other); made here, on the RTSP thread, when the camera's mode is new.
    const auto ps = parameter_sets(width, height, settings.fps);
    if (!ps) {
        return std::nullopt;
    }
    return RtspServer::StreamInfo{ps->sps,    ps->pps,      ps->width,
                                  ps->height, settings.fps, settings.bitrate_kbps};
}

void VideoService::playing(int instance, bool on) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        Stream &s = stream_for(instance);
        if (!on) {
            if (s.clients > 0 && --s.clients == 0) {
                s.idle_since = Clock::now();
                s.pending.reset();
                ++house_epoch_;
                house_cv_.notify_all();
            }
            return;
        }
        ++s.clients;
        s.force_idr = true;
    }
    // Ask for pictures now rather than at the next refresh: the first frame is then one render
    // away. The housekeeping thread keeps asking while the session plays.
    const uint32_t id = hooks_.resolve ? hooks_.resolve(instance) : 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        bind_vehicle(stream_for(instance), id);
        ++house_epoch_;
    }
    house_cv_.notify_all();
    if (id != 0 && hooks_.want_frames) {
        hooks_.want_frames(id);
    }
}

void VideoService::on_frame(uint32_t vehicle_id, std::shared_ptr<const render::Image> image,
                            double sim_time_s, Clock::time_point drawn, const render::CameraMode &mode) {
    if (!image || image->empty()) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_) {
        return;
    }
    const auto it = by_vehicle_.find(vehicle_id);
    if (it == by_vehicle_.end()) {
        return;
    }
    Stream &s = streams_.at(it->second);
    if (s.clients == 0) {
        return;
    }
    // The camera this picture was drawn by: its size is the picture's, its rate and bitrate the
    // mode's (TASK-2661). A picture with no mode keeps what the stream knew.
    s.camera_width = image->width;
    s.camera_height = image->height;
    if (mode.fps > 0.0) {
        if (std::fabs(mode.fps - s.camera_fps) > 1e-9) {
            s.have_due = false;
        }
        s.camera_fps = mode.fps;
        s.camera_bitrate_kbps = mode.bitrate_kbps;
    }
    // Below the render rate, take pictures on a schedule of the stream's own rate, with half a
    // render period of slack either way so render jitter neither doubles nor skips a frame.
    const double cam_fps = camera_fps(s);
    const EncoderSettings eff = effective(s);
    if (eff.fps < cam_fps - 1e-9) {
        const auto period = seconds(1.0 / eff.fps);
        const auto slack = seconds(0.5 / cam_fps);
        if (!s.have_due || drawn - s.next_due > period) {
            s.next_due = drawn;
            s.have_due = true;
        }
        if (drawn < s.next_due - slack) {
            return;
        }
        s.next_due += period;
    }
    if (s.pending) {
        ++frames_dropped_;
    }
    s.pending = std::move(image);
    s.pending_sim_s = sim_time_s;
    s.pending_at = drawn;
    if (!s.queued && !s.busy) {
        s.queued = true;
        ready_.push_back(s.instance);
        work_cv_.notify_one();
    }
}

void VideoService::worker() {
    if (cfg_.background_priority) {
        render::lower_thread_priority();
    }
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
        work_cv_.wait(lock, [this] { return stopping_ || !ready_.empty(); });
        if (stopping_) {
            return;
        }
        const int instance = ready_.front();
        ready_.pop_front();
        Stream &s = streams_.at(instance);
        s.queued = false;
        if (!s.pending) {
            continue;
        }
        s.busy = true;
        const std::shared_ptr<const render::Image> image = std::move(s.pending);
        s.pending.reset();
        const Clock::time_point captured = s.pending_at;
        const double sim_time_s = s.pending_sim_s;
        std::unique_ptr<H264Encoder> enc = std::move(s.encoder);
        const EncoderSettings settings = effective(s);
        // A keyframe when asked for (a new receiver, a forced IDR, a new camera), and on the
        // stream's wall-clock interval, with half a frame of slack so a 1 s interval at 10 fps is
        // every tenth frame rather than every eleventh.
        const bool idr_due = !s.have_idr || captured - s.last_idr >=
                                                seconds(settings.keyframe_interval_s - 0.5 / settings.fps);
        bool force = s.force_idr || idr_due;
        s.force_idr = false;
        lock.unlock();

        // The picture's size or the stream's fps changed: a new encoder, and an IDR with the new
        // SPS/PPS, which is how a receiver learns of a new picture size (TASK-2661). A new
        // bitrate is applied in place.
        const bool rebuild = !enc || enc->width() != (image->width & ~1) ||
                             enc->height() != (image->height & ~1) ||
                             std::fabs(enc->config().fps - settings.fps) > 1e-9;
        if (rebuild) {
            EncoderConfig ec;
            ec.width = image->width;
            ec.height = image->height;
            ec.fps = settings.fps;
            ec.bitrate_kbps = settings.bitrate_kbps;
            ec.preset = cfg_.preset;
            enc = H264Encoder::create(ec);
            force = true;
        } else if (enc->config().bitrate_kbps != settings.bitrate_kbps) {
            enc->set_bitrate(settings.bitrate_kbps);
        }
        auto au = std::make_shared<AccessUnit>();
        const auto t0 = Clock::now();
        const bool ok = enc && enc->encode(*image, force, *au);
        const double ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        if (ok) {
            au->captured = captured;
            au->sim_time_s = sim_time_s;
            rtsp_->publish(instance, au);
        }

        lock.lock();
        if (rebuild && enc) {
            parameter_sets_.try_emplace(std::make_tuple(enc->width(), enc->height(), settings.fps),
                                        ParameterSets{enc->sps(), enc->pps(), enc->width(), enc->height()});
        }
        s.encoder = std::move(enc);
        s.busy = false;
        if (!ok) {
            s.force_idr = s.force_idr || force; // an IDR that did not happen is still owed
        }
        if (s.clients == 0) {
            // The last receiver left while this was encoding: housekeeping skipped the busy
            // encoder, so tell it there is one to time out.
            ++house_epoch_;
            house_cv_.notify_all();
        }
        if (ok) {
            ++frames_encoded_;
            bytes_encoded_ += au->bytes();
            window_ms_sum_ += ms;
            ++window_frames_;
            if (au->idr) {
                ++keyframes_;
                s.have_idr = true;
                s.last_idr = captured;
            }
        }
        const auto now = Clock::now();
        if (now - window_start_ >= kStatsWindow) {
            encode_ms_ = window_frames_ > 0 ? window_ms_sum_ / static_cast<double>(window_frames_) : 0.0;
            window_ms_sum_ = 0.0;
            window_frames_ = 0;
            window_start_ = now;
        }
        if (s.pending && !s.queued) {
            s.queued = true;
            ready_.push_back(instance);
            work_cv_.notify_one();
        }
    }
}

void VideoService::housekeeping() {
    std::unique_lock<std::mutex> lock(mutex_);
    std::vector<int> active;
    std::vector<std::pair<int, uint32_t>> resolved;
    while (!stopping_) {
        const auto now = Clock::now();
        auto wake = Clock::time_point::max();
        active.clear();
        for (auto &[instance, s] : streams_) {
            if (s.clients > 0) {
                active.push_back(instance);
            } else if (s.encoder && !s.busy) {
                // Nobody has played it for the linger period: free the encoder (its parameter
                // sets stay cached for the next DESCRIBE).
                const auto expiry = s.idle_since + seconds(cfg_.linger_s);
                if (now >= expiry) {
                    s.encoder.reset();
                    s.have_idr = false;
                } else {
                    wake = std::min(wake, expiry);
                }
            }
        }
        if (!active.empty()) {
            lock.unlock();
            resolved.clear();
            for (const int instance : active) {
                resolved.emplace_back(instance, hooks_.resolve ? hooks_.resolve(instance) : 0);
            }
            lock.lock();
            for (const auto &[instance, id] : resolved) {
                Stream &s = stream_for(instance);
                if (s.clients > 0) {
                    bind_vehicle(s, id);
                }
            }
            lock.unlock();
            for (const auto &[instance, id] : resolved) {
                if (id != 0 && hooks_.want_frames) {
                    hooks_.want_frames(id);
                }
            }
            lock.lock();
            wake = std::min(wake, now + seconds(cfg_.refresh_s));
        }
        // Nobody playing and no encoder waiting to be freed: an untimed wait, the whole idle cost.
        const uint64_t seen = house_epoch_;
        const auto woken = [&] { return stopping_ || house_epoch_ != seen; };
        if (wake == Clock::time_point::max()) {
            house_cv_.wait(lock, woken);
        } else {
            house_cv_.wait_until(lock, wake, woken);
        }
    }
}

VideoService::Stats VideoService::stats() const {
    Stats st;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        st.threads = cfg_.threads;
        for (const auto &[instance, s] : streams_) {
            if (s.clients > 0) {
                ++st.streams;
                st.clients += s.clients;
            }
        }
        st.frames_encoded = frames_encoded_;
        st.frames_dropped = frames_dropped_;
        st.keyframes = keyframes_;
        st.bytes_encoded = bytes_encoded_;
        st.encode_ms = encode_ms_;
        for (const auto &[instance, s] : streams_) {
            st.encoders += (s.encoder || s.busy) ? 1 : 0;
        }
    }
    if (rtsp_) {
        st.port = rtsp_->port();
        st.rtsp = rtsp_->stats();
    }
    return st;
}

} // namespace skysim::video
