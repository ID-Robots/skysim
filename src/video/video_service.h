#pragma once
// The pilot camera as H.264 over RTSP (TASK-2561): what makes a simulated vehicle's camera look,
// on the wire, like the SIYI ZR30 on Observer 1 — rtsp://<host>:8554/instances/<n>/main.264.
//
// Three hand-offs, none of which can stall the one before it:
//
//   tick thread ──poses──▶ render thread ──picture──▶ encoder worker ──frame──▶ RTSP thread
//
// The tick thread only ever publishes poses (RenderService); nothing here runs on it. The render
// thread hands each picture a stream wants to on_frame(), which drops it into the stream's
// one-picture mailbox — replacing, and counting as dropped, one the encoder has not got to yet —
// and returns. Encoder workers (a small pool at background priority) take a stream's latest
// picture, encode it, and publish the frame to the RTSP server, which queues a pointer and sends
// from its own thread. A slow encoder therefore costs frames, never a render pass; a slow
// receiver costs its own frames, never the encoder.
//
// Streams are keyed by instance, like the URL, and encode only while somebody is playing them:
// with no RTSP session open on an instance nothing asks the renderer for its pictures, and its
// encoder is freed after a linger period. The renderer keeps drawing for its demand window
// (RenderService::Config::demand_window_s) after the last session ends; those pictures are
// dropped here unencoded.
#include "render/image.h"
#include "video/h264.h"
#include "video/rtsp_server.h"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace skysim::video {

class H264Encoder;

// What GET/POST /instances/<n>/camera/encoder read and write.
struct EncoderSettings {
    int bitrate_kbps{1200};
    double fps{10.0};
    double keyframe_interval_s{1.0};
};

// A POST body: only the fields present change.
struct EncoderPatch {
    std::optional<double> bitrate_kbps;
    std::optional<double> fps;
    std::optional<double> keyframe_interval_s;
};

// The bounds a POST is clamped to. The frame rate's ceiling is the camera's render rate
// (--camera-fps): a stream cannot have pictures that were never drawn, and the render rate is
// the CPU budget the deployment chose.
inline constexpr int kMinBitrateKbps = 64;
inline constexpr int kMaxBitrateKbps = 8000;
inline constexpr double kMinFps = 1.0;
inline constexpr double kMinKeyframeIntervalS = 0.1;
inline constexpr double kMaxKeyframeIntervalS = 10.0;

// `s` with `patch` applied and every field clamped (bitrate rounded to whole kbps).
EncoderSettings apply_patch(EncoderSettings s, const EncoderPatch &patch, double max_fps);

class VideoService {
  public:
    using Clock = std::chrono::steady_clock;

    struct Config {
        int width{256}; // the camera's picture size
        int height{144};
        double max_fps{10.0}; // the camera's render rate, the ceiling for a stream's fps
        // A stream's settings until somebody changes them. fps is clamped to max_fps.
        EncoderSettings defaults{};
        std::string preset{"veryfast"};
        int threads{1}; // encoder workers
        bool background_priority{true};
        RtspServer::Config rtsp{};
        // An instance nobody plays keeps its encoder this long, so a receiver that reconnects
        // (ffprobe, then the real pipeline) does not wait for a new one.
        double linger_s{30.0};
        // How often a played stream renews its request for pictures and re-checks which
        // vehicle holds its instance. Well inside the renderer's demand window.
        double refresh_s{0.25};
    };

    struct Hooks {
        // The vehicle now at an instance, 0 when none. Called with no lock of this class held.
        std::function<uint32_t(int instance)> resolve;
        // Keep drawing this vehicle for a stream (RenderService::want_video). Same.
        std::function<void(uint32_t vehicle_id)> want_frames;
    };

    struct Stats {
        int port{0};
        int threads{0};
        size_t streams{0};  // instances being played
        size_t clients{0};  // sessions playing them
        size_t encoders{0}; // open x264 encoders: played streams, and idle ones within their linger
        uint64_t frames_encoded{0};
        uint64_t frames_dropped{0}; // pictures replaced before an encoder took them
        uint64_t keyframes{0};
        uint64_t bytes_encoded{0};
        double encode_ms{0.0}; // mean per frame over the last stats window (about 2 s)
        RtspServer::Stats rtsp{};
    };

    // One instance's stream as the control API reports it.
    struct StreamState {
        EncoderSettings settings;
        int width{0};
        int height{0};
        double max_fps{0.0};
        size_t clients{0};
    };

    // Starts the RTSP server (throws std::runtime_error if it cannot bind) and the workers.
    VideoService(const Config &cfg, Hooks hooks);
    ~VideoService();
    VideoService(const VideoService &) = delete;
    VideoService &operator=(const VideoService &) = delete;

    // Stops the RTSP server and every thread of this service; on_frame() is a no-op after.
    // Must run before whatever the hooks call into goes away. Idempotent.
    void stop();

    // From the render thread: a picture of a vehicle a stream asked for. Never blocks.
    void on_frame(uint32_t vehicle_id, std::shared_ptr<const render::Image> image, double sim_time_s,
                  Clock::time_point drawn);

    // The control API. `vehicle_id` is the vehicle the caller found at `instance`: settings
    // belong to that vehicle, so the next vehicle spawned at the instance starts from defaults.
    StreamState encoder(int instance, uint32_t vehicle_id, const EncoderPatch *patch = nullptr);
    // The next frame of the instance's stream is an IDR. Returns the sessions playing it.
    size_t request_keyframe(int instance, uint32_t vehicle_id);

    Stats stats() const;
    int port() const;

  private:
    struct Stream {
        int instance{0};
        uint32_t vehicle_id{0};     // where its pictures come from while it is played
        uint32_t settings_owner{0}; // the vehicle `settings` were made for
        EncoderSettings settings;
        size_t clients{0};
        Clock::time_point idle_since{};

        // Taken by the worker encoding it (busy) and put back after.
        std::unique_ptr<H264Encoder> encoder;
        bool rebuild{false};   // fps changed: a new encoder
        bool rebitrate{false}; // bitrate changed: reconfigure in place
        bool force_idr{true};
        bool have_idr{false};
        Clock::time_point last_idr{};

        // The one-picture mailbox.
        std::shared_ptr<const render::Image> pending;
        double pending_sim_s{0.0};
        Clock::time_point pending_at{};
        bool queued{false};
        bool busy{false};

        // Frame-rate decimation below the render rate.
        bool have_due{false};
        Clock::time_point next_due{};
    };

    // What an encoder at a frame rate writes as SPS and PPS (they depend on nothing else here).
    struct ParameterSets {
        std::vector<uint8_t> sps;
        std::vector<uint8_t> pps;
        int width{0};
        int height{0};
    };

    Stream &stream_for(int instance);
    void adopt(Stream &s, uint32_t vehicle_id);
    void bind_vehicle(Stream &s, uint32_t vehicle_id);
    void set_settings(Stream &s, const EncoderSettings &next);
    StreamState state_of(const Stream &s) const;
    std::optional<RtspServer::StreamInfo> describe(int instance);
    // From the cache, or made (outside the lock) and cached. Null when x264 refuses the size.
    std::optional<ParameterSets> parameter_sets(double fps);
    void playing(int instance, bool on);
    void worker();
    void housekeeping();

    Config cfg_;
    Hooks hooks_;

    mutable std::mutex mutex_;
    std::condition_variable work_cv_;
    std::condition_variable house_cv_;
    bool stopping_{false};
    uint64_t house_epoch_{0};
    std::unordered_map<int, Stream> streams_; // never erased: references stay valid unlocked
    std::unordered_map<uint32_t, int> by_vehicle_;
    std::map<double, ParameterSets> parameter_sets_; // by fps: few distinct rates ever exist
    std::deque<int> ready_;

    uint64_t frames_encoded_{0};
    uint64_t frames_dropped_{0};
    uint64_t keyframes_{0};
    uint64_t bytes_encoded_{0};
    double encode_ms_{0.0};
    double window_ms_sum_{0.0};
    uint64_t window_frames_{0};
    Clock::time_point window_start_{};

    std::unique_ptr<RtspServer> rtsp_;
    std::vector<std::thread> workers_;
    std::thread house_;
    bool stopped_{false};
};

} // namespace skysim::video
