#pragma once

#include "core/thread_pool.h"
#include "core/world.h"
#include "render/camera.h"
#include "render/camera_mode.h"
#include "render/dock_marker.h"
#include "render/frame_store.h"
#include "render/ground_cache.h"
#include "render/image.h"
#include "render/raster.h"
#include "render/scene.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
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
// So this draws from its own copy of the static world (a RasterScene, built from the same
// tile directory, or shared with the survey still camera). Buildings and ground are static
// geometry loaded from files, so a private copy is not a copy of anything that changes —
// and it can be read freely while the real world steps, with no lock between them and no
// way for a slow frame to slow a vehicle.
//
// The cost is that other vehicles do not appear in the picture. That is the whole
// trade: ground, sky and buildings, drawn without ever touching the simulation.
//
// Frames are drawn on demand. A vehicle is drawn, at the camera rate, only while somebody is
// asking for its picture — an open MJPEG stream asks continuously, a camera.jpg request once —
// and with nobody asking the render thread sleeps on a condition variable rather than a timer.
// Drawing every vehicle whether or not anyone looked is what had an idle dev server, with one
// parked vehicle slot and no viewer, burning two vCPU on textured 640x360 frames.
//
// Two kinds of asking, each with its own demand window: frame() for JPEGs (the MJPEG and
// camera.jpg routes) and want_video() for pictures handed to video_sink (the H.264 encoders,
// video/video_service.h). A vehicle is drawn when its next frame is due if either wants it; the
// JPEG is made only for the first kind, so a vehicle watched only over RTSP costs no JPEG encode.
//
// Each vehicle has its own picture size (the fleet's, Config::camera, unless configure() gave it
// its own) and draws it in a mode (TASK-2661, camera_mode.h): its normal mode (Config::draw_* and
// Config::fps unless it has its own), the landing mode (Config::landing) while it is in the air
// near its dock, and its normal mode at Config::rest_fps while it sits still on the ground. A mode
// drawn smaller than the picture is scaled up to it (render/resize.h), so every frame of a vehicle
// is the same size. Each vehicle is drawn on its own schedule, so a full-size landing at 15 fps
// and a fleet drawn at a third of the size at 30 share the threads without one setting the pace
// of the others. The lit ground comes from a cache shared by every vehicle
// (Config::ground_cache_bytes, render/ground_cache.h).
class RenderService {
  public:
    struct Config {
        CameraConfig camera;
        // The scene to draw. When null the service loads its own from tiles_dir, textured
        // with texture_seed; when shared, scene_tiles says how many tiles it holds.
        std::shared_ptr<const RasterScene> scene;
        size_t scene_tiles{0};
        // The dock markers on the ground (TASK-2562); none drawn when null. A vehicle's picture
        // shows the docks DockBoard::seen_by gives it: by default its own alone (TASK-2609).
        std::shared_ptr<const DockBoard> docks;
        std::string tiles_dir;
        uint32_t texture_seed{1};
        // The fleet's normal mode: drawn at draw_width x draw_height (0: camera.width x
        // camera.height, the picture size) at `fps`, unless a vehicle has its own.
        double fps{10.0};
        int draw_width{0};
        int draw_height{0};
        // A vehicle at rest on the ground (settle_s, below) is drawn at this rate instead of its
        // mode's (0: no rest rate). Its picture does not change, so this only saves re-sending —
        // and re-encoding — the same picture: a parked vehicle somebody reads at 30 fps costs a
        // full-size H.264 encode 30 times a second for nothing. Its stream keeps its frame rate.
        double rest_fps{0.0};
        int quality{70};
        // The landing mode (TASK-2661): a vehicle within landing_radius_m of its dock (its
        // DockBoard dock; where it spawned, Pose::home, without one) and at most landing_height_m
        // above the ground, once it is off the ground (0.12 m up), draws `landing` instead of its
        // normal mode until it leaves the zone or comes to rest on the ground (its size 0:
        // the picture size). Leaving takes landing_margin_m more of either, so a vehicle on the
        // boundary does not flap. Off while landing.fps is 0 (a vehicle can still be given one).
        CameraMode landing{};
        double landing_radius_m{25.0};
        double landing_height_m{40.0};
        double landing_margin_m{5.0};
        // A vehicle whose pose has not moved by 2 cm or 0.5 degrees, gimbal included, for this long
        // is at rest: parked on its dock it hands the landing camera back. A vehicle is at rest
        // when it first appears, so one spawned on its dock starts with its normal camera.
        double settle_s{5.0};
        // Bytes of lit ground the camera may remember (render/ground_cache.h); 0 computes every
        // ground pixel, as the stills do.
        size_t ground_cache_bytes{0};
        // Threads drawing a frame, the service's own thread included: 1 draws on that thread
        // alone, n adds a pool of n - 1 workers. So `threads` is the most CPUs a frame takes.
        int threads{4};
        // A vehicle stays on the draw list for this long after the last request for its
        // picture. Longer than a poll-based viewer's interval, short enough that a closed
        // stream stops costing CPU within seconds.
        double demand_window_s{3.0};
        // Run the render thread and its workers at nice +10, as the still camera's are: on a
        // box short of cores the physics tick then wins every contest for a CPU.
        bool background_priority{true};
        // Seconds between "camera:" log lines while frames are being drawn. When drawing stops,
        // the frames since the last line and an "idle" line follow once that long has passed
        // since it (so never more than one line a period); 0 = never.
        double log_every_s{0.0};
        // Where those lines go, one call per line without the newline; stdout when unset.
        // Called on the render thread.
        std::function<void(const char *line)> log_sink;
        // Where pictures go for a vehicle somebody's video stream asked for (want_video): the
        // RGB picture itself, shared rather than copied, with the sim time of its pose and the
        // moment it was drawn (or re-sent unchanged). Called on the render thread, once per pass
        // per vehicle; it must hand the picture off and return, never encode it there.
        // `mode` is the camera mode it was drawn in (its size is the picture's; its frame rate and
        // bitrate are what the stream should run at).
        std::function<void(uint32_t vehicle_id, std::shared_ptr<const Image> image, double sim_time_s,
                           std::chrono::steady_clock::time_point drawn, const CameraMode &mode)>
            video_sink;
    };

    // One vehicle's pose, as the tick thread last published it.
    struct Pose {
        uint32_t vehicle_id{0};
        core::Vec3 position_ned{};
        core::Quat quat_ned_frd{1.0, 0.0, 0.0, 0.0};
        // When the tick that produced this pose happened, carried through to the
        // frame so a consumer can tell a fresh picture from a stalled one.
        double sim_time_s{0.0};
        // The vehicle's gimbal as its last servo frame drove it (TASK-2548). With one, the
        // pilot camera looks where it points (stabilised: heading + yaw, its pitch, no roll)
        // instead of along the nose at the fixed mount pitch.
        bool gimbal{false};
        double gimbal_pitch_rad{0.0};
        double gimbal_yaw_rad{0.0};
        // Where the vehicle spawned (NED, metres): the dock the landing camera measures from when
        // there is no DockBoard (TASK-2661).
        double home_north_m{0.0};
        double home_east_m{0.0};
    };

    // What the camera is achieving, over the last completed stats window (about two seconds).
    struct Stats {
        double target_fps{0.0};
        int threads{0};
        size_t watched{0};         // vehicles on the draw list in the last pass
        double fps{0.0};           // frames per second each watched vehicle received
        double render_ms{0.0};     // mean time to rasterise one frame
        double render_ms_max{0.0}; // the slowest frame
        double encode_ms{0.0};     // mean JPEG encode per frame
        uint64_t frames_drawn{0};  // since start
        uint64_t frames_reused{0}; // since start: the pose had not moved, so the last picture was re-sent
        size_t landing{0};         // vehicles drawing the landing camera now
        GroundCache::Stats ground; // the lit-ground cache (all zero without one)
    };

    // A vehicle's own camera settings (render/camera_mode.h).
    using CameraPatch = CameraSettingsPatch;
    using VehicleCamera = VehicleCameraState;

    explicit RenderService(const Config &cfg);
    ~RenderService();
    RenderService(const RenderService &) = delete;
    RenderService &operator=(const RenderService &) = delete;

    // Called from the tick thread once per tick. Cheap: copies a handful of
    // doubles under a mutex and returns.
    void publish_poses(std::vector<Pose> poses);

    // The latest frame for a vehicle — and a request for more: calling this is what puts the
    // vehicle on the draw list, for demand_window_s. With `wait` > 0 and nothing newer than
    // `after_seq` yet, blocks up to `wait` for the render thread to draw one; a vehicle that
    // is not in the fleet never gets one, so for that it returns at once. Empty until a frame
    // has been drawn, and again once nobody has asked for one in a while (a picture left over
    // from minutes ago is not the camera's view).
    FrameStore::Frame frame(uint32_t vehicle_id, uint64_t after_seq = 0,
                            std::chrono::milliseconds wait = std::chrono::milliseconds(0));

    // A request for a vehicle's pictures on behalf of a video stream: like frame() it puts the
    // vehicle on the draw list for demand_window_s, but what is drawn goes to Config::video_sink
    // as a picture, and no JPEG is made for it unless frame() is asking too. Returns at once;
    // false (and nothing recorded) for a vehicle that is not in the fleet.
    bool want_video(uint32_t vehicle_id);

    Stats stats() const;
    int threads() const { return threads_; }

    // A vehicle's own camera settings (TASK-2661). Returns the camera after the change, or nothing
    // with `error` set when a field is out of bounds (camera_mode.h; nothing is changed then).
    // Settings may be made before the vehicle's first pose arrives (at spawn) and last until
    // forget().
    std::optional<VehicleCamera> configure(uint32_t vehicle_id, const CameraPatch &patch, std::string *error);
    // Whether configure() would take `patch` for a vehicle that has no settings of its own yet
    // (a spawn's): the same checks, nothing changed.
    bool check(const CameraPatch &patch, std::string *error) const;
    // The vehicle's camera: its modes and what it draws now.
    VehicleCamera camera(uint32_t vehicle_id) const;
    // The mode it draws in now (sizes resolved; at rest, the rest rate).
    CameraMode mode_now(uint32_t vehicle_id) const;
    // What its stream carries now: its picture size, its mode's frame rate (at rest too: resting
    // changes how often a frame comes, not the stream) and bitrate. For the H.264 encoder.
    CameraMode stream_mode(uint32_t vehicle_id) const;
    // The vehicle despawned: its settings go with it.
    void forget(uint32_t vehicle_id);
    // The fleet's normal mode (its size 0: the picture size).
    CameraMode fleet_mode() const { return {cfg_.draw_width, cfg_.draw_height, cfg_.fps, 0}; }
    const Config &config() const { return cfg_; }

    size_t tiles_loaded() const { return tiles_loaded_; }
    size_t triangle_count() const { return scene_->triangle_count(); }

  private:
    using Clock = std::chrono::steady_clock;

    // Running totals for one pass, or for one stats or log window of passes.
    struct Window {
        Clock::time_point started{}; // the first pass's start
        uint64_t passes{0};
        // Pass-to-pass steps within runs of drawing, the seconds they took and the frames the
        // passes that ended them delivered: the rate is delivered / active_s per vehicle, so idle
        // gaps between viewers do not dilute it.
        uint64_t intervals{0};
        double active_s{0.0};
        uint64_t delivered{0};
        double vehicle_s{0.0}; // active_s weighted by the vehicles watched: the fps denominator
        uint64_t drawn{0};
        uint64_t reused{0};
        double render_ms_sum{0.0};
        double render_ms_max{0.0};
        double encode_ms_sum{0.0};
        uint64_t encoded{0}; // JPEGs made (a re-sent picture can need one for a new viewer)
        size_t watched{0};   // the most watched at once
        size_t landing{0};   // the most drawing the landing camera at once
        // Frames each watched vehicle got per second of drawing: every frame delivered (drawn or
        // re-sent) over the time spent drawing and the vehicles watched. With every vehicle at one
        // rate that is the rate each stream sees.
        double fps() const;
    };

    void run();
    void record_pass(const Window &pass, Clock::time_point now);
    // The vehicles on the draw list are `watched` now, at a loop turn that may or may not draw
    // anything. When that is not the number the last pass (or the last such change) saw, the time
    // since was spent on that number: it is booked at once, so a demand that lapses between two
    // passes — with no frame due to end the step there — does not count for the whole step.
    void note_watched(Clock::time_point now, size_t watched);
    void go_idle();
    Clock::duration log_period() const;
    // When the deferred "camera: idle" line (with the frames since the last line) is due;
    // time_point::max() when there is none.
    Clock::time_point idle_log_due() const;
    void flush_idle_log(Clock::time_point now);
    void log_window(const Window &w) const;
    void emit(const char *line) const;
    bool in_fleet(uint32_t vehicle_id) const;

    // One vehicle's settings and what the render thread last decided for it, under settings_mutex_.
    struct Setting {
        std::optional<std::pair<int, int>> picture; // its own picture size
        uint64_t picture_revision{0};               // picture size changes so far (configure())
        std::optional<CameraMode> normal;           // its own
        std::optional<CameraMode> landing; // its own
        bool landing_enabled{true};
        bool landing_active{false};
        bool resting{false};
        bool watched{false};
        double fps{0.0};
        double render_ms{0.0};
        uint64_t drawn{0};
        uint64_t reused{0};
    };
    std::pair<int, int> picture_of(const Setting *s) const;
    CameraMode normal_of(const Setting *s) const;
    std::optional<CameraMode> landing_of(const Setting *s) const;
    VehicleCamera describe(const Setting *s) const;
    // `patch` applied to `next`, checked (bounds, aspect ratio, a size inferred from one side); false
    // with `error` set, and `next` then half-changed, when it does not hold.
    bool apply(Setting &next, const CameraPatch &patch, std::string *error) const;
    // Records a request in `demand` (under demand_mutex_), waking the render thread for a new one.
    void note_demand(std::unordered_map<uint32_t, Clock::time_point> &demand, uint32_t vehicle_id,
                     Clock::time_point now);

    Config cfg_;
    int threads_{1};
    std::shared_ptr<const RasterScene> scene_;
    std::unique_ptr<core::ThreadPool> pool_;
    std::unique_ptr<GroundCache> ground_;
    FrameStore frames_;

    mutable std::mutex settings_mutex_;
    std::unordered_map<uint32_t, Setting> settings_;
    std::unordered_set<uint32_t> retired_; // forgotten vehicles: configure() refuses them
    size_t tiles_loaded_{0};

    mutable std::mutex poses_mutex_;
    std::vector<Pose> poses_;
    // Whether poses_ holds a vehicle: what the render thread, waiting for somebody to ask, watches
    // for to start following the fleet's modes (publish_poses wakes it when the first one arrives).
    std::atomic<bool> fleet_present_{false};

    // Who asked for which vehicle's picture, and when. A new entry bumps demand_epoch_ and
    // wakes the render thread if it is waiting for work.
    std::mutex demand_mutex_;
    std::condition_variable wake_;
    std::unordered_map<uint32_t, Clock::time_point> demand_;       // JPEG: camera.jpg / .mjpg
    std::unordered_map<uint32_t, Clock::time_point> video_demand_; // pictures: video streams
    uint64_t demand_epoch_{0};
    std::atomic<bool> stop_{false}; // set under demand_mutex_, so a waiter cannot miss it

    mutable std::mutex stats_mutex_;
    Stats stats_;
    Window stats_window_;
    Window log_window_;
    Clock::time_point last_log_{}; // the last "camera:" line, to keep them a log period apart
    Clock::time_point burst_prev_{}; // the latest pass's start, while drawing continues
    size_t burst_prev_watched_{0};   // the vehicles it watched: who the time since then was spent on
    bool in_burst_{false};
    bool idle_line_pending_{false};

    std::thread thread_;
};

} // namespace skysim::render
