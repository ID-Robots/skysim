#pragma once

#include "core/thread_pool.h"
#include "core/world.h"
#include "render/camera.h"
#include "render/frame_store.h"
#include "render/image.h"
#include "render/raster.h"
#include "render/scene.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
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
class RenderService {
  public:
    struct Config {
        CameraConfig camera;
        // The scene to draw. When null the service loads its own from tiles_dir, textured
        // with texture_seed; when shared, scene_tiles says how many tiles it holds.
        std::shared_ptr<const RasterScene> scene;
        size_t scene_tiles{0};
        std::string tiles_dir;
        uint32_t texture_seed{1};
        double fps{10.0};
        int quality{70};
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
        uint64_t frames_reused{0}; // since start: the pose had not moved, so the last JPEG was re-sent
    };

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

    Stats stats() const;
    int threads() const { return threads_; }

    size_t tiles_loaded() const { return tiles_loaded_; }
    size_t triangle_count() const { return scene_->triangle_count(); }

  private:
    using Clock = std::chrono::steady_clock;

    // Running totals for one pass, or for one stats or log window of passes.
    struct Window {
        Clock::time_point started{}; // the first pass's start
        uint64_t passes{0};
        // Pass-to-pass steps within runs of drawing, and the seconds they took: the rate is
        // intervals / active_s, so idle gaps between viewers do not dilute it.
        uint64_t intervals{0};
        double active_s{0.0};
        uint64_t drawn{0};
        uint64_t reused{0};
        double render_ms_sum{0.0};
        double render_ms_max{0.0};
        double encode_ms_sum{0.0};
        size_t watched{0}; // the most watched at once
        double fps() const;
    };

    void run();
    void record_pass(const Window &pass, Clock::time_point now);
    void go_idle();
    Clock::duration log_period() const;
    // When the deferred "camera: idle" line (with the frames since the last line) is due;
    // time_point::max() when there is none.
    Clock::time_point idle_log_due() const;
    void flush_idle_log(Clock::time_point now);
    void log_window(const Window &w) const;
    void emit(const char *line) const;

    Config cfg_;
    int threads_{1};
    std::unique_ptr<Rasterizer> rasterizer_;
    std::shared_ptr<const RasterScene> scene_;
    std::unique_ptr<core::ThreadPool> pool_;
    FrameStore frames_;
    size_t tiles_loaded_{0};

    mutable std::mutex poses_mutex_;
    std::vector<Pose> poses_;

    // Who asked for which vehicle's picture, and when. A new entry bumps demand_epoch_ and
    // wakes the render thread if it is waiting for work.
    std::mutex demand_mutex_;
    std::condition_variable wake_;
    std::unordered_map<uint32_t, Clock::time_point> demand_;
    uint64_t demand_epoch_{0};
    std::atomic<bool> stop_{false}; // set under demand_mutex_, so a waiter cannot miss it

    mutable std::mutex stats_mutex_;
    Stats stats_;
    Window stats_window_;
    Window log_window_;
    Clock::time_point last_log_{}; // the last "camera:" line, to keep them a log period apart
    Clock::time_point burst_prev_{}; // the latest pass's start, while drawing continues
    bool in_burst_{false};
    bool idle_line_pending_{false};

    std::thread thread_;
};

} // namespace skysim::render
