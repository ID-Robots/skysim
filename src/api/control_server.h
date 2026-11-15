#pragma once
// Control plane for SkyHub: REST (spawn/despawn/list/metrics, simulated radio field). The HTTP
// thread never touches world state: writes go through the tick-boundary CommandQueue (promises
// for replies), reads come from snapshots published once per tick (CLAUDE.md invariant 5).
// WS /state lands at M6.
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "core/world.h"
#include "radio/radio_model.h"
#include "render/frame_store.h"
#include "render/still_request.h"
#include "vehicle/manager.h"

namespace skysim::api {

struct VehicleInfo {
    uint32_t id = 0;
    int instance = 0;
    int json_port = 0;
    int mavlink_tcp_port = 0;
    bool connected = false; // has ever delivered a servo frame
    bool frozen = false;    // straggler policy engaged (kinematic hold)
    uint64_t silent_ticks = 0;
    uint64_t midair_collisions = 0;  // vehicle-vehicle contact events (crash telemetry)
    uint64_t static_contacts = 0;    // any static touch, landings included
    uint64_t building_contacts = 0;  // building-tile strikes only — the "hit a building" signal
    double pos_ned[3] = {0.0, 0.0, 0.0};
    // Attitude (FRD -> NED, w x y z) and the sim time of the tick this snapshot is from:
    // with pos_ned, the exact pose a survey still is drawn from and stamped with.
    double quat_ned_frd[4] = {1.0, 0.0, 0.0, 0.0};
    double sim_time_s = 0.0;
    // Its gimbal as the same tick's servo frame drove it (TASK-2548; vehicle/gimbal.h).
    bool gimbal = false;
    double gimbal_pitch_rad = 0.0;
    double gimbal_yaw_rad = 0.0;
};

// The pilot camera's achieved rate and cost (RenderService::Stats), so a laggy stream can be
// read off /metrics instead of guessed at.
struct CameraMetrics {
    bool enabled = false; // --camera-fps given; the rest is meaningless without it
    double target_fps = 0.0;
    double fps = 0.0;  // frames per second each watched vehicle is getting
    size_t watched = 0; // vehicles being drawn because somebody is asking for them
    double render_ms = 0.0;
    double render_ms_max = 0.0;
    double encode_ms = 0.0;
    int threads = 0;
    uint64_t frames_drawn = 0;
    uint64_t frames_reused = 0;
};

// The pilot camera's H.264/RTSP streams (TASK-2561; video/video_service.h), so their cost and
// health can be read off /metrics. Null there when RTSP video is off.
struct VideoMetrics {
    bool enabled = false;
    int rtsp_port = 0;
    size_t streams = 0; // instances being played
    size_t clients = 0; // RTSP sessions playing them
    int threads = 0;    // encoder workers
    double encode_ms = 0.0;
    uint64_t frames_encoded = 0;
    uint64_t frames_dropped = 0; // pictures an encoder or a receiver could not take in time
    uint64_t keyframes = 0;
    uint64_t bytes_sent = 0; // RTP + RTCP to every receiver
};

// GET/POST /instances/<n>/camera/encoder: what a POST may change (absent fields keep their
// value) and what both answer with — the applied values, after clamping.
struct EncoderPatch {
    std::optional<double> bitrate_kbps;
    std::optional<double> fps;
    std::optional<double> keyframe_interval_s;
};

struct EncoderState {
    int bitrate_kbps = 0;
    double fps = 0.0;
    double keyframe_interval_s = 0.0;
    int width = 0; // the stream's picture size (the camera's, rounded down to even)
    int height = 0;
    double max_fps = 0.0; // the ceiling fps is clamped to: the camera's render rate
    size_t clients = 0;   // RTSP sessions playing the stream now
};

struct MetricsInfo {
    double tick_p50_us = 0.0;
    double tick_p99_us = 0.0;
    uint64_t straggler_events = 0;
    uint64_t freezes = 0;
    uint64_t tick = 0;
    double sim_time_s = 0.0;
    size_t vehicles = 0;
    size_t resident_tiles = 0; // streamed-in city tiles (M5 memory bound observable)
    CameraMetrics camera;
    VideoMetrics video;
};

struct SpawnCommand {
    vehicle::SpawnRequest request;
    std::promise<std::optional<vehicle::SpawnResult>> done;
};

struct DespawnCommand {
    uint32_t id = 0;
    std::promise<bool> done;
};

// A fresh pack on an existing vehicle. Docked operations swap batteries rather than
// recharging in place, so without this a simulated fleet can only ever fly one sortie —
// and the second sortie is where handover and turnaround bugs actually live.
struct BatteryResetCommand {
    uint32_t id = 0;
    std::promise<bool> done;
};

// A planned mission path swept through the world's building geometry. Runs on the tick
// thread like any other world access, and answers in microseconds — the pre-flight
// question is "would this mission hit a building?", which a real flight, even sped up,
// is far too slow to answer while an operator waits.
struct PathCheckResult {
    std::vector<core::World::PathHit> hits;
    // False when there is no building geometry to sweep against. Reported rather than
    // folded into an empty hit list, because "nothing loaded" must never be presented
    // to an operator as "the path is clear".
    bool geometry_available = false;
};

struct PathCheckCommand {
    std::vector<core::Vec3> waypoints_ned;
    double clearance_m = 1.0;
    std::promise<PathCheckResult> done;
};

using Command = std::variant<SpawnCommand, DespawnCommand, PathCheckCommand, BatteryResetCommand>;

// Mutex-protected handoff HTTP thread -> tick thread; drained at tick boundaries only.
class CommandQueue {
  public:
    void push(Command &&c) {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_.push_back(std::move(c));
    }
    std::vector<Command> drain() {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<Command> out;
        out.swap(pending_);
        return out;
    }

  private:
    std::mutex mutex_;
    std::vector<Command> pending_;
};

class ControlServer {
  public:
    struct Snapshots {
        std::function<std::vector<VehicleInfo>()> vehicles;
        std::function<MetricsInfo()> metrics;
        // Latest camera frame for a vehicle as encoded JPEG, with the sim time it was drawn at
        // and its sequence number; empty when the camera is off or that vehicle has not been
        // rendered yet. Asking is also what gets a vehicle drawn: the camera renders only the
        // vehicles somebody is asking for. With a wait, blocks up to that long for a frame
        // newer than after_seq — how an MJPEG route sends each frame once, as it lands, without
        // polling the slot on a timer.
        std::function<skysim::render::FrameStore::Frame(uint32_t id, uint64_t after_seq,
                                                         std::chrono::milliseconds wait)>
            camera_frame;
        // Survey still camera: render one frame from this vehicle's snapshot pose. Null when
        // the still camera is off, and the still routes answer 404. Called on an HTTP thread;
        // it must not touch world state (it gets everything it needs in the VehicleInfo).
        std::function<skysim::render::StillFrame(const VehicleInfo &, const skysim::render::StillRequest &)>
            still;
        // The H.264 encoder behind an instance's RTSP stream (TASK-2561): its settings, or — with
        // a patch — the settings after applying it, clamped. Null when RTSP video is off, and the
        // encoder and keyframe routes answer 404.
        std::function<EncoderState(const VehicleInfo &, const EncoderPatch *patch)> camera_encoder;
        // Makes the stream's next frame an IDR; returns the sessions playing it.
        std::function<size_t(const VehicleInfo &)> camera_keyframe;
        // Serve the camera.mjpg routes. On by default so SITL images keep streaming MJPEG until
        // every consumer reads RTSP; off (--camera-mjpeg 0), they answer 404 naming the RTSP URL.
        bool mjpeg = true;
    };

    // The simulated radio field behind /radio (docs/RADIO.md). It is not world state — no
    // tick reads it — so the HTTP threads own it outright: PUT swaps the model, and every
    // read samples whichever model it took, without touching the tick thread.
    struct RadioConfig {
        // Shared with whoever loaded a spec at startup; null makes the server keep its own.
        std::shared_ptr<radio::RadioField> field;
        // Where the world's NED origin is on the earth (the spawn home and its ground AMSL).
        // Without it GET /vehicles cannot place a vehicle in the field and carries no radio,
        // and a survey still carries no X-Skysim-Lat / -Lon / -Alt-Amsl.
        std::optional<radio::GeoAnchor> home;
    };

    // Largest raster GET /radio/grid computes: rows × columns × carriers.
    static constexpr size_t kMaxGridCells = 250000;
    // Rasters computed at once. One at the cap costs about a core-second; the HTTP pool is
    // sized for camera streams, and without a bound a burst of grid requests could take
    // every core the physics tick needs. Requests past it get 503.
    static constexpr int kMaxGridsInFlight = 2;

    // Binds bind_addr:port and serves on its own thread. Throws on bind failure (startup
    // only). Use "0.0.0.0" when the SkyHub gateway calls in from a docker bridge network.
    ControlServer(const std::string &bind_addr, int port, CommandQueue &queue, Snapshots snapshots,
                  RadioConfig radio = {});
    ~ControlServer();
    ControlServer(const ControlServer &) = delete;
    ControlServer &operator=(const ControlServer &) = delete;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace skysim::api
