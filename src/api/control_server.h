#pragma once
// Control plane for SkyHub: REST (spawn/despawn/list/metrics). The HTTP thread never touches
// world state: writes go through the tick-boundary CommandQueue (promises for replies), reads
// come from snapshots published once per tick (CLAUDE.md invariant 5). WS /state lands at M6.
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include "core/world.h"
#include "vehicle/manager.h"

namespace skysim::api {

struct VehicleInfo {
    uint32_t id = 0;
    int instance = 0;
    int json_port = 0;
    int mavlink_tcp_port = 0;
    bool connected = false; // has ever delivered a servo frame
    bool frozen = false;    // straggler policy engaged (kinematic hold)
    uint64_t held_ticks = 0;
    uint64_t midair_collisions = 0;  // vehicle-vehicle contact events (crash telemetry)
    uint64_t static_contacts = 0;    // any static touch, landings included
    uint64_t building_contacts = 0;  // building-tile strikes only — the "hit a building" signal
    double pos_ned[3] = {0.0, 0.0, 0.0};
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
    };

    // Binds bind_addr:port and serves on its own thread. Throws on bind failure (startup
    // only). Use "0.0.0.0" when the SkyHub gateway calls in from a docker bridge network.
    ControlServer(const std::string &bind_addr, int port, CommandQueue &queue, Snapshots snapshots);
    ~ControlServer();
    ControlServer(const ControlServer &) = delete;
    ControlServer &operator=(const ControlServer &) = delete;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace skysim::api
