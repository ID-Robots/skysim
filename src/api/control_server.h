#pragma once
// Control plane for SkyHub: REST (spawn/despawn/list/metrics). The HTTP thread never touches
// world state: writes go through the tick-boundary CommandQueue (promises for replies), reads
// come from snapshots published once per tick (CLAUDE.md invariant 5). WS /state lands at M6.
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <variant>
#include <vector>

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
    uint64_t midair_collisions = 0; // vehicle-vehicle contact events (crash telemetry)
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

using Command = std::variant<SpawnCommand, DespawnCommand>;

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

    // Binds 127.0.0.1:port and serves on its own thread. Throws on bind failure (startup only).
    ControlServer(int port, CommandQueue &queue, Snapshots snapshots);
    ~ControlServer();
    ControlServer(const ControlServer &) = delete;
    ControlServer &operator=(const ControlServer &) = delete;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace skysim::api
