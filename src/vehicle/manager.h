#pragma once
// Vehicle lifecycle: instance/port allocation, optional arducopter process launch, body
// creation. All mutations executed at tick boundaries via the world command queue (M4).
#include <cstdint>
#include <string>

namespace skysim::vehicle {

struct SpawnRequest {
    std::string frame = "quad_x";     // resolves to a params TOML
    double home_lat, home_lon, home_alt_m, home_yaw_deg;
    bool launch_process = true;       // false => external/remote SITL will connect
    std::string ardupilot_binary;     // used when launch_process
};

struct SpawnResult { uint32_t id; int instance; int json_port; int mavlink_tcp_port; };

// TODO(M4): class VehicleManager { spawn(), despawn(id), list(), straggler bookkeeping };
// instance allocator must be leak-free across 100 spawn/despawn cycles (M4 acceptance).

}  // namespace skysim::vehicle
