#pragma once
// Vehicle lifecycle: instance/port allocation, optional arducopter process launch. All fleet
// mutations are executed at tick boundaries via the app command queue (CLAUDE.md invariant 5);
// this class only allocates instances and manages child processes.
#include <cstdint>
#include <optional>
#include <string>
#include <sys/types.h>
#include <vector>

namespace skysim::vehicle {

struct SpawnRequest {
    std::string frame = "quad_x"; // only frame supported until per-frame TOML lands (M5+)
    bool launch_process = false;  // true => manager spawns an arducopter for this slot
};

struct SpawnResult {
    uint32_t id;
    int instance;
    int json_port;
    int mavlink_tcp_port;
};

// Launch config for managed arducopter processes (used when launch_process).
struct ProcessConfig {
    std::string binary;   // $ARDUPILOT_ROOT/build/sitl/bin/arducopter
    std::string home;     // lat,lon,alt,yaw
    std::string defaults; // comma-separated .parm list
    std::string run_dir;  // per-instance cwd created beneath this
};

class VehicleManager {
  public:
    explicit VehicleManager(int base_instance, ProcessConfig process_cfg)
        : base_instance_(base_instance), process_cfg_(std::move(process_cfg)) {}
    ~VehicleManager();

    // Lowest free instance number (leak-free across spawn/despawn cycles — M4 acceptance).
    int allocate_instance();
    void release_instance(int instance);
    size_t instances_in_use() const;

    // Fork/exec an arducopter for `instance`; returns pid or nullopt on failure.
    std::optional<pid_t> launch_sitl(int instance);
    void stop_sitl(pid_t pid);                           // SIGTERM, reaped asynchronously via reap()
    void reap();                                         // call once per tick: waitpid(WNOHANG) zombies
    size_t children() const { return children_.size(); } // unreaped managed processes

  private:
    int base_instance_;
    ProcessConfig process_cfg_;
    std::vector<bool> used_; // index = instance - base_instance
    std::vector<pid_t> children_;
};

} // namespace skysim::vehicle
