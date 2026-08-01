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

// The pack a vehicle carries, as the discharge model sees it.
//
// This lives here rather than as a file-scope constant because the pack is the single
// biggest determinant of how a simulated fleet behaves: endurance sets sortie length,
// sortie length sets how often relief happens, and relief is the whole point of a rotating
// survey. A fleet standing in for 6S 27 Ah Observers cannot be studied on the 3S 3.3 Ah
// hobby pack that used to be compiled in.
//
// Worth being explicit about why this is skysim's business at all: ArduPilot's own
// SIM_BATT_* parameters do nothing for a `--model json` vehicle. The JSON backend takes
// voltage and current from the simulator, so whatever the autopilot has been told about
// its pack is ignored in favour of these numbers. Setting SIM_BATT_CAP_AH on a
// skysim-backed SITL reads back correctly and changes nothing at all.
struct BatteryPack {
    double capacity_mah = 3300.0;
    double full_v = 12.6;
    double empty_v = 10.2;
    double idle_a = 0.7;   // avionics, receivers, companion board
    double hover_a = 42.0; // draw at a mid-throttle hover for this airframe

    // Defaults describe a 3S hobby pack, kept so an unconfigured spawn behaves exactly as
    // it did before the pack became a per-vehicle property.
};

struct SpawnRequest {
    std::string frame = "quad_x"; // only frame supported until per-frame TOML lands (M5+)
    bool launch_process = false;  // true => manager spawns an arducopter for this slot
    int instance = -1;            // >=0: caller-chosen instance (SkyHub gateway numbering);
                                  // -1: lowest free (skysim allocates)
    BatteryPack battery{};        // omitted in the request => the defaults above
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
    // Claim a specific instance (external allocator, e.g. the SkyHub gateway). False if
    // already in use or below base_instance.
    bool allocate_instance_at(int instance);
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
