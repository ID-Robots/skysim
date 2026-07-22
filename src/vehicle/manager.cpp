// See manager.h.
#include "vehicle/manager.h"

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>

namespace skysim::vehicle {

VehicleManager::~VehicleManager() {
    for (pid_t pid : children_) {
        ::kill(pid, SIGTERM);
    }
    for (pid_t pid : children_) {
        int status = 0;
        ::waitpid(pid, &status, 0);
    }
}

int VehicleManager::allocate_instance() {
    for (size_t i = 0; i < used_.size(); ++i) {
        if (!used_[i]) {
            used_[i] = true;
            return base_instance_ + static_cast<int>(i);
        }
    }
    used_.push_back(true);
    return base_instance_ + static_cast<int>(used_.size()) - 1;
}

void VehicleManager::release_instance(int instance) {
    const size_t idx = static_cast<size_t>(instance - base_instance_);
    if (idx < used_.size()) {
        used_[idx] = false;
    }
}

size_t VehicleManager::instances_in_use() const {
    return static_cast<size_t>(std::count(used_.begin(), used_.end(), true));
}

std::optional<pid_t> VehicleManager::launch_sitl(int instance) {
    const std::string run_dir = process_cfg_.run_dir + "/I" + std::to_string(instance);
    std::error_code ec;
    std::filesystem::create_directories(run_dir, ec);

    const pid_t pid = ::fork();
    if (pid < 0) {
        return std::nullopt;
    }
    if (pid == 0) {
        // Child: exec arducopter with its own cwd (it drops eeprom.bin/logs there).
        if (::chdir(run_dir.c_str()) != 0) {
            std::_Exit(127);
        }
        const std::string inst = std::to_string(instance);
        ::execl(process_cfg_.binary.c_str(), process_cfg_.binary.c_str(), "--model", "json:127.0.0.1", "-I",
                inst.c_str(), "--home", process_cfg_.home.c_str(), "--defaults",
                process_cfg_.defaults.c_str(), static_cast<char *>(nullptr));
        std::_Exit(127); // exec failed
    }
    children_.push_back(pid);
    return pid;
}

void VehicleManager::stop_sitl(pid_t pid) { ::kill(pid, SIGTERM); }

void VehicleManager::reap() {
    for (auto it = children_.begin(); it != children_.end();) {
        int status = 0;
        const pid_t r = ::waitpid(*it, &status, WNOHANG);
        if (r == *it) {
            it = children_.erase(it);
        } else {
            ++it;
        }
    }
}

} // namespace skysim::vehicle
