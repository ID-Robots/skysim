#include "core/cpus.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <sstream>
#include <thread>
#include <vector>

#if defined(__linux__)
#include <sched.h>
#endif

namespace skysim::core {
namespace {

// Whole file, or empty when it cannot be read (no such cgroup, no such controller).
std::string read_file(const std::string &path) {
    std::ifstream f(path);
    if (!f) {
        return {};
    }
    return std::string(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
}

bool read_number(const std::string &path, long long &out) {
    const std::string text = read_file(path);
    if (text.empty()) {
        return false;
    }
    char *end = nullptr;
    out = std::strtoll(text.c_str(), &end, 10);
    return end != text.c_str();
}

// "/a/b/c" -> "/a/b/c", "/a/b", "/a", "": every ancestor of a cgroup, the cgroup first and the
// mount root ("") last. A quota anywhere on that path caps everything below it.
std::vector<std::string> self_and_ancestors(std::string path) {
    while (!path.empty() && (path.back() == '/' || path.back() == '\n' || path.back() == '\r')) {
        path.pop_back();
    }
    std::vector<std::string> out;
    for (;;) {
        out.push_back(path);
        const size_t slash = path.rfind('/');
        if (path.empty() || slash == std::string::npos) {
            break;
        }
        path.erase(slash);
    }
    return out;
}

// The smaller of two quotas where 0 means "no limit".
double tighter(double a, double b) {
    if (a <= 0.0) {
        return b;
    }
    if (b <= 0.0) {
        return a;
    }
    return std::min(a, b);
}

} // namespace

double cgroup_v2_quota_cpus(const std::string &cpu_max) {
    std::istringstream in(cpu_max);
    std::string quota;
    long long period = 0;
    if (!(in >> quota >> period) || quota == "max" || period <= 0) {
        return 0.0;
    }
    char *end = nullptr;
    const long long q = std::strtoll(quota.c_str(), &end, 10);
    if (end == quota.c_str() || *end != '\0' || q <= 0) {
        return 0.0;
    }
    return static_cast<double>(q) / static_cast<double>(period);
}

double cgroup_v1_quota_cpus(long long quota_us, long long period_us) {
    if (quota_us <= 0 || period_us <= 0) {
        return 0.0;
    }
    return static_cast<double>(quota_us) / static_cast<double>(period_us);
}

double cgroup_quota_cpus(const std::string &cgroup_root, const std::string &proc_cgroup) {
    double best = 0.0;
    std::istringstream lines(proc_cgroup);
    std::string line;
    while (std::getline(lines, line)) {
        // hierarchy-id:controllers:path
        const size_t a = line.find(':');
        const size_t b = a == std::string::npos ? std::string::npos : line.find(':', a + 1);
        if (b == std::string::npos) {
            continue;
        }
        const std::string controllers = line.substr(a + 1, b - a - 1);
        const std::string path = line.substr(b + 1);
        if (controllers.empty()) {
            // cgroup v2: one unified tree, cpu.max at every level that sets one.
            for (const std::string &dir : self_and_ancestors(path)) {
                best = tighter(best, cgroup_v2_quota_cpus(read_file(cgroup_root + dir + "/cpu.max")));
            }
            continue;
        }
        // cgroup v1: only the hierarchy carrying the cpu controller. It is mounted under its
        // controller list ("cpu,cpuacct") and usually also linked as "cpu". Inside a container
        // the mount's root is the container's own cgroup, so the host path in /proc/self/cgroup
        // does not exist under it — walking up to the mount root still finds the quota.
        std::istringstream names(controllers);
        std::string name;
        bool has_cpu = false;
        while (std::getline(names, name, ',')) {
            has_cpu = has_cpu || name == "cpu";
        }
        if (!has_cpu) {
            continue;
        }
        for (const std::string &mount : {cgroup_root + "/" + controllers, cgroup_root + "/cpu"}) {
            for (const std::string &dir : self_and_ancestors(path)) {
                long long quota = 0;
                long long period = 0;
                if (read_number(mount + dir + "/cpu.cfs_quota_us", quota) &&
                    read_number(mount + dir + "/cpu.cfs_period_us", period)) {
                    best = tighter(best, cgroup_v1_quota_cpus(quota, period));
                }
            }
        }
    }
    return best;
}

int available_cpus() {
    int cpus = static_cast<int>(std::thread::hardware_concurrency());
    if (cpus <= 0) {
        cpus = 1;
    }
#if defined(__linux__)
    cpu_set_t set;
    CPU_ZERO(&set);
    if (sched_getaffinity(0, sizeof(set), &set) == 0) {
        const int allowed = CPU_COUNT(&set);
        if (allowed > 0) {
            cpus = std::min(cpus, allowed);
        }
    }
    const double quota = cgroup_quota_cpus("/sys/fs/cgroup", read_file("/proc/self/cgroup"));
    if (quota > 0.0) {
        // Rounded down, with a little slack so a quota of exactly 2.0 read as 1.9999999 is 2.
        cpus = std::min(cpus, std::max(1, static_cast<int>(std::floor(quota + 1e-6))));
    }
#endif
    return std::max(1, cpus);
}

int default_render_threads(int cpus) { return std::clamp(cpus - 1, 1, 4); }

} // namespace skysim::core
