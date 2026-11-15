#include "core/cpus.h"

#include <algorithm>
#include <cctype>
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

std::vector<CgroupMount> cgroup_mounts(const std::string &mountinfo) {
    // mountinfo escapes space, tab, newline and backslash in paths as \ooo.
    const auto unescape = [](const std::string &in) {
        std::string out;
        for (size_t i = 0; i < in.size(); ++i) {
            const auto digit = [&in](size_t k) {
                return std::isdigit(static_cast<unsigned char>(in[k])) != 0;
            };
            if (in[i] == '\\' && i + 3 < in.size() && digit(i + 1) && digit(i + 2) && digit(i + 3)) {
                out.push_back(static_cast<char>(std::strtol(in.substr(i + 1, 3).c_str(), nullptr, 8)));
                i += 3;
            } else {
                out.push_back(in[i]);
            }
        }
        return out;
    };
    std::vector<CgroupMount> mounts;
    std::istringstream lines(mountinfo);
    std::string line;
    while (std::getline(lines, line)) {
        // id parent major:minor root mount-point options [optional...] - fstype source super-options
        std::istringstream in(line);
        std::vector<std::string> fields;
        for (std::string f; in >> f;) {
            fields.push_back(f);
        }
        size_t sep = 6;
        while (sep < fields.size() && fields[sep] != "-") {
            ++sep;
        }
        if (sep + 1 >= fields.size()) {
            continue; // no "- fstype ..." part: not a line this can read
        }
        const std::string &fstype = fields[sep + 1];
        if (fstype != "cgroup2" && fstype != "cgroup") {
            continue;
        }
        CgroupMount m;
        m.root = unescape(fields[3]);
        m.mount_point = unescape(fields[4]);
        m.v2 = fstype == "cgroup2";
        if (!m.v2 && sep + 3 < fields.size()) {
            std::istringstream opts(fields[sep + 3]);
            for (std::string o; std::getline(opts, o, ',');) {
                m.has_cpu = m.has_cpu || o == "cpu";
            }
        }
        mounts.push_back(std::move(m));
    }
    return mounts;
}

namespace {

// The directories under a mount that hold cgroup `path` and each of its ancestors, the
// process's own cgroup first and the mount point last. `path` names the cgroup from the
// hierarchy's root, but a mount can show only a subtree (its mountinfo root): the cgroup
// /tenant/app under a mount of /tenant is <mount>/app, not <mount>/tenant/app. A cgroup outside
// the subtree a mount shows (a container's view of a host path, a cgroup namespace's "/..")
// has only the mount's own root visible, which is then the tightest limit there is to read.
std::vector<std::string> visible_dirs(const CgroupMount &m, const std::string &path) {
    std::string root = m.root;
    while (!root.empty() && root.back() == '/') {
        root.pop_back();
    }
    std::string rel;
    if (root.empty()) {
        rel = path;
    } else if (path == root || path.rfind(root + "/", 0) == 0) {
        rel = path.substr(root.size());
    }
    std::vector<std::string> dirs;
    for (const std::string &dir : self_and_ancestors(rel)) {
        dirs.push_back(m.mount_point + dir);
    }
    return dirs;
}

} // namespace

double cgroup_quota_cpus(const std::vector<CgroupMount> &mounts, const std::string &proc_cgroup) {
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
            for (const CgroupMount &m : mounts) {
                if (m.v2) {
                    for (const std::string &dir : visible_dirs(m, path)) {
                        best = tighter(best, cgroup_v2_quota_cpus(read_file(dir + "/cpu.max")));
                    }
                }
            }
            continue;
        }
        // cgroup v1: only the hierarchy carrying the cpu controller.
        std::istringstream names(controllers);
        bool has_cpu = false;
        for (std::string name; std::getline(names, name, ',');) {
            has_cpu = has_cpu || name == "cpu";
        }
        if (!has_cpu) {
            continue;
        }
        for (const CgroupMount &m : mounts) {
            if (m.v2 || !m.has_cpu) {
                continue;
            }
            for (const std::string &dir : visible_dirs(m, path)) {
                long long quota = 0;
                long long period = 0;
                if (read_number(dir + "/cpu.cfs_quota_us", quota) &&
                    read_number(dir + "/cpu.cfs_period_us", period)) {
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
    std::vector<CgroupMount> mounts = cgroup_mounts(read_file("/proc/self/mountinfo"));
    if (mounts.empty()) {
        // No mountinfo to go by: the usual places, each showing its whole hierarchy.
        mounts = {{"/sys/fs/cgroup", "/", true, true},
                  {"/sys/fs/cgroup/cpu", "/", false, true},
                  {"/sys/fs/cgroup/cpu,cpuacct", "/", false, true}};
    }
    const double quota = cgroup_quota_cpus(mounts, read_file("/proc/self/cgroup"));
    if (quota > 0.0) {
        // Rounded down, with a little slack so a quota of exactly 2.0 read as 1.9999999 is 2.
        cpus = std::min(cpus, std::max(1, static_cast<int>(std::floor(quota + 1e-6))));
    }
#endif
    return std::max(1, cpus);
}

int default_render_threads(int cpus) { return std::clamp(cpus, 1, 4); }

} // namespace skysim::core
