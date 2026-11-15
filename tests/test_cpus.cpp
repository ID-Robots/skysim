// How many CPUs skysim believes it has, and what the camera does with that.
//
// The default render pool used to be a fixed four workers plus the render thread: five threads
// on a 2-vCPU ECS task whose quota the kernel enforces by throttling the whole cgroup, the
// physics tick included. These hold the CPU count to the quota and the default to the count.
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

#include <vector>

#include <unistd.h>
#if defined(__linux__)
#include <sched.h>
#endif

#include "core/cpus.h"

namespace {

int g_failures = 0;

#define CHECK(cond)                                                                                          \
    do {                                                                                                     \
        if (!(cond)) {                                                                                       \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                      \
            ++g_failures;                                                                                    \
        }                                                                                                    \
    } while (0)

namespace fs = std::filesystem;
using skysim::core::cgroup_quota_cpus;
using skysim::core::cgroup_v1_quota_cpus;
using skysim::core::cgroup_v2_quota_cpus;

void write(const fs::path &path, const std::string &text) {
    fs::create_directories(path.parent_path());
    std::ofstream(path) << text;
}

bool near(double a, double b) { return a > b - 1e-9 && a < b + 1e-9; }

void test_parsers() {
    CHECK(near(cgroup_v2_quota_cpus("200000 100000\n"), 2.0)); // what docker --cpus 2 writes
    CHECK(near(cgroup_v2_quota_cpus("150000 100000"), 1.5));
    CHECK(near(cgroup_v2_quota_cpus("max 100000\n"), 0.0)); // no limit
    CHECK(near(cgroup_v2_quota_cpus(""), 0.0));
    CHECK(near(cgroup_v2_quota_cpus("garbage"), 0.0));
    CHECK(near(cgroup_v2_quota_cpus("12x 100000"), 0.0));
    CHECK(near(cgroup_v2_quota_cpus("100000 0"), 0.0));
    CHECK(near(cgroup_v1_quota_cpus(200000, 100000), 2.0));
    CHECK(near(cgroup_v1_quota_cpus(-1, 100000), 0.0)); // v1's "no limit"
    CHECK(near(cgroup_v1_quota_cpus(50000, 0), 0.0));
}

// A path as mountinfo writes it: space, tab, newline and backslash as octal escapes, so a
// temporary directory with a space in it still makes a well-formed record.
std::string mountinfo_escape(const std::string &path) {
    std::string out;
    for (const char c : path) {
        switch (c) {
        case ' ':
            out += "\\040";
            break;
        case '\t':
            out += "\\011";
            break;
        case '\n':
            out += "\\012";
            break;
        case '\\':
            out += "\\134";
            break;
        default:
            out.push_back(c);
        }
    }
    return out;
}

// A mountinfo line for a cgroup mount, the way the kernel writes one.
std::string mountinfo_line(const fs::path &mount_point, const std::string &root, bool v2) {
    return "30 25 0:26 " + mountinfo_escape(root) + " " + mountinfo_escape(mount_point.string()) +
           " rw,nosuid,nodev,noexec,relatime shared:9 - " +
           (v2 ? "cgroup2 cgroup2 rw,nsdelegate" : "cgroup cgroup rw,cpu,cpuacct") + "\n";
}

std::vector<skysim::core::CgroupMount> mounts(const std::string &mountinfo) {
    return skysim::core::cgroup_mounts(mountinfo);
}

void test_mountinfo() {
    const std::string text =
        "22 1 8:2 / / rw,relatime shared:1 - ext4 /dev/sda2 rw\n"
        "35 24 0:30 / /sys/fs/cgroup rw,nosuid,nodev,noexec,relatime shared:9 - cgroup2 cgroup2 "
        "rw,nsdelegate\n"
        "40 35 0:31 /docker/abc /sys/fs/cgroup/cpu,cpuacct rw,nosuid - cgroup cgroup rw,cpu,cpuacct\n"
        "41 35 0:32 / /sys/fs/cgroup/memory rw,nosuid - cgroup cgroup rw,memory\n"
        "42 35 0:33 / /mnt/with\\040space rw master:3 shared:7 - cgroup2 cgroup2 rw\n"
        "garbage line\n";
    const auto m = mounts(text);
    CHECK(m.size() == 4);
    if (m.size() == 4) {
        CHECK(m[0].mount_point == "/sys/fs/cgroup" && m[0].root == "/" && m[0].v2);
        CHECK(m[1].mount_point == "/sys/fs/cgroup/cpu,cpuacct" && m[1].root == "/docker/abc" && !m[1].v2 &&
              m[1].has_cpu);
        CHECK(!m[2].v2 && !m[2].has_cpu);                        // memory controller only
        CHECK(m[3].mount_point == "/mnt/with space" && m[3].v2); // optional fields skipped, \040 decoded
    }
    CHECK(mounts("").empty());
    // Every escape mountinfo uses comes back as the character it stands for.
    const auto odd = mounts(mountinfo_line("/tmp/a b\tc\\d\ne", "/x y", true));
    CHECK(odd.size() == 1 && odd[0].mount_point == "/tmp/a b\tc\\d\ne" && odd[0].root == "/x y");
}

void test_cgroup_tree() {
    // One directory per process: two test runs at once must not delete each other's files.
    const fs::path root = fs::temp_directory_path() / ("skysim_test_cpus_" + std::to_string(getpid()));
    fs::remove_all(root);

    // cgroup v2 inside a container (cgroup namespace): the process is at "/", and the
    // container's limit is the mount root's cpu.max.
    write(root / "v2" / "cpu.max", "200000 100000\n");
    CHECK(near(cgroup_quota_cpus(mounts(mountinfo_line(root / "v2", "/", true)), "0::/\n"), 2.0));

    // cgroup v2 on a host: the tightest quota anywhere on the path wins, wherever it is set.
    const auto host = mounts(mountinfo_line(root / "v2host", "/", true));
    write(root / "v2host" / "ecs" / "cpu.max", "400000 100000\n");
    write(root / "v2host" / "ecs" / "task" / "cpu.max", "max 100000\n");
    write(root / "v2host" / "ecs" / "task" / "app" / "cpu.max", "max 100000\n");
    CHECK(near(cgroup_quota_cpus(host, "0::/ecs/task/app\n"), 4.0));
    write(root / "v2host" / "ecs" / "task" / "app" / "cpu.max", "100000 100000\n");
    CHECK(near(cgroup_quota_cpus(host, "0::/ecs/task/app\n"), 1.0));

    // A mount that shows a subtree: /tenant mounted at v2sub, the process in /tenant/app. Its
    // quota is at v2sub/app, not v2sub/tenant/app, and the unlimited mount root must not hide it.
    const auto sub = mounts(mountinfo_line(root / "v2sub", "/tenant", true));
    write(root / "v2sub" / "cpu.max", "max 100000\n");
    write(root / "v2sub" / "app" / "cpu.max", "100000 100000\n");
    CHECK(near(cgroup_quota_cpus(sub, "0::/tenant/app\n"), 1.0));
    // A cgroup outside what the mount shows: only the mount's own root is visible.
    write(root / "v2sub2" / "cpu.max", "300000 100000\n");
    CHECK(near(cgroup_quota_cpus(mounts(mountinfo_line(root / "v2sub2", "/tenant", true)), "0::/../other\n"),
               3.0));

    // No limit anywhere, and no cgroup filesystem at all.
    write(root / "v2free" / "cpu.max", "max 100000\n");
    CHECK(near(cgroup_quota_cpus(mounts(mountinfo_line(root / "v2free", "/", true)), "0::/\n"), 0.0));
    CHECK(near(cgroup_quota_cpus(mounts(mountinfo_line(root / "nowhere", "/", true)), "0::/x\n"), 0.0));
    CHECK(near(cgroup_quota_cpus({}, "0::/x\n"), 0.0));
    CHECK(near(cgroup_quota_cpus(host, ""), 0.0));

    // cgroup v1 in a container: the cpu mount's root is the container's own cgroup, which
    // /proc/self/cgroup names by its host path; its quota is at the mount point.
    const auto v1 = mounts(mountinfo_line(root / "v1" / "cpu,cpuacct", "/docker/abc", false));
    write(root / "v1" / "cpu,cpuacct" / "cpu.cfs_quota_us", "150000\n");
    write(root / "v1" / "cpu,cpuacct" / "cpu.cfs_period_us", "100000\n");
    const std::string v1_self =
        "12:memory:/docker/abc\n4:cpu,cpuacct:/docker/abc\n1:name=systemd:/docker/abc\n";
    CHECK(near(cgroup_quota_cpus(v1, v1_self), 1.5));

    // ... and on a host, the whole hierarchy mounted: no quota at the root (-1), one below.
    const auto v1host = mounts(mountinfo_line(root / "v1b" / "cpu,cpuacct", "/", false));
    write(root / "v1b" / "cpu,cpuacct" / "cpu.cfs_quota_us", "-1\n");
    write(root / "v1b" / "cpu,cpuacct" / "cpu.cfs_period_us", "100000\n");
    CHECK(near(cgroup_quota_cpus(v1host, v1_self), 0.0));
    write(root / "v1b" / "cpu,cpuacct" / "docker" / "abc" / "cpu.cfs_quota_us", "300000\n");
    write(root / "v1b" / "cpu,cpuacct" / "docker" / "abc" / "cpu.cfs_period_us", "100000\n");
    CHECK(near(cgroup_quota_cpus(v1host, v1_self), 3.0));

    // A v1 line for another controller is not a CPU limit.
    CHECK(near(cgroup_quota_cpus(v1, "12:memory:/docker/abc\n"), 0.0));

    fs::remove_all(root);
}

void test_available_and_default() {
    const int cpus = skysim::core::available_cpus();
    const auto hw = static_cast<int>(std::thread::hardware_concurrency());
    CHECK(cpus >= 1);
    CHECK(hw <= 0 || cpus <= hw);
#if defined(__linux__)
    cpu_set_t set;
    CPU_ZERO(&set);
    if (sched_getaffinity(0, sizeof(set), &set) == 0) {
        CHECK(cpus <= CPU_COUNT(&set)); // taskset -c 0 ctest ... must see one CPU
    }
#endif
    std::printf("test_cpus: %d CPU(s) available here (hardware_concurrency %d), default render threads %d\n",
                cpus, hw, skysim::core::default_render_threads(cpus));

    using skysim::core::default_render_threads;
    // Never more render threads than CPUs, never more than four, never fewer than one.
    CHECK(default_render_threads(0) == 1);
    CHECK(default_render_threads(1) == 1);
    CHECK(default_render_threads(2) == 2); // the 2-vCPU dev task: two, not the old five
    CHECK(default_render_threads(3) == 3);
    CHECK(default_render_threads(4) == 4);
    CHECK(default_render_threads(5) == 4);
    CHECK(default_render_threads(64) == 4);
    for (int n = 1; n <= 64; ++n) {
        CHECK(default_render_threads(n) <= n);
    }
    CHECK(default_render_threads(cpus) <= cpus);
}

} // namespace

int main() {
    test_parsers();
    test_mountinfo();
    test_cgroup_tree();
    test_available_and_default();
    if (g_failures == 0) {
        std::printf("test_cpus: OK\n");
    }
    return g_failures == 0 ? 0 : 1;
}
