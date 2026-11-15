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

void test_cgroup_tree() {
    const fs::path root = fs::temp_directory_path() / "skysim_test_cpus";
    fs::remove_all(root);

    // cgroup v2 inside a container (cgroup namespace): the process is at "/", and the
    // container's limit is the root's cpu.max.
    write(root / "v2" / "cpu.max", "200000 100000\n");
    CHECK(near(cgroup_quota_cpus((root / "v2").string(), "0::/\n"), 2.0));

    // cgroup v2 on a host: the tightest quota anywhere on the path wins, wherever it is set.
    write(root / "v2host" / "ecs" / "cpu.max", "400000 100000\n");
    write(root / "v2host" / "ecs" / "task" / "cpu.max", "max 100000\n");
    write(root / "v2host" / "ecs" / "task" / "app" / "cpu.max", "max 100000\n");
    CHECK(near(cgroup_quota_cpus((root / "v2host").string(), "0::/ecs/task/app\n"), 4.0));
    write(root / "v2host" / "ecs" / "task" / "app" / "cpu.max", "100000 100000\n");
    CHECK(near(cgroup_quota_cpus((root / "v2host").string(), "0::/ecs/task/app\n"), 1.0));

    // No limit anywhere, and no cgroup filesystem at all.
    write(root / "v2free" / "cpu.max", "max 100000\n");
    CHECK(near(cgroup_quota_cpus((root / "v2free").string(), "0::/\n"), 0.0));
    CHECK(near(cgroup_quota_cpus((root / "nowhere").string(), "0::/x\n"), 0.0));
    CHECK(near(cgroup_quota_cpus(root.string(), ""), 0.0));

    // cgroup v1 in a container: /proc/self/cgroup names the host path, which does not exist
    // under the container's mount; the mount root holds the container's quota.
    write(root / "v1" / "cpu" / "cpu.cfs_quota_us", "150000\n");
    write(root / "v1" / "cpu" / "cpu.cfs_period_us", "100000\n");
    const std::string v1_self =
        "12:memory:/docker/abc\n4:cpu,cpuacct:/docker/abc\n1:name=systemd:/docker/abc\n";
    CHECK(near(cgroup_quota_cpus((root / "v1").string(), v1_self), 1.5));

    // ... and mounted only under its controller list, with no quota (-1).
    write(root / "v1b" / "cpu,cpuacct" / "cpu.cfs_quota_us", "-1\n");
    write(root / "v1b" / "cpu,cpuacct" / "cpu.cfs_period_us", "100000\n");
    CHECK(near(cgroup_quota_cpus((root / "v1b").string(), v1_self), 0.0));
    write(root / "v1b" / "cpu,cpuacct" / "docker" / "abc" / "cpu.cfs_quota_us", "300000\n");
    write(root / "v1b" / "cpu,cpuacct" / "docker" / "abc" / "cpu.cfs_period_us", "100000\n");
    CHECK(near(cgroup_quota_cpus((root / "v1b").string(), v1_self), 3.0));

    // A v1 line for another controller is not a CPU limit.
    CHECK(near(cgroup_quota_cpus((root / "v1").string(), "12:memory:/docker/abc\n"), 0.0));

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
    test_cgroup_tree();
    test_available_and_default();
    if (g_failures == 0) {
        std::printf("test_cpus: OK\n");
    }
    return g_failures == 0 ? 0 : 1;
}
