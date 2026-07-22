// VehicleManager: leak-free instance allocator + managed-process lifecycle, and TickMetrics.
#include <chrono>
#include <cstdio>
#include <thread>

#include "core/metrics.h"
#include "vehicle/manager.h"

namespace {

int g_failures = 0;

#define CHECK(cond)                                                                                          \
    do {                                                                                                     \
        if (!(cond)) {                                                                                       \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                      \
            ++g_failures;                                                                                    \
        }                                                                                                    \
    } while (0)

} // namespace

int main() {
    using skysim::vehicle::ProcessConfig;
    using skysim::vehicle::VehicleManager;

    // --- Allocator: lowest-free reuse, no leaks over churn. ---
    {
        ProcessConfig cfg;
        VehicleManager m(30, cfg);
        CHECK(m.allocate_instance() == 30);
        CHECK(m.allocate_instance() == 31);
        CHECK(m.allocate_instance() == 32);
        CHECK(m.instances_in_use() == 3);
        m.release_instance(31);
        CHECK(m.instances_in_use() == 2);
        CHECK(m.allocate_instance() == 31); // lowest free slot reused
        for (int cycle = 0; cycle < 100; ++cycle) {
            const int inst = m.allocate_instance();
            CHECK(inst == 33); // never grows past the working set
            m.release_instance(inst);
        }
        CHECK(m.instances_in_use() == 3);
        m.release_instance(999); // out of range: ignored, no crash
    }

    // --- Managed processes: fork/exec, reap, stop; exec failure still yields a corpse. ---
    {
        ProcessConfig cfg;
        cfg.binary = "/bin/true"; // exits immediately regardless of the arducopter arg shape
        cfg.home = "0,0,0,0";
        cfg.defaults = "/dev/null";
        cfg.run_dir = "/tmp/skysim_test_mgr";
        VehicleManager m(50, cfg);
        auto pid = m.launch_sitl(50);
        CHECK(pid.has_value());
        CHECK(m.children() == 1);
        m.stop_sitl(*pid); // may race with natural exit — must be harmless either way
        for (int i = 0; i < 200 && m.children() > 0; ++i) {
            m.reap();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        CHECK(m.children() == 0);

        cfg.binary = "/nonexistent/arducopter";
        VehicleManager bad(51, cfg);
        auto pid2 = bad.launch_sitl(51); // fork succeeds, exec fails, child _Exit(127)
        CHECK(pid2.has_value());
        for (int i = 0; i < 200 && bad.children() > 0; ++i) {
            bad.reap();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        CHECK(bad.children() == 0);
    }

    // --- TickMetrics: percentiles over the ring buffer + counters. ---
    {
        skysim::core::TickMetrics tm;
        auto empty = tm.snapshot();
        CHECK(empty.p50_us == 0.0 && empty.p99_us == 0.0);
        for (int i = 1; i <= 100; ++i) {
            tm.record_us(static_cast<double>(i));
        }
        tm.straggler_events = 7;
        tm.freezes = 2;
        const auto s = tm.snapshot();
        CHECK(s.p50_us >= 50.0 && s.p50_us <= 52.0);
        CHECK(s.p99_us >= 99.0 && s.p99_us <= 100.0);
        CHECK(s.straggler_events == 7 && s.freezes == 2);
    }

    if (g_failures == 0) {
        std::printf("test_manager: all checks OK\n");
        return 0;
    }
    std::printf("test_manager: %d failure(s)\n", g_failures);
    return 1;
}
