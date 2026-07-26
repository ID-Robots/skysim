// ControlServer: REST surface against a fake tick thread (queue consumer fulfilling
// promises) and canned snapshots. Uses the real HTTP stack via httplib::Client.
#include <httplib.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <thread>

#include "api/control_server.h"

namespace {

int g_failures = 0;

#define CHECK(cond)                                                                                          \
    do {                                                                                                     \
        if (!(cond)) {                                                                                       \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                      \
            ++g_failures;                                                                                    \
        }                                                                                                    \
    } while (0)

constexpr int kPort = 18642;

} // namespace

int main() {
    using namespace skysim::api;

    // --- CommandQueue basics. ---
    {
        CommandQueue q;
        DespawnCommand d;
        d.id = 7;
        q.push(Command{std::move(d)});
        auto drained = q.drain();
        CHECK(drained.size() == 1);
        CHECK(q.drain().empty());
        std::get<DespawnCommand>(drained[0]).done.set_value(false); // don't leak the promise
    }

    CommandQueue queue;
    std::atomic<bool> stop{false};
    std::atomic<bool> saw_launch_process{false};
    std::atomic<int> last_instance{-2};
    std::atomic<size_t> path_checks{0};
    std::atomic<size_t> last_waypoint_count{0};
    std::atomic<double> last_clearance{-1.0};

    // Fake tick thread: drain + answer. Spawn -> fixed result; despawn -> ok iff id == 1.
    // Path checks return a hit, a clear result, or unavailable geometry based on the first
    // waypoint's north coordinate so the HTTP response cases stay deterministic.
    std::thread consumer([&] {
        while (!stop.load()) {
            for (auto &cmd : queue.drain()) {
                if (auto *s = std::get_if<SpawnCommand>(&cmd)) {
                    saw_launch_process = s->request.launch_process;
                    last_instance = s->request.instance;
                    s->done.set_value(skysim::vehicle::SpawnResult{1, 30, 9302, 6060});
                } else if (auto *d = std::get_if<DespawnCommand>(&cmd)) {
                    d->done.set_value(d->id == 1);
                } else if (auto *p = std::get_if<PathCheckCommand>(&cmd)) {
                    ++path_checks;
                    last_waypoint_count = p->waypoints_ned.size();
                    last_clearance = p->clearance_m;

                    PathCheckResult result;
                    if (!p->waypoints_ned.empty() && p->waypoints_ned.front()[0] == 1.0) {
                        result.geometry_available = true;
                        result.hits.push_back({1, {12.0, -3.5, -20.0}, 7.25});
                    } else if (!p->waypoints_ned.empty() && p->waypoints_ned.front()[0] == 9.0) {
                        result.geometry_available = true;
                    }
                    p->done.set_value(std::move(result));
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    });

    ControlServer::Snapshots snaps;
    snaps.vehicles = [] {
        VehicleInfo v;
        v.id = 1;
        v.instance = 30;
        v.json_port = 9302;
        v.mavlink_tcp_port = 6060;
        v.connected = true;
        v.frozen = true;
        v.held_ticks = 4;
        v.pos_ned[0] = 1.5;
        return std::vector<VehicleInfo>{v};
    };
    snaps.metrics = [] {
        MetricsInfo m;
        m.tick = 1234;
        m.sim_time_s = 1.5425;
        m.tick_p99_us = 321.5;
        m.vehicles = 1;
        m.freezes = 1;
        return m;
    };

    {
        ControlServer server("127.0.0.1", kPort, queue, snaps);
        httplib::Client client("127.0.0.1", kPort);
        client.set_read_timeout(15, 0);

        auto spawn = client.Post("/vehicles", "{\"launch_process\":true}", "application/json");
        CHECK(spawn && spawn->status == 200);
        CHECK(spawn && spawn->body.find("\"id\":1") != std::string::npos);
        CHECK(spawn && spawn->body.find("\"json_port\":9302") != std::string::npos);
        CHECK(saw_launch_process.load());

        auto spawn2 = client.Post("/vehicles", "{}", "application/json");
        CHECK(spawn2 && spawn2->status == 200);
        CHECK(!saw_launch_process.load()); // absent key parses as false
        CHECK(last_instance.load() == -1); // absent instance -> skysim allocates

        auto spawn3 = client.Post("/vehicles", "{\"instance\":7}", "application/json");
        CHECK(spawn3 && spawn3->status == 200);
        CHECK(last_instance.load() == 7); // gateway-chosen instance passes through

        // Mission checks validate input before reaching the tick thread.
        auto mission_bad =
            client.Post("/mission/check", R"({"waypoints_ned":[[0,0,-10]]})", "application/json");
        CHECK(mission_bad && mission_bad->status == 400);
        CHECK(mission_bad && mission_bad->body.find("at least two waypoints") != std::string::npos);
        CHECK(path_checks.load() == 0);

        // A valid blocked path is parsed, queued, and serialized with the hit details.
        auto mission_hit =
            client.Post("/mission/check", R"({"clearance_m":2.5,"waypoints_ned":[[1,2,-3],[4.5,-6,-7]]})",
                        "application/json");
        CHECK(mission_hit && mission_hit->status == 200);
        CHECK(mission_hit && mission_hit->body.find("\"checked\":true") != std::string::npos);
        CHECK(mission_hit && mission_hit->body.find("\"clear\":false") != std::string::npos);
        CHECK(mission_hit && mission_hit->body.find("\"leg\":1") != std::string::npos);
        CHECK(mission_hit && mission_hit->body.find("\"distance_m\":7.25") != std::string::npos);
        CHECK(mission_hit &&
              mission_hit->body.find("\"position_ned\":[12.00,-3.50,-20.00]") != std::string::npos);
        CHECK(path_checks.load() == 1);
        CHECK(last_waypoint_count.load() == 2);
        CHECK(std::abs(last_clearance.load() - 2.5) < 1e-12);

        // Missing clearance uses the conservative default; an empty hit list is clear only
        // when the tick thread confirms that geometry was available.
        auto mission_clear =
            client.Post("/mission/check", R"({"waypoints_ned":[[9,0,-10],[20,0,-10]]})", "application/json");
        CHECK(mission_clear && mission_clear->status == 200);
        CHECK(mission_clear && mission_clear->body.find("\"checked\":true") != std::string::npos);
        CHECK(mission_clear && mission_clear->body.find("\"clear\":true") != std::string::npos);
        CHECK(mission_clear && mission_clear->body.find("\"hits\":[]") != std::string::npos);
        CHECK(path_checks.load() == 2);
        CHECK(std::abs(last_clearance.load() - 1.0) < 1e-12);

        auto mission_unavailable =
            client.Post("/mission/check", R"({"waypoints_ned":[[8,0,-10],[20,0,-10]]})", "application/json");
        CHECK(mission_unavailable && mission_unavailable->status == 200);
        CHECK(mission_unavailable &&
              mission_unavailable->body.find("\"checked\":false") != std::string::npos);
        CHECK(mission_unavailable && mission_unavailable->body.find("\"clear\":false") != std::string::npos);
        CHECK(path_checks.load() == 3);

        auto del = client.Delete("/vehicles/1");
        CHECK(del && del->status == 200 && del->body.find("\"ok\":true") != std::string::npos);
        auto del404 = client.Delete("/vehicles/99");
        CHECK(del404 && del404->status == 404);

        auto list = client.Get("/vehicles");
        CHECK(list && list->status == 200);
        CHECK(list && list->body.find("\"frozen\":true") != std::string::npos);
        CHECK(list && list->body.find("\"held_ticks\":4") != std::string::npos);

        auto metrics = client.Get("/metrics");
        CHECK(metrics && metrics->status == 200);
        CHECK(metrics && metrics->body.find("\"tick\":1234") != std::string::npos);
        CHECK(metrics && metrics->body.find("\"freezes\":1") != std::string::npos);
    } // ~ControlServer stops + joins the HTTP thread

    // Rebind after teardown must work (the port is released, not leaked). Note a LIVE
    // conflict cannot be tested here: httplib sets SO_REUSEPORT, so two servers on one
    // port both bind and the kernel load-balances — don't share --api-port between sims.
    ControlServer server_again("127.0.0.1", kPort, queue, snaps);

    stop = true;
    consumer.join();

    if (g_failures == 0) {
        std::printf("test_api: all checks OK\n");
        return 0;
    }
    std::printf("test_api: %d failure(s)\n", g_failures);
    return 1;
}
