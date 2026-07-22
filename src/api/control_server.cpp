// See control_server.h. cpp-httplib blocking server on a dedicated thread; JSON is emitted
// with snprintf and parsed with strstr — the API surface is tiny and we keep zero JSON deps.
#include "api/control_server.h"

#include <httplib.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>

namespace skysim::api {

namespace {

// {"launch_process":true} — absent key means false.
bool parse_launch_process(const std::string &body) {
    const char *p = std::strstr(body.c_str(), "\"launch_process\"");
    if (p == nullptr) {
        return false;
    }
    p += std::strlen("\"launch_process\"");
    while (*p == ':' || *p == ' ' || *p == '\t') {
        ++p;
    }
    return std::strncmp(p, "true", 4) == 0 || *p == '1';
}

std::string vehicle_json(const VehicleInfo &v) {
    char buf[512];
    std::snprintf(buf, sizeof(buf),
                  "{\"id\":%u,\"instance\":%d,\"json_port\":%d,\"mavlink_tcp\":%d,"
                  "\"connected\":%s,\"frozen\":%s,\"held_ticks\":%llu,"
                  "\"pos_ned\":[%.3f,%.3f,%.3f]}",
                  v.id, v.instance, v.json_port, v.mavlink_tcp_port, v.connected ? "true" : "false",
                  v.frozen ? "true" : "false", static_cast<unsigned long long>(v.held_ticks), v.pos_ned[0],
                  v.pos_ned[1], v.pos_ned[2]);
    return buf;
}

} // namespace

struct ControlServer::Impl {
    httplib::Server server;
    std::thread thread;
};

ControlServer::ControlServer(int port, CommandQueue &queue, Snapshots snapshots)
    : impl_(std::make_unique<Impl>()) {
    auto &s = impl_->server;

    s.Post("/vehicles", [&queue](const httplib::Request &req, httplib::Response &res) {
        SpawnCommand cmd;
        cmd.request.launch_process = parse_launch_process(req.body);
        auto future = cmd.done.get_future();
        queue.push(Command{std::move(cmd)});
        if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
            res.status = 504;
            res.set_content("{\"error\":\"tick thread did not answer\"}", "application/json");
            return;
        }
        const auto result = future.get();
        if (!result.has_value()) {
            res.status = 503;
            res.set_content("{\"error\":\"spawn failed\"}", "application/json");
            return;
        }
        char buf[256];
        std::snprintf(buf, sizeof(buf), "{\"id\":%u,\"instance\":%d,\"json_port\":%d,\"mavlink_tcp\":%d}",
                      result->id, result->instance, result->json_port, result->mavlink_tcp_port);
        res.set_content(buf, "application/json");
    });

    s.Delete(R"(/vehicles/(\d+))", [&queue](const httplib::Request &req, httplib::Response &res) {
        DespawnCommand cmd;
        cmd.id = static_cast<uint32_t>(std::strtoul(req.matches[1].str().c_str(), nullptr, 10));
        auto future = cmd.done.get_future();
        queue.push(Command{std::move(cmd)});
        if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
            res.status = 504;
            return;
        }
        if (!future.get()) {
            res.status = 404;
            res.set_content("{\"error\":\"no such vehicle\"}", "application/json");
            return;
        }
        res.set_content("{\"ok\":true}", "application/json");
    });

    s.Get("/vehicles", [snapshots](const httplib::Request &, httplib::Response &res) {
        std::string out = "[";
        bool first = true;
        for (const auto &v : snapshots.vehicles()) {
            out += (first ? "" : ",") + vehicle_json(v);
            first = false;
        }
        out += "]";
        res.set_content(out, "application/json");
    });

    s.Get("/metrics", [snapshots](const httplib::Request &, httplib::Response &res) {
        const MetricsInfo m = snapshots.metrics();
        char buf[512];
        std::snprintf(buf, sizeof(buf),
                      "{\"tick\":%llu,\"sim_time_s\":%.3f,\"tick_p50_us\":%.1f,"
                      "\"tick_p99_us\":%.1f,\"straggler_events\":%llu,\"freezes\":%llu,"
                      "\"vehicles\":%zu}",
                      static_cast<unsigned long long>(m.tick), m.sim_time_s, m.tick_p50_us, m.tick_p99_us,
                      static_cast<unsigned long long>(m.straggler_events),
                      static_cast<unsigned long long>(m.freezes), m.vehicles);
        res.set_content(buf, "application/json");
    });

    if (!s.bind_to_port("127.0.0.1", port)) {
        throw std::runtime_error("ControlServer: cannot bind 127.0.0.1:" + std::to_string(port));
    }
    impl_->thread = std::thread([this] { impl_->server.listen_after_bind(); });
    std::printf("skysim: control plane on http://127.0.0.1:%d\n", port);
}

ControlServer::~ControlServer() {
    impl_->server.stop();
    if (impl_->thread.joinable()) {
        impl_->thread.join();
    }
}

} // namespace skysim::api
