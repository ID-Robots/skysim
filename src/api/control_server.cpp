// See control_server.h. cpp-httplib blocking server on a dedicated thread; JSON is emitted
// with snprintf and parsed with strstr — the API surface is tiny and we keep zero JSON deps.
#include "api/control_server.h"

#include <httplib.h>

#include <chrono>
#include <cstdio>
#include <cctype>
#include <cstdlib>
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

// {"instance":N} — absent key means -1 (skysim picks the lowest free slot).
int parse_instance(const std::string &body) {
    const char *p = std::strstr(body.c_str(), "\"instance\"");
    if (p == nullptr) {
        return -1;
    }
    p += std::strlen("\"instance\"");
    while (*p == ':' || *p == ' ' || *p == '\t') {
        ++p;
    }
    return std::atoi(p);
}

// One numeric field out of a flat JSON body, or `fallback` when it is absent.
//
// Same hand-rolled shape as the parsers around it: the control plane takes small, known
// bodies from one caller, so a JSON dependency has never been worth it here. A missing
// field must fall back rather than read as zero — a pack of zero capacity is empty from
// the first tick, which would ground a fleet rather than misconfigure it.
double parse_double_field(const std::string &body, const char *field, double fallback) {
    const std::string key = std::string("\"") + field + "\"";
    const char *p = std::strstr(body.c_str(), key.c_str());
    if (p == nullptr) {
        return fallback;
    }
    p += key.size();
    while (*p == ':' || *p == ' ' || *p == '\t') {
        ++p;
    }
    const double value = std::atof(p);
    return value > 0.0 ? value : fallback;
}

// The pack block of a spawn request. Every field is independently optional, so a caller
// that only knows its capacity gets a sane voltage curve for free.
skysim::vehicle::BatteryPack parse_battery(const std::string &body) {
    skysim::vehicle::BatteryPack pack{};
    pack.capacity_mah = parse_double_field(body, "battery_capacity_mah", pack.capacity_mah);
    pack.full_v = parse_double_field(body, "battery_full_v", pack.full_v);
    pack.empty_v = parse_double_field(body, "battery_empty_v", pack.empty_v);
    pack.idle_a = parse_double_field(body, "battery_idle_a", pack.idle_a);
    pack.hover_a = parse_double_field(body, "battery_hover_a", pack.hover_a);
    // An empty voltage at or above full would invert the sag and make a draining pack
    // report a rising charge. Cheaper to reject here than to debug in flight.
    if (pack.empty_v >= pack.full_v) {
        pack.empty_v = pack.full_v * 0.8;
    }
    return pack;
}

// {"clearance_m":1.5} — absent means a conservative default airframe half-width.
double parse_clearance(const std::string &body) {
    const char *p = std::strstr(body.c_str(), "\"clearance_m\"");
    if (p == nullptr) {
        return 1.0;
    }
    p += std::strlen("\"clearance_m\"");
    while (*p == ':' || *p == ' ' || *p == '\t') {
        ++p;
    }
    const double value = std::atof(p);
    return value > 0.0 ? value : 1.0;
}

// {"waypoints_ned":[[n,e,d],[n,e,d],...]} — flat scan for numbers in triples, matching the
// deliberately dependency-free parsing style of this file.
std::vector<core::Vec3> parse_waypoints(const std::string &body) {
    std::vector<core::Vec3> out;
    const char *p = std::strstr(body.c_str(), "\"waypoints_ned\"");
    if (p == nullptr) {
        return out;
    }
    p = std::strchr(p, '[');
    if (p == nullptr) {
        return out;
    }
    const char *end = body.c_str() + body.size();

    while (p < end) {
        p = std::strchr(p + 1, '[');
        if (p == nullptr) {
            break;
        }
        core::Vec3 wp{};
        bool ok = true;
        const char *cursor = p + 1;
        for (int axis = 0; axis < 3; ++axis) {
            while (cursor < end && (*cursor == ' ' || *cursor == ',' || *cursor == '\t')) {
                ++cursor;
            }
            if (cursor >= end || (*cursor != '-' && *cursor != '+' && *cursor != '.' && std::isdigit(static_cast<unsigned char>(*cursor)) == 0)) {
                ok = false;
                break;
            }
            char *next = nullptr;
            wp[static_cast<size_t>(axis)] = std::strtod(cursor, &next);
            if (next == cursor) {
                ok = false;
                break;
            }
            cursor = next;
        }
        if (!ok) {
            break;
        }
        out.push_back(wp);
        p = cursor;
    }
    return out;
}

std::string vehicle_json(const VehicleInfo &v) {
    char buf[512];
    std::snprintf(buf, sizeof(buf),
                  "{\"id\":%u,\"instance\":%d,\"json_port\":%d,\"mavlink_tcp\":%d,"
                  "\"connected\":%s,\"frozen\":%s,\"held_ticks\":%llu,"
                  "\"midair_collisions\":%llu,\"static_contacts\":%llu,"
                  "\"building_contacts\":%llu,\"pos_ned\":[%.3f,%.3f,%.3f]}",
                  v.id, v.instance, v.json_port, v.mavlink_tcp_port, v.connected ? "true" : "false",
                  v.frozen ? "true" : "false", static_cast<unsigned long long>(v.held_ticks),
                  static_cast<unsigned long long>(v.midair_collisions),
                  static_cast<unsigned long long>(v.static_contacts),
                  static_cast<unsigned long long>(v.building_contacts), v.pos_ned[0], v.pos_ned[1],
                  v.pos_ned[2]);
    return buf;
}

} // namespace

struct ControlServer::Impl {
    httplib::Server server;
    std::thread thread;
};

ControlServer::ControlServer(const std::string &bind_addr, int port, CommandQueue &queue, Snapshots snapshots)
    : impl_(std::make_unique<Impl>()) {
    auto &s = impl_->server;

    s.Post("/vehicles", [&queue](const httplib::Request &req, httplib::Response &res) {
        SpawnCommand cmd;
        cmd.request.launch_process = parse_launch_process(req.body);
        cmd.request.instance = parse_instance(req.body);
        cmd.request.battery = parse_battery(req.body);
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

    // Sweep a planned mission path through the building geometry. Answers the pre-flight
    // question directly, without flying anything.
    s.Post("/mission/check", [&queue](const httplib::Request &req, httplib::Response &res) {
        PathCheckCommand cmd;
        cmd.waypoints_ned = parse_waypoints(req.body);
        cmd.clearance_m = parse_clearance(req.body);

        if (cmd.waypoints_ned.size() < 2) {
            res.status = 400;
            res.set_content(R"({"error":"at least two waypoints are required"})", "application/json");
            return;
        }

        auto future = cmd.done.get_future();
        queue.push(Command{std::move(cmd)});
        if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
            res.status = 504;
            res.set_content("{\"error\":\"tick thread did not answer\"}", "application/json");
            return;
        }

        const auto result = future.get();
        const auto &hits = result.hits;
        // 'checked' distinguishes "swept and found nothing" from "had nothing to sweep".
        std::string body = std::string("{\"checked\":") + (result.geometry_available ? "true" : "false") +
                           ",\"clear\":" + (result.geometry_available && hits.empty() ? "true" : "false") +
                           ",\"hits\":[";
        for (size_t i = 0; i < hits.size(); ++i) {
            char entry[256];
            std::snprintf(entry, sizeof(entry),
                          "%s{\"leg\":%zu,\"distance_m\":%.2f,\"position_ned\":[%.2f,%.2f,%.2f]}",
                          i ? "," : "", hits[i].leg, hits[i].distance_m, hits[i].position_ned[0],
                          hits[i].position_ned[1], hits[i].position_ned[2]);
            body += entry;
        }
        body += "]}";
        res.set_content(body, "application/json");
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

    s.Post(R"(/vehicles/(\d+)/battery/reset)", [&queue](const httplib::Request &req, httplib::Response &res) {
        BatteryResetCommand cmd;
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
                      "\"vehicles\":%zu,\"resident_tiles\":%zu}",
                      static_cast<unsigned long long>(m.tick), m.sim_time_s, m.tick_p50_us, m.tick_p99_us,
                      static_cast<unsigned long long>(m.straggler_events),
                      static_cast<unsigned long long>(m.freezes), m.vehicles, m.resident_tiles);
        res.set_content(buf, "application/json");
    });

    if (!s.bind_to_port(bind_addr, port)) {
        throw std::runtime_error("ControlServer: cannot bind " + bind_addr + ":" + std::to_string(port));
    }
    impl_->thread = std::thread([this] { impl_->server.listen_after_bind(); });
    // Wait for the listen loop to actually start: stop() delivered before is_running()
    // would be lost and ~ControlServer would join() forever (httplib stop/listen race).
    for (int i = 0; i < 1000 && !s.is_running(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::printf("skysim: control plane on http://%s:%d\n", bind_addr.c_str(), port);
}

ControlServer::~ControlServer() {
    impl_->server.stop();
    if (impl_->thread.joinable()) {
        impl_->thread.join();
    }
}

} // namespace skysim::api
