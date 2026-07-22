// skysim entry point. M1 wires: config -> UdpEndpoints -> strict lockstep tick loop with a
// canned "sits on ground" state per vehicle. M2 replaces the canned update with core::World.
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "core/clock.h"
#include "protocol/packets.h"
#include "protocol/udp_endpoint.h"

namespace {

struct Options {
    int vehicles = 1;
    int base_instance = 0;
    double dt_s = 1.0 / 400.0;
    std::string time_mode = "strict";
};

Options parse_args(int argc, char **argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        auto need_value = [&](const char *flag) -> const char * {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "skysim: %s requires a value\n", flag);
                std::exit(2);
            }
            return argv[++i];
        };
        if (std::strcmp(argv[i], "--vehicles") == 0) {
            o.vehicles = std::atoi(need_value("--vehicles"));
        } else if (std::strcmp(argv[i], "--base-instance") == 0) {
            o.base_instance = std::atoi(need_value("--base-instance"));
        } else if (std::strcmp(argv[i], "--dt") == 0) {
            o.dt_s = std::atof(need_value("--dt"));
        } else if (std::strcmp(argv[i], "--time-mode") == 0) {
            o.time_mode = need_value("--time-mode");
        } else {
            std::fprintf(stderr, "skysim: unknown arg %s\n", argv[i]);
            std::exit(2);
        }
    }
    if (o.vehicles < 1 || o.dt_s <= 0.0 || o.time_mode != "strict") {
        std::fprintf(stderr, "skysim: invalid options (M1 supports --time-mode strict only)\n");
        std::exit(2);
    }
    return o;
}

std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }

// One connected SITL vehicle: endpoint + per-vehicle clock + canned truth.
struct VehicleSlot {
    std::unique_ptr<skysim::protocol::UdpEndpoint> endpoint;
    skysim::core::PhysicsClock clock;
    skysim::protocol::VehicleTruth truth{};
    char last_json[1024];
    size_t last_json_len = 0;
    bool has_pending_input = false;
    skysim::protocol::ParsedServos pending{};
    uint64_t gaps = 0, reboots = 0, bad = 0;

    explicit VehicleSlot(int instance, double dt)
        : endpoint(std::make_unique<skysim::protocol::UdpEndpoint>(instance)), clock(dt) {
        truth.accel_body[2] = -9.81; // at rest: specific force = -g, FRD z down
        truth.quat_wxyz[0] = 1.0;
    }
};

} // namespace

int main(int argc, char **argv) {
    const Options opt = parse_args(argc, argv);
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    std::vector<std::unique_ptr<VehicleSlot>> fleet;
    for (int i = 0; i < opt.vehicles; ++i) {
        fleet.push_back(std::make_unique<VehicleSlot>(opt.base_instance + i, opt.dt_s));
        std::printf("skysim: vehicle %d listening on udp %d\n", i, fleet.back()->endpoint->port());
    }
    std::printf("skysim: %d vehicle(s), dt=%.6f s, %s mode, canned physics (M1)\n", opt.vehicles, opt.dt_s,
                opt.time_mode.c_str());
    std::fflush(stdout);

    using skysim::protocol::FrameStatus;
    while (!g_stop.load(std::memory_order_relaxed)) {
        // Strict mode: barrier — a vehicle ticks only when its next input frame has arrived.
        // Per-vehicle clocks (not one world clock) until M2 introduces the shared world.
        bool any_progress = false;
        for (auto &v : fleet) {
            if (!v->has_pending_input) {
                if (auto p = v->endpoint->take_latest()) {
                    switch (p->status) {
                    case FrameStatus::kOk:
                    case FrameStatus::kGap:
                    case FrameStatus::kReboot:
                        if (p->status == FrameStatus::kGap) {
                            ++v->gaps;
                        }
                        if (p->status == FrameStatus::kReboot) {
                            ++v->reboots;
                            std::printf("skysim: vehicle reboot detected (frame_count %u)\n", p->frame_count);
                        }
                        v->pending = *p;
                        v->has_pending_input = true;
                        break;
                    case FrameStatus::kDuplicate:
                        // Normal during handshake (docs/PROTOCOL.md): don't re-step, do re-reply.
                        if (v->last_json_len > 0) {
                            v->endpoint->send_reply(v->last_json, v->last_json_len);
                        }
                        break;
                    case FrameStatus::kBadMagic:
                    case FrameStatus::kBadSize:
                        ++v->bad;
                        break;
                    }
                }
            }
            if (v->has_pending_input) {
                // M1 canned physics: consume the input, advance this vehicle's clock, stay put.
                v->clock.advance();
                v->truth.timestamp_s = v->clock.now();
                v->last_json_len =
                    skysim::protocol::build_state_json(v->truth, v->last_json, sizeof(v->last_json));
                if (v->last_json_len == 0) {
                    std::fprintf(stderr, "skysim: build_state_json overflow\n");
                    return 1;
                }
                v->endpoint->send_reply(v->last_json, v->last_json_len);
                v->has_pending_input = false;
                any_progress = true;
            }
        }
        if (!any_progress) {
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    }

    for (size_t i = 0; i < fleet.size(); ++i) {
        std::printf("skysim: vehicle %zu ticks=%llu gaps=%llu reboots=%llu bad=%llu\n", i,
                    static_cast<unsigned long long>(fleet[i]->clock.tick_index()),
                    static_cast<unsigned long long>(fleet[i]->gaps),
                    static_cast<unsigned long long>(fleet[i]->reboots),
                    static_cast<unsigned long long>(fleet[i]->bad));
    }
    return 0;
}
