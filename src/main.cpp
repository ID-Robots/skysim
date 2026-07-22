// skysim entry point. Strict lockstep: config -> World -> UdpEndpoints -> tick loop.
// M2: Jolt world + X-quad per vehicle (flat ground; --tiles adds cooked city tiles).
// --canned keeps the M1 kinematic on-ground reply path (no physics) for protocol debugging.
#include <algorithm>
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

#include "core/world.h"
#include "protocol/packets.h"
#include "protocol/udp_endpoint.h"
#include "vehicle/quad.h"

namespace {

struct Options {
    int vehicles = 1;
    int base_instance = 0;
    double dt_s = 1.0 / 400.0;
    std::string time_mode = "strict";
    std::string tiles;
    bool canned = false;
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
        } else if (std::strcmp(argv[i], "--tiles") == 0) {
            o.tiles = need_value("--tiles");
        } else if (std::strcmp(argv[i], "--canned") == 0) {
            o.canned = true;
        } else {
            std::fprintf(stderr, "skysim: unknown arg %s\n", argv[i]);
            std::exit(2);
        }
    }
    if (o.vehicles < 1 || o.dt_s <= 0.0 || o.time_mode != "strict") {
        std::fprintf(stderr, "skysim: invalid options (strict time mode only until M4)\n");
        std::exit(2);
    }
    return o;
}

std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }

// One connected SITL vehicle: endpoint + body + motor states + last reply.
struct VehicleSlot {
    std::unique_ptr<skysim::protocol::UdpEndpoint> endpoint;
    uint32_t body_id = 0;
    skysim::vehicle::QuadParams params;
    std::array<skysim::vehicle::MotorState, 4> motors{};
    skysim::core::BodyState state;
    char last_json[1024];
    size_t last_json_len = 0;
    bool has_pending_input = false;
    skysim::protocol::ParsedServos pending{};
    uint64_t gaps = 0, reboots = 0, bad = 0, ticks = 0;

    explicit VehicleSlot(int instance)
        : endpoint(std::make_unique<skysim::protocol::UdpEndpoint>(instance)) {}
};

skysim::protocol::VehicleTruth truth_from_state(const skysim::core::BodyState &s, double t) {
    skysim::protocol::VehicleTruth out{};
    out.timestamp_s = t;
    for (int k = 0; k < 3; ++k) {
        out.gyro_rps[k] = s.gyro_frd[k];
        out.accel_body[k] = s.accel_body_frd[k];
        out.pos_ned_m[k] = s.pos_ned[k];
        out.vel_ned_mps[k] = s.vel_ned[k];
    }
    for (int k = 0; k < 4; ++k) {
        out.quat_wxyz[k] = s.quat_ned_frd[k];
    }
    return out;
}

} // namespace

int main(int argc, char **argv) {
    const Options opt = parse_args(argc, argv);
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    skysim::core::WorldConfig wcfg;
    wcfg.dt_s = opt.dt_s;
    std::unique_ptr<skysim::core::World> world;
    if (!opt.canned) {
        world = std::make_unique<skysim::core::World>(wcfg);
        world->add_ground_plane();
        if (!opt.tiles.empty()) {
            const size_t n = world->load_tiles(opt.tiles);
            std::printf("skysim: loaded %zu tile(s) from %s\n", n, opt.tiles.c_str());
        }
    }

    std::vector<std::unique_ptr<VehicleSlot>> fleet;
    for (int i = 0; i < opt.vehicles; ++i) {
        auto slot = std::make_unique<VehicleSlot>(opt.base_instance + i);
        if (world) {
            skysim::core::VehicleBodyParams bp;
            bp.mass_kg = slot->params.mass_kg;
            bp.inertia_diag_frd = {slot->params.inertia_diag[0], slot->params.inertia_diag[1],
                                   slot->params.inertia_diag[2]};
            // Spawn side by side, 5 m east apart, resting just above the ground.
            bp.start_pos_ned = {0.0, 5.0 * i, -(bp.half_extents_frd[2] + 0.005)};
            slot->body_id = world->add_vehicle(bp);
            slot->state = world->get_state(slot->body_id);
        }
        std::printf("skysim: vehicle %d listening on udp %d\n", i, slot->endpoint->port());
        fleet.push_back(std::move(slot));
    }
    std::printf("skysim: %d vehicle(s), dt=%.6f s, %s mode, %s\n", opt.vehicles, opt.dt_s,
                opt.time_mode.c_str(), opt.canned ? "canned physics (M1)" : "jolt physics");
    std::fflush(stdout);

    double canned_time = 0.0;
    using skysim::protocol::FrameStatus;
    while (!g_stop.load(std::memory_order_relaxed)) {
        // Strict mode barrier: consume input from every vehicle, then step the world once.
        bool all_ready = true;
        for (auto &v : fleet) {
            if (v->has_pending_input) {
                continue;
            }
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
                        std::printf("skysim: reboot detected (frame_count %u); pose kept\n", p->frame_count);
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
            if (!v->has_pending_input) {
                all_ready = false;
            }
        }
        if (!all_ready) {
            std::this_thread::sleep_for(std::chrono::microseconds(50));
            continue;
        }

        double now_s;
        if (world) {
            for (auto &v : fleet) {
                // Airspeed = body velocity (no wind until M3), rotated into FRD.
                const auto vel_frd =
                    skysim::core::quat_rotate_inverse(v->state.quat_ned_frd, v->state.vel_ned);
                std::array<uint16_t, 16> pwm{};
                std::copy_n(v->pending.pwm.begin(), 16, pwm.begin());
                const double vel_air_frd[3] = {vel_frd[0], vel_frd[1], vel_frd[2]};
                const auto wrench =
                    skysim::vehicle::compute_wrench(v->params, v->motors, pwm, vel_air_frd, opt.dt_s);
                world->apply_body_wrench(v->body_id,
                                         {wrench.force_frd[0], wrench.force_frd[1], wrench.force_frd[2]},
                                         {wrench.torque_frd[0], wrench.torque_frd[1], wrench.torque_frd[2]});
            }
            world->step();
            now_s = world->now();
            for (auto &v : fleet) {
                v->state = world->get_state(v->body_id);
            }
        } else {
            canned_time += opt.dt_s;
            now_s = canned_time;
        }

        for (auto &v : fleet) {
            skysim::protocol::VehicleTruth truth;
            if (world) {
                truth = truth_from_state(v->state, now_s);
            } else {
                truth = {};
                truth.timestamp_s = now_s;
                truth.accel_body[2] = -9.81;
                truth.quat_wxyz[0] = 1.0;
            }
            v->last_json_len = skysim::protocol::build_state_json(truth, v->last_json, sizeof(v->last_json));
            if (v->last_json_len == 0) {
                std::fprintf(stderr, "skysim: build_state_json overflow\n");
                return 1;
            }
            v->endpoint->send_reply(v->last_json, v->last_json_len);
            v->has_pending_input = false;
            ++v->ticks;
        }
    }

    for (size_t i = 0; i < fleet.size(); ++i) {
        std::printf("skysim: vehicle %zu ticks=%llu gaps=%llu reboots=%llu bad=%llu\n", i,
                    static_cast<unsigned long long>(fleet[i]->ticks),
                    static_cast<unsigned long long>(fleet[i]->gaps),
                    static_cast<unsigned long long>(fleet[i]->reboots),
                    static_cast<unsigned long long>(fleet[i]->bad));
    }
    return 0;
}
