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
    std::string truth_log;    // CSV: t,vehicle,north,east,down — ground-truth flight records
    std::string record_servo; // CSV of consumed servo frames (one line per vehicle per tick)
    std::string replay_servo; // drive the tick loop from a recorded CSV instead of UDP
    skysim::core::Vec3 wind_ned{0.0, 0.0, 0.0};
    double gust_sigma = 0.0;
    double gust_tau = 2.0;
    uint64_t seed = 1;
    bool rangefinder = false; // emit rng_1 (down-facing raycast, 40 m)
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
        } else if (std::strcmp(argv[i], "--truth-log") == 0) {
            o.truth_log = need_value("--truth-log");
        } else if (std::strcmp(argv[i], "--record-servo") == 0) {
            o.record_servo = need_value("--record-servo");
        } else if (std::strcmp(argv[i], "--replay-servo") == 0) {
            o.replay_servo = need_value("--replay-servo");
        } else if (std::strcmp(argv[i], "--wind") == 0) {
            if (std::sscanf(need_value("--wind"), "%lf,%lf,%lf", &o.wind_ned[0], &o.wind_ned[1],
                            &o.wind_ned[2]) != 3) {
                std::fprintf(stderr, "skysim: --wind expects n,e,d (m/s)\n");
                std::exit(2);
            }
        } else if (std::strcmp(argv[i], "--gust") == 0) {
            const char *val = need_value("--gust");
            if (std::sscanf(val, "%lf,%lf", &o.gust_sigma, &o.gust_tau) < 1) {
                std::fprintf(stderr, "skysim: --gust expects sigma[,tau]\n");
                std::exit(2);
            }
        } else if (std::strcmp(argv[i], "--seed") == 0) {
            o.seed = std::strtoull(need_value("--seed"), nullptr, 10);
        } else if (std::strcmp(argv[i], "--rangefinder") == 0) {
            o.rangefinder = true;
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

    VehicleSlot() = default; // replay mode: no endpoint, inputs staged from file
    explicit VehicleSlot(int instance)
        : endpoint(std::make_unique<skysim::protocol::UdpEndpoint>(instance)) {}
};

// Consume staged inputs (fleet[i]->pending), advance the world one tick, refresh states,
// append record/truth logs. Shared by the live UDP loop and --replay-servo.
void step_fleet(skysim::core::World &world, std::vector<std::unique_ptr<VehicleSlot>> &fleet, double dt_s,
                FILE *truth_log, FILE *record_log) {
    const auto wind = world.wind_ned();
    for (size_t i = 0; i < fleet.size(); ++i) {
        auto &v = *fleet[i];
        if (record_log != nullptr) {
            std::fprintf(record_log, "%llu,%zu,%u", static_cast<unsigned long long>(world.tick_index()), i,
                         v.pending.frame_rate_hz);
            for (int k = 0; k < 16; ++k) {
                std::fprintf(record_log, ",%u", v.pending.pwm[k]);
            }
            std::fputc('\n', record_log);
        }
        // Airspeed = velocity relative to the air mass (steady wind + gusts), body FRD.
        const skysim::core::Vec3 v_air_ned{v.state.vel_ned[0] - wind[0], v.state.vel_ned[1] - wind[1],
                                           v.state.vel_ned[2] - wind[2]};
        const auto vel_frd = skysim::core::quat_rotate_inverse(v.state.quat_ned_frd, v_air_ned);
        std::array<uint16_t, 16> pwm{};
        std::copy_n(v.pending.pwm.begin(), 16, pwm.begin());
        const double vel_air_frd[3] = {vel_frd[0], vel_frd[1], vel_frd[2]};
        const auto wrench = skysim::vehicle::compute_wrench(v.params, v.motors, pwm, vel_air_frd, dt_s);
        world.apply_body_wrench(v.body_id, {wrench.force_frd[0], wrench.force_frd[1], wrench.force_frd[2]},
                                {wrench.torque_frd[0], wrench.torque_frd[1], wrench.torque_frd[2]});
    }
    world.step();
    const double now_s = world.now();
    for (size_t i = 0; i < fleet.size(); ++i) {
        fleet[i]->state = world.get_state(fleet[i]->body_id);
        if (truth_log != nullptr) {
            const auto &pos = fleet[i]->state.pos_ned;
            std::fprintf(truth_log, "%.4f,%zu,%.4f,%.4f,%.4f\n", now_s, i, pos[0], pos[1], pos[2]);
        }
    }
}

// --replay-servo: drive the world from a recorded CSV (tick,vehicle,frame_rate,pwm0..15),
// no network at all. Same spawn conditions + same seed => bit-identical truth log.
int run_replay(const Options &opt, skysim::core::World &world,
               std::vector<std::unique_ptr<VehicleSlot>> &fleet, FILE *truth_log) {
    FILE *in = std::fopen(opt.replay_servo.c_str(), "r");
    if (in == nullptr) {
        std::fprintf(stderr, "skysim: cannot open --replay-servo %s\n", opt.replay_servo.c_str());
        return 1;
    }
    char line[512];
    size_t staged = 0, steps = 0;
    while (std::fgets(line, sizeof(line), in) != nullptr) {
        unsigned long long tick;
        size_t vi;
        unsigned rate;
        int consumed = 0;
        if (std::sscanf(line, "%llu,%zu,%u%n", &tick, &vi, &rate, &consumed) != 3 || vi >= fleet.size()) {
            std::fprintf(stderr, "skysim: bad replay line: %s", line);
            std::fclose(in);
            return 1;
        }
        auto &p = fleet[vi]->pending;
        p.frame_rate_hz = static_cast<uint16_t>(rate);
        const char *cursor = line + consumed;
        for (int k = 0; k < 16; ++k) {
            unsigned pwm_val = 0;
            int used = 0;
            if (std::sscanf(cursor, ",%u%n", &pwm_val, &used) != 1) {
                std::fprintf(stderr, "skysim: bad replay pwm at line: %s", line);
                std::fclose(in);
                return 1;
            }
            p.pwm[k] = static_cast<uint16_t>(pwm_val);
            cursor += used;
        }
        if (++staged == fleet.size()) {
            step_fleet(world, fleet, opt.dt_s, truth_log, nullptr);
            staged = 0;
            ++steps;
        }
    }
    std::fclose(in);
    std::printf("skysim: replay done, %zu steps\n", steps);
    return 0;
}

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
    wcfg.wind_steady_ned = opt.wind_ned;
    wcfg.gust_sigma_mps = opt.gust_sigma;
    wcfg.gust_tau_s = opt.gust_tau;
    wcfg.rng_seed = opt.seed;
    std::unique_ptr<skysim::core::World> world;
    if (!opt.canned) {
        world = std::make_unique<skysim::core::World>(wcfg);
        world->add_ground_plane();
        if (!opt.tiles.empty()) {
            const size_t n = world->load_tiles(opt.tiles);
            std::printf("skysim: loaded %zu tile(s) from %s\n", n, opt.tiles.c_str());
            if (n == 0) {
                // A requested map that silently loads empty is a phantom-free world — abort.
                std::fprintf(stderr, "skysim: --tiles %s yielded no tiles\n", opt.tiles.c_str());
                return 1;
            }
        }
    }

    const bool replaying = !opt.replay_servo.empty();
    if (replaying && !world) {
        std::fprintf(stderr, "skysim: --replay-servo requires jolt physics (not --canned)\n");
        return 1;
    }

    std::vector<std::unique_ptr<VehicleSlot>> fleet;
    for (int i = 0; i < opt.vehicles; ++i) {
        auto slot = replaying ? std::make_unique<VehicleSlot>()
                              : std::make_unique<VehicleSlot>(opt.base_instance + i);
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
        if (slot->endpoint) {
            std::printf("skysim: vehicle %d listening on udp %d\n", i, slot->endpoint->port());
        }
        fleet.push_back(std::move(slot));
    }
    std::printf("skysim: %d vehicle(s), dt=%.6f s, %s mode, %s\n", opt.vehicles, opt.dt_s,
                opt.time_mode.c_str(), opt.canned ? "canned physics (M1)" : "jolt physics");
    std::fflush(stdout);

    FILE *truth_log = nullptr;
    if (!opt.truth_log.empty()) {
        truth_log = std::fopen(opt.truth_log.c_str(), "w");
        if (truth_log == nullptr) {
            std::fprintf(stderr, "skysim: cannot open --truth-log %s\n", opt.truth_log.c_str());
            return 1;
        }
        std::fprintf(truth_log, "t,vehicle,north,east,down\n");
    }

    if (replaying) {
        const int rc = run_replay(opt, *world, fleet, truth_log);
        if (truth_log != nullptr) {
            std::fclose(truth_log);
        }
        return rc;
    }

    FILE *record_log = nullptr;
    if (!opt.record_servo.empty()) {
        record_log = std::fopen(opt.record_servo.c_str(), "w");
        if (record_log == nullptr) {
            std::fprintf(stderr, "skysim: cannot open --record-servo %s\n", opt.record_servo.c_str());
            return 1;
        }
    }

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
                    // Normal during handshake (docs/PROTOCOL.md): don't re-step, do re-reply —
                    // but only if this re-send arrived AFTER our last reply. A duplicate that
                    // was already answered must not be answered twice (lockstep desync).
                    if (v->last_json_len > 0 && v->endpoint->last_taken_unanswered()) {
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
            step_fleet(*world, fleet, opt.dt_s, truth_log, record_log);
            now_s = world->now();
        } else {
            canned_time += opt.dt_s;
            now_s = canned_time;
        }

        for (auto &v : fleet) {
            skysim::protocol::VehicleTruth truth;
            if (world) {
                truth = truth_from_state(v->state, now_s);
                if (opt.rangefinder) {
                    // Down-facing rangefinder: body +z (FRD) in world frame, 40 m max range.
                    const auto down_world = skysim::core::quat_rotate(v->state.quat_ned_frd, {0.0, 0.0, 1.0});
                    const double d = world->raycast(v->state.pos_ned, down_world, 40.0, v->body_id);
                    truth.rangefinder_m[0] = d < 0.0 ? 40.0 : d;
                    truth.rangefinder_count = 1;
                }
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

    if (truth_log != nullptr) {
        std::fclose(truth_log);
    }
    if (record_log != nullptr) {
        std::fclose(record_log);
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
