// skysim entry point: config -> World -> UdpEndpoints/ControlServer -> tick loop.
// Time policy (docs/DESIGN.md): strict = barrier on every vehicle's next frame (abort on
// timeout); interactive = wall-clock paced, stragglers hold->freeze->grace-despawn.
// --canned keeps the M1 kinematic reply path; --replay-servo re-runs a recorded input tape.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "api/control_server.h"
#include "core/metrics.h"
#include "core/world.h"
#include "protocol/packets.h"
#include "protocol/udp_endpoint.h"
#include "vehicle/manager.h"
#include "vehicle/quad.h"

namespace {

struct Options {
    int vehicles = 1;
    int base_instance = 0;
    double dt_s = 1.0 / 400.0;
    std::string time_mode = "strict";
    std::string tiles;
    std::string truth_log;
    std::string record_servo;
    std::string replay_servo;
    skysim::core::Vec3 wind_ned{0.0, 0.0, 0.0};
    double gust_sigma = 0.0;
    double gust_tau = 2.0;
    uint64_t seed = 1;
    bool rangefinder = false;
    bool canned = false;
    // M4: control plane + straggler policy + managed SITL processes.
    int api_port = 0;               // 0 = control plane disabled
    int hold_ticks = 3;             // interactive: reuse last PWM for up to k missed deadlines
    double grace_s = 30.0;          // interactive: frozen longer than this => auto-despawn
    double strict_timeout_s = 10.0; // strict: barrier stalled longer => abort with report
    std::string spawn_binary;       // arducopter for POST /vehicles {"launch_process":true}
    std::string spawn_home = "42.1354,24.7453,164,0";
    std::string spawn_defaults;
    double spacing_m = 5.0; // east spacing between spawn positions (fleet airspace layout)
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
            if (std::sscanf(need_value("--gust"), "%lf,%lf", &o.gust_sigma, &o.gust_tau) < 1) {
                std::fprintf(stderr, "skysim: --gust expects sigma[,tau]\n");
                std::exit(2);
            }
        } else if (std::strcmp(argv[i], "--seed") == 0) {
            o.seed = std::strtoull(need_value("--seed"), nullptr, 10);
        } else if (std::strcmp(argv[i], "--rangefinder") == 0) {
            o.rangefinder = true;
        } else if (std::strcmp(argv[i], "--canned") == 0) {
            o.canned = true;
        } else if (std::strcmp(argv[i], "--api-port") == 0) {
            o.api_port = std::atoi(need_value("--api-port"));
        } else if (std::strcmp(argv[i], "--hold-ticks") == 0) {
            o.hold_ticks = std::atoi(need_value("--hold-ticks"));
        } else if (std::strcmp(argv[i], "--grace") == 0) {
            o.grace_s = std::atof(need_value("--grace"));
        } else if (std::strcmp(argv[i], "--strict-timeout") == 0) {
            o.strict_timeout_s = std::atof(need_value("--strict-timeout"));
        } else if (std::strcmp(argv[i], "--spawn-binary") == 0) {
            o.spawn_binary = need_value("--spawn-binary");
        } else if (std::strcmp(argv[i], "--spawn-home") == 0) {
            o.spawn_home = need_value("--spawn-home");
        } else if (std::strcmp(argv[i], "--spawn-defaults") == 0) {
            o.spawn_defaults = need_value("--spawn-defaults");
        } else if (std::strcmp(argv[i], "--spacing") == 0) {
            o.spacing_m = std::atof(need_value("--spacing"));
        } else {
            std::fprintf(stderr, "skysim: unknown arg %s\n", argv[i]);
            std::exit(2);
        }
    }
    if (o.vehicles < 0 || o.dt_s <= 0.0 || (o.time_mode != "strict" && o.time_mode != "interactive")) {
        std::fprintf(stderr, "skysim: invalid options (--time-mode strict|interactive)\n");
        std::exit(2);
    }
    return o;
}

std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop.store(true); }

// One vehicle: endpoint (null when replaying) + body + motors + straggler bookkeeping.
struct VehicleSlot {
    std::unique_ptr<skysim::protocol::UdpEndpoint> endpoint;
    uint32_t vehicle_id = 0; // app-level id (control plane)
    int instance = 0;
    std::optional<pid_t> pid; // set when this slot's arducopter is managed by us
    uint32_t body_id = 0;
    skysim::vehicle::QuadParams params;
    std::array<skysim::vehicle::MotorState, 4> motors{};
    skysim::core::BodyState state;
    char last_json[1024];
    size_t last_json_len = 0;
    bool has_pending_input = false; // fresh frame consumed, reply owed after this tick
    skysim::protocol::ParsedServos pending{};
    bool connected = false; // ever received a frame
    int hold_ticks = 0;     // consecutive deadline misses (interactive)
    bool frozen = false;
    uint64_t frozen_at_tick = 0;
    uint64_t gaps = 0, reboots = 0, bad = 0, ticks = 0;

    VehicleSlot() = default;
    explicit VehicleSlot(int inst)
        : endpoint(std::make_unique<skysim::protocol::UdpEndpoint>(inst)), instance(inst) {}
};

using Fleet = std::vector<std::unique_ptr<VehicleSlot>>;

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

// Apply wrenches for every active vehicle and advance the world one tick.
// A slot participates with its `pending` PWM whether fresh (has_pending_input) or held.
void step_world(skysim::core::World &world, Fleet &fleet, double dt_s, FILE *truth_log, FILE *record_log) {
    const auto wind = world.wind_ned();
    for (size_t i = 0; i < fleet.size(); ++i) {
        auto &v = *fleet[i];
        if (v.frozen || !v.connected) {
            continue; // kinematic hold / never-armed slot: gravity + contacts only
        }
        if (record_log != nullptr && v.has_pending_input) {
            std::fprintf(record_log, "%llu,%zu,%u", static_cast<unsigned long long>(world.tick_index()), i,
                         v.pending.frame_rate_hz);
            for (int k = 0; k < 16; ++k) {
                std::fprintf(record_log, ",%u", v.pending.pwm[k]);
            }
            std::fputc('\n', record_log);
        }
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

// Poll one endpoint; returns true if a fresh steppable frame was staged.
bool poll_endpoint(VehicleSlot &v) {
    using skysim::protocol::FrameStatus;
    if (v.has_pending_input || !v.endpoint) {
        return v.has_pending_input;
    }
    if (auto p = v.endpoint->take_latest()) {
        switch (p->status) {
        case FrameStatus::kOk:
        case FrameStatus::kGap:
        case FrameStatus::kReboot:
            if (p->status == FrameStatus::kGap) {
                ++v.gaps;
            }
            if (p->status == FrameStatus::kReboot) {
                ++v.reboots;
                std::printf("skysim: vehicle %u reboot detected (frame_count %u); pose kept\n", v.vehicle_id,
                            p->frame_count);
            }
            v.pending = *p;
            v.has_pending_input = true;
            v.connected = true;
            break;
        case FrameStatus::kDuplicate:
            // Handshake duplicates must be re-answered, already-answered ones must not
            // (lockstep desync — see udp_endpoint.h).
            if (v.last_json_len > 0 && v.endpoint->last_taken_unanswered()) {
                v.endpoint->send_reply(v.last_json, v.last_json_len);
            }
            break;
        case FrameStatus::kBadMagic:
        case FrameStatus::kBadSize:
            ++v.bad;
            break;
        }
    }
    return v.has_pending_input;
}

// Build + send the reply for every vehicle whose fresh input was consumed this tick.
int send_replies(skysim::core::World *world, Fleet &fleet, const Options &opt, double now_s) {
    for (auto &v : fleet) {
        if (!v->has_pending_input) {
            continue;
        }
        skysim::protocol::VehicleTruth truth;
        if (world != nullptr) {
            truth = truth_from_state(v->state, now_s);
            if (opt.rangefinder) {
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
        if (v->endpoint) {
            v->endpoint->send_reply(v->last_json, v->last_json_len);
        }
        v->has_pending_input = false;
        ++v->ticks;
    }
    return 0;
}

int run_replay(const Options &opt, skysim::core::World &world, Fleet &fleet, FILE *truth_log) {
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
        auto &v = *fleet[vi];
        v.pending.frame_rate_hz = static_cast<uint16_t>(rate);
        const char *cursor = line + consumed;
        for (int k = 0; k < 16; ++k) {
            unsigned pwm_val = 0;
            int used = 0;
            if (std::sscanf(cursor, ",%u%n", &pwm_val, &used) != 1) {
                std::fprintf(stderr, "skysim: bad replay pwm at line: %s", line);
                std::fclose(in);
                return 1;
            }
            v.pending.pwm[k] = static_cast<uint16_t>(pwm_val);
            cursor += used;
        }
        v.has_pending_input = true;
        v.connected = true;
        if (++staged == fleet.size()) {
            step_world(world, fleet, opt.dt_s, truth_log, nullptr);
            for (auto &s : fleet) {
                s->has_pending_input = false;
            }
            staged = 0;
            ++steps;
        }
    }
    std::fclose(in);
    std::printf("skysim: replay done, %zu steps\n", steps);
    return 0;
}

// The whole mutable app state the control plane interacts with (tick thread owns it all).
struct App {
    Options opt;
    std::unique_ptr<skysim::core::World> world;
    std::unique_ptr<skysim::vehicle::VehicleManager> manager;
    Fleet fleet;
    skysim::api::CommandQueue queue;
    skysim::core::TickMetrics metrics;
    uint32_t next_vehicle_id = 1;

    std::mutex snapshot_mutex;
    std::vector<skysim::api::VehicleInfo> vehicle_snapshot;
    skysim::api::MetricsInfo metrics_snapshot;

    std::unique_ptr<VehicleSlot> make_slot(int instance) {
        auto slot = std::make_unique<VehicleSlot>(instance);
        slot->vehicle_id = next_vehicle_id++;
        skysim::core::VehicleBodyParams bp;
        bp.mass_kg = slot->params.mass_kg;
        bp.inertia_diag_frd = {slot->params.inertia_diag[0], slot->params.inertia_diag[1],
                               slot->params.inertia_diag[2]};
        // Spawn on a 5 m east-spaced line by instance offset (deterministic, collision-free).
        bp.start_pos_ned = {0.0, opt.spacing_m * (instance - opt.base_instance),
                            -(bp.half_extents_frd[2] + 0.005)};
        slot->body_id = world->add_vehicle(bp);
        slot->state = world->get_state(slot->body_id);
        return slot;
    }

    void despawn(size_t idx) {
        auto &v = *fleet[idx];
        std::printf("skysim: despawning vehicle %u (instance %d)\n", v.vehicle_id, v.instance);
        if (v.pid.has_value()) {
            manager->stop_sitl(*v.pid);
        }
        world->remove_vehicle(v.body_id);
        manager->release_instance(v.instance);
        fleet.erase(fleet.begin() + static_cast<long>(idx));
    }

    // Drain control-plane commands at the tick boundary (DESIGN.md threading step 1).
    void drain_commands() {
        for (auto &cmd : queue.drain()) {
            if (auto *spawn = std::get_if<skysim::api::SpawnCommand>(&cmd)) {
                std::optional<skysim::vehicle::SpawnResult> result;
                const int instance = manager->allocate_instance();
                try {
                    auto slot = make_slot(instance);
                    if (spawn->request.launch_process) {
                        slot->pid = manager->launch_sitl(instance);
                        if (!slot->pid.has_value()) {
                            world->remove_vehicle(slot->body_id);
                            throw std::runtime_error("launch failed");
                        }
                    }
                    result = skysim::vehicle::SpawnResult{slot->vehicle_id, instance, 9002 + 10 * instance,
                                                          5760 + 10 * instance};
                    std::printf("skysim: spawned vehicle %u on instance %d\n", slot->vehicle_id, instance);
                    fleet.push_back(std::move(slot));
                } catch (const std::exception &e) {
                    std::fprintf(stderr, "skysim: spawn failed: %s\n", e.what());
                    manager->release_instance(instance);
                }
                spawn->done.set_value(result);
            } else if (auto *despawn_cmd = std::get_if<skysim::api::DespawnCommand>(&cmd)) {
                bool found = false;
                for (size_t i = 0; i < fleet.size(); ++i) {
                    if (fleet[i]->vehicle_id == despawn_cmd->id) {
                        despawn(i);
                        found = true;
                        break;
                    }
                }
                despawn_cmd->done.set_value(found);
            }
        }
        manager->reap();
    }

    void publish_snapshot() {
        std::vector<skysim::api::VehicleInfo> infos;
        infos.reserve(fleet.size());
        for (const auto &v : fleet) {
            skysim::api::VehicleInfo info;
            info.id = v->vehicle_id;
            info.instance = v->instance;
            info.json_port = 9002 + 10 * v->instance;
            info.mavlink_tcp_port = 5760 + 10 * v->instance;
            info.connected = v->connected;
            info.frozen = v->frozen;
            info.held_ticks = static_cast<uint64_t>(v->hold_ticks);
            for (int k = 0; k < 3; ++k) {
                info.pos_ned[k] = v->state.pos_ned[k];
            }
            infos.push_back(info);
        }
        skysim::api::MetricsInfo m;
        const auto snap = metrics.snapshot();
        m.tick_p50_us = snap.p50_us;
        m.tick_p99_us = snap.p99_us;
        m.straggler_events = snap.straggler_events;
        m.freezes = snap.freezes;
        m.tick = world ? world->tick_index() : 0;
        m.sim_time_s = world ? world->now() : 0.0;
        m.vehicles = fleet.size();
        std::lock_guard<std::mutex> lock(snapshot_mutex);
        vehicle_snapshot = std::move(infos);
        metrics_snapshot = m;
    }
};

// Strict: barrier on every CONNECTED vehicle's next frame; not-yet-connected vehicles don't
// block (they sit on the ground until their SITL shows up). Abort on stalled barrier.
int run_strict(App &app, FILE *truth_log, FILE *record_log) {
    using clock = std::chrono::steady_clock;
    auto barrier_stalled_since = clock::now();
    bool barrier_was_complete = true;
    while (!g_stop.load(std::memory_order_relaxed)) {
        app.drain_commands();

        bool all_fresh = true;
        bool any_fresh = false;
        for (auto &v : app.fleet) {
            const bool fresh = poll_endpoint(*v);
            any_fresh = any_fresh || fresh;
            if (v->connected && !fresh) {
                all_fresh = false;
            }
        }
        const bool ready = all_fresh && any_fresh;
        if (!ready) {
            if (barrier_was_complete) {
                barrier_was_complete = false;
                barrier_stalled_since = clock::now();
            } else if (app.opt.strict_timeout_s > 0.0 && any_fresh) {
                // Someone is waiting on someone else: that's a genuine straggler stall.
                const double stalled =
                    std::chrono::duration<double>(clock::now() - barrier_stalled_since).count();
                if (stalled > app.opt.strict_timeout_s) {
                    std::fprintf(stderr, "skysim: STRICT ABORT after %.1f s — stalled vehicles:\n", stalled);
                    for (const auto &v : app.fleet) {
                        if (v->connected && !v->has_pending_input) {
                            std::fprintf(stderr, "  vehicle %u (instance %d, udp %d): no frame\n",
                                         v->vehicle_id, v->instance, 9002 + 10 * v->instance);
                        }
                    }
                    return 3;
                }
            }
            std::this_thread::sleep_for(std::chrono::microseconds(50));
            continue;
        }
        barrier_was_complete = true;

        const auto t0 = clock::now();
        step_world(*app.world, app.fleet, app.opt.dt_s, truth_log, record_log);
        app.metrics.record_us(std::chrono::duration<double, std::micro>(clock::now() - t0).count());
        if (const int rc = send_replies(app.world.get(), app.fleet, app.opt, app.world->now())) {
            return rc;
        }
        app.publish_snapshot();
    }
    return 0;
}

// Interactive: tick on schedule; a vehicle missing its deadline gets its last PWM held for
// up to k ticks, then freezes (kinematic hold, flagged), then despawns after the grace.
int run_interactive(App &app, FILE *truth_log, FILE *record_log) {
    using clock = std::chrono::steady_clock;
    const auto dt = std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(app.opt.dt_s));
    auto next_tick = clock::now() + dt;
    const uint64_t grace_ticks = static_cast<uint64_t>(app.opt.grace_s / app.opt.dt_s);

    while (!g_stop.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_until(next_tick);
        next_tick += dt;
        if (clock::now() > next_tick + 100 * dt) {
            next_tick = clock::now() + dt; // fell far behind (debugger, suspend): resync
        }

        app.drain_commands();

        for (auto &v : app.fleet) {
            const bool fresh = poll_endpoint(*v);
            if (fresh) {
                if (v->frozen) {
                    std::printf("skysim: vehicle %u thawed (frames resumed)\n", v->vehicle_id);
                    v->frozen = false;
                    app.world->set_frozen(v->body_id, false);
                }
                v->hold_ticks = 0;
            } else if (v->connected && !v->frozen) {
                ++v->hold_ticks;
                ++app.metrics.straggler_events;
                if (v->hold_ticks > app.opt.hold_ticks) {
                    std::printf("skysim: vehicle %u FROZEN after %d held ticks\n", v->vehicle_id,
                                v->hold_ticks);
                    v->frozen = true;
                    v->frozen_at_tick = app.world->tick_index();
                    ++app.metrics.freezes;
                    app.world->set_frozen(v->body_id, true);
                }
            }
        }

        const auto t0 = clock::now();
        step_world(*app.world, app.fleet, app.opt.dt_s, truth_log, record_log);
        app.metrics.record_us(std::chrono::duration<double, std::micro>(clock::now() - t0).count());
        if (const int rc = send_replies(app.world.get(), app.fleet, app.opt, app.world->now())) {
            return rc;
        }

        // Grace expiry (reverse order: despawn erases by index).
        for (size_t i = app.fleet.size(); i-- > 0;) {
            auto &v = *app.fleet[i];
            if (v.frozen && app.world->tick_index() - v.frozen_at_tick > grace_ticks) {
                std::printf("skysim: vehicle %u exceeded grace (%.0f s frozen) — despawning\n", v.vehicle_id,
                            app.opt.grace_s);
                app.despawn(i);
            }
        }
        app.publish_snapshot();
    }
    return 0;
}

// M1 canned mode: kinematic on-ground reply, no world. Kept for protocol debugging.
int run_canned(App &app) {
    double canned_time = 0.0;
    while (!g_stop.load(std::memory_order_relaxed)) {
        bool any = false;
        for (auto &v : app.fleet) {
            if (poll_endpoint(*v)) {
                any = true;
            }
        }
        if (!any) {
            std::this_thread::sleep_for(std::chrono::microseconds(50));
            continue;
        }
        canned_time += app.opt.dt_s;
        if (const int rc = send_replies(nullptr, app.fleet, app.opt, canned_time)) {
            return rc;
        }
    }
    return 0;
}

} // namespace

int main(int argc, char **argv) {
    App app;
    app.opt = parse_args(argc, argv);
    const Options &opt = app.opt;
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    std::signal(SIGPIPE, SIG_IGN);

    skysim::core::WorldConfig wcfg;
    wcfg.dt_s = opt.dt_s;
    wcfg.wind_steady_ned = opt.wind_ned;
    wcfg.gust_sigma_mps = opt.gust_sigma;
    wcfg.gust_tau_s = opt.gust_tau;
    wcfg.rng_seed = opt.seed;
    if (!opt.canned) {
        app.world = std::make_unique<skysim::core::World>(wcfg);
        app.world->add_ground_plane();
        if (!opt.tiles.empty()) {
            const size_t n = app.world->load_tiles(opt.tiles);
            std::printf("skysim: loaded %zu tile(s) from %s\n", n, opt.tiles.c_str());
            if (n == 0) {
                std::fprintf(stderr, "skysim: --tiles %s yielded no tiles\n", opt.tiles.c_str());
                return 1;
            }
        }
    }

    skysim::vehicle::ProcessConfig pcfg;
    pcfg.binary = opt.spawn_binary;
    pcfg.home = opt.spawn_home;
    pcfg.defaults = opt.spawn_defaults;
    pcfg.run_dir = "/tmp/skysim_managed";
    app.manager = std::make_unique<skysim::vehicle::VehicleManager>(opt.base_instance, pcfg);

    const bool replaying = !opt.replay_servo.empty();
    if (replaying && (!app.world || opt.time_mode != "strict")) {
        std::fprintf(stderr, "skysim: --replay-servo requires strict mode + jolt physics\n");
        return 1;
    }

    for (int i = 0; i < opt.vehicles; ++i) {
        const int instance = app.manager->allocate_instance();
        if (opt.canned) {
            auto slot = std::make_unique<VehicleSlot>(instance);
            slot->vehicle_id = app.next_vehicle_id++;
            app.fleet.push_back(std::move(slot));
        } else if (replaying) {
            auto slot = std::make_unique<VehicleSlot>();
            slot->vehicle_id = app.next_vehicle_id++;
            slot->instance = instance;
            skysim::core::VehicleBodyParams bp;
            bp.mass_kg = slot->params.mass_kg;
            bp.inertia_diag_frd = {slot->params.inertia_diag[0], slot->params.inertia_diag[1],
                                   slot->params.inertia_diag[2]};
            bp.start_pos_ned = {0.0, opt.spacing_m * i, -(bp.half_extents_frd[2] + 0.005)};
            slot->body_id = app.world->add_vehicle(bp);
            slot->state = app.world->get_state(slot->body_id);
            app.fleet.push_back(std::move(slot));
        } else {
            app.fleet.push_back(app.make_slot(instance));
        }
        if (app.fleet.back()->endpoint) {
            std::printf("skysim: vehicle %u listening on udp %d\n", app.fleet.back()->vehicle_id,
                        app.fleet.back()->endpoint->port());
        }
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
    FILE *record_log = nullptr;
    if (!opt.record_servo.empty()) {
        record_log = std::fopen(opt.record_servo.c_str(), "w");
        if (record_log == nullptr) {
            std::fprintf(stderr, "skysim: cannot open --record-servo %s\n", opt.record_servo.c_str());
            return 1;
        }
    }

    std::unique_ptr<skysim::api::ControlServer> api;
    if (opt.api_port > 0 && !opt.canned && !replaying) {
        skysim::api::ControlServer::Snapshots snaps;
        snaps.vehicles = [&app] {
            std::lock_guard<std::mutex> lock(app.snapshot_mutex);
            return app.vehicle_snapshot;
        };
        snaps.metrics = [&app] {
            std::lock_guard<std::mutex> lock(app.snapshot_mutex);
            return app.metrics_snapshot;
        };
        api = std::make_unique<skysim::api::ControlServer>(opt.api_port, app.queue, std::move(snaps));
    }

    int rc = 0;
    if (replaying) {
        rc = run_replay(opt, *app.world, app.fleet, truth_log);
    } else if (opt.canned) {
        rc = run_canned(app);
    } else if (opt.time_mode == "interactive") {
        rc = run_interactive(app, truth_log, record_log);
    } else {
        rc = run_strict(app, truth_log, record_log);
    }

    api.reset(); // stop HTTP thread before tearing down the fleet it reads
    if (truth_log != nullptr) {
        std::fclose(truth_log);
    }
    if (record_log != nullptr) {
        std::fclose(record_log);
    }
    for (size_t i = 0; i < app.fleet.size(); ++i) {
        std::printf("skysim: vehicle %zu ticks=%llu gaps=%llu reboots=%llu bad=%llu\n", i,
                    static_cast<unsigned long long>(app.fleet[i]->ticks),
                    static_cast<unsigned long long>(app.fleet[i]->gaps),
                    static_cast<unsigned long long>(app.fleet[i]->reboots),
                    static_cast<unsigned long long>(app.fleet[i]->bad));
    }
    return rc;
}
