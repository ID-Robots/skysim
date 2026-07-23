# skysim

Headless, CPU-parallel multi-vehicle physics server for ArduPilot SITL (`--model JSON`),
built on [Jolt Physics]. A Gazebo replacement scoped to one job: many quads over a large
scanned-city mesh, runtime spawn/despawn, lockstep determinism when you want it, wall-clock
realtime when operators are in the loop. Vehicles are real, unmodified `arducopter`
processes exposing plain MAVLink, so SkyHub connects to the fleet exactly as it would to
real aircraft.

Read in order: `CLAUDE.md` → `docs/PROTOCOL.md` → `docs/DESIGN.md` → `docs/MILESTONES.md`.

## Status (2026-07-23)

| Milestone | State | Acceptance evidence (in the commit message) |
|-----------|-------|---------------------------------------------|
| M0 constants & repo boot | ✅ | `42ae9c9` `a088083` — Copter-4.7.0 pinned, ABI verified, real fixture |
| M1 protocol layer        | ✅ | `b962d6b` — 60 s soak: 61 heartbeats, EKF origin, 0 warnings |
| M2 single quad flies     | ✅ | `40af291` — full mission PASS; hover pwm 1573 vs stock 1578 |
| M3 wind / rangefinder / determinism | ✅ | `47b4982` — live == replayA == replayB (sha256) |
| M4 multi-vehicle lifecycle | ✅ | `1cad5d3` — churn 100×, SIGSTOP freeze/abort, 10-vehicle mission |
| M5 city tiles & streaming | ✅ | `e743305` `b2ec28e` — corridor walls @30 m, tile bound held, zero tunneling |
| M6 scale & SkyHub        | ✅ | ~200 quads @ 800 Hz w/ ~5× headroom; SkyHub gateway spawns SITL into one shared world, mid-air collisions surface live in the dashboard. WS `/state` deferred — crash telemetry rides `GET /vehicles` polling instead (see below). |

Unit coverage: **~87% lines** on `src/` (CI gates at 60%). Pending human sign-off (flight
model): `k_thrust=7.65e-6`, linear-only drag, `SIM_RATE_HZ=800` + `--dt 1/800` for
conformance (ArduPilot prearm requires gyro rate ≥ 1.8× loop rate).

### Performance

Profiled with `tools/bench/{world_bench,proto_bench}`; details and the measured tick
breakdown live in `docs/DESIGN.md`. Four optimizations landed: physics job system
single-threaded by default (Jolt thread jitter is a net loss below ~1000 bodies), an
integer fixed-precision JSON formatter (6.3× faster than the old `snprintf`), O(1) contact
attribution, and a reply path (build + `sendto`) that fans across `--io-threads` cores
above 48 vehicles. **One world sustains ~200 quads at 800 Hz on a single physics core with
~5× realtime headroom**; scale past that by sharding worlds across processes (`--physics-threads`
opts a single huge world into a Jolt thread pool, only worth it past ~1000 bodies).

## How it works (one paragraph)

Each `arducopter` outsources physics over UDP: it sends a servo packet (16 PWM values) to
`9002 + 10·I` and blocks until skysim replies. skysim turns PWM into motor thrust (spin-up
lag, X-quad mixer verified against the pinned ArduPilot source), steps one shared Jolt
world by a fixed dt, and replies with the truth state (IMU specific force, position,
velocity, attitude, optional rangefinder raycasts) as one JSON line — whose timestamp IS
ArduPilot's clock (lockstep). Strict mode barriers on every vehicle for determinism;
interactive mode ticks on the wall clock and freezes stragglers (kinematic hold) instead
of stalling the fleet. City tiles are cooked offline to Jolt mesh shapes and streamed
in/out by vehicle proximity under a hard residency cap.

## Prerequisites

- Ubuntu 22.04/24.04, CMake ≥ 3.24, Ninja, GCC 12+ or Clang 16+
- ArduPilot checkout built for SITL, exported as `ARDUPILOT_ROOT` (tag pinned in
  `docs/PROTOCOL.md`, currently **Copter-4.7.0**):

  ```bash
  git clone --recurse-submodules https://github.com/ArduPilot/ardupilot ~/ardupilot
  cd ~/ardupilot && git checkout Copter-4.7.0 && git submodule update --init --recursive
  ./waf configure --board sitl && ./waf copter
  export ARDUPILOT_ROOT=~/ardupilot   # add to your shell profile
  ```

- `pip install pymavlink` (harnesses), `gcovr` (coverage), `trimesh` (real-mesh pre-step, M5+)

## Build & test

```bash
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j
ctest --test-dir build --output-on-failure   # 12 suites, no SITL required
tools/coverage.sh                            # gcovr report, fails under 60% lines
```

Live acceptance harnesses (need `ARDUPILOT_ROOT`; use `--base-instance 1`+ if something
already owns the instance-0 SITL ports):

```bash
python3 tools/harness/conformance.py --base-instance 1 [--vehicles 10]   # mission gate
python3 tools/harness/churn.py --sim ./build/skysim                      # spawn/despawn 100×
PYTHONPATH=tools/harness python3 tools/harness/determinism.py --base-instance 1
PYTHONPATH=tools/harness python3 tools/harness/straggler.py   --base-instance 1
PYTHONPATH=tools/harness python3 tools/harness/collision_demo.py --base-instance 1
PYTHONPATH=tools/harness python3 tools/harness/m5_corridor.py   --base-instance 1
```

## Running the sim

```bash
# demo map (cook once)
python3 tools/cooker/pretile.py build/demo_map_obj
./build/tile_cooker build/demo_map/tiles build/demo_map_obj/*.obj

./build/skysim \
  --vehicles 2 --base-instance 1 \        # endpoints on udp 9012, 9022
  --time-mode interactive \               # or: strict (CI/replays; barrier + abort)
  --dt 0.00125 \                          # pairs with SIM_RATE_HZ=800 overlay
  --tiles build/demo_map/tiles \          # streamed: --stream-radius / --stream-max
  --rangefinders 2 \                      # rng_1 down, rng_2 right
  --wind 4,1,0 --gust 1.5,2 --seed 42 \   # steady + seeded OU gusts
  --api-port 8642                         # REST control plane
# then per vehicle:
$ARDUPILOT_ROOT/build/sitl/bin/arducopter --model json:127.0.0.1 -I 1 \
  --home 42.1354,24.7453,164,0 \
  --defaults $ARDUPILOT_ROOT/Tools/autotest/default_params/copter.parm,tools/harness/params/skysim.parm
```

Useful extras: `--truth-log out.csv` (ground truth per tick — judge physics with this, not
the EKF), `--record-servo` / `--replay-servo` (deterministic input tapes), `--canned`
(M1 protocol-debug mode).

### Control plane (`--api-port`)

| Endpoint | Effect |
|----------|--------|
| `POST /vehicles` `{"launch_process":true?}` | spawn at next free instance (optionally forks arducopter) → `{id, instance, json_port, mavlink_tcp}` |
| `DELETE /vehicles/{id}` | despawn, release instance, kill managed process |
| `GET /vehicles` | per-vehicle: connected, frozen, held_ticks, pos_ned, `midair_collisions` |
| `GET /metrics` | tick p50/p99 µs, straggler_events, freezes, resident_tiles |

All mutations execute at tick boundaries; reads come from per-tick snapshots. The SkyHub
gateway polls `GET /vehicles` and, on a `midair_collisions` increase, emits a `crash_alert`
to the dashboard — that is the live mid-air-collision path until WS `/state` lands.

## Repo map

```
src/protocol/   servo packet ABI + state JSON emitter + UDP endpoints (no Jolt)
src/core/       frames.h (NED/FRD <-> Jolt), world.cpp (the ONLY Jolt-aware TU), metrics
src/vehicle/    motor lag + X-quad mixer + instance allocator / process manager
src/terrain/    OBJ -> MeshShape cooker + proximity tile streamer
src/api/        REST control plane (cpp-httplib)
tools/cooker/   pretile.py (demo city generator) + tile_cooker CLI
tools/bench/    world_bench (tick breakdown), proto_bench (JSON/parse hot path)
tools/harness/  conformance / determinism / straggler / churn / collision / corridor
tests/          12 ctest suites incl. app_smoke.py driving the real binary + thread_pool
```

Notable interop findings baked into `docs/PROTOCOL.md`: ArduPilot's strstr JSON parser
rules (field ordering, compact booleans), the two-line handshake (always reply, even to
duplicates), and an upstream Copter-4.7.0 bug where rangefinder data only flows when euler
`attitude` accompanies the quaternion.

[Jolt Physics]: https://github.com/jrouwe/JoltPhysics
