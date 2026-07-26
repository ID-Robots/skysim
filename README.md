<h1 align="center">skysim</h1>

<p align="center">
  <strong>Fly hundreds of drones in one shared physics world — on a single CPU core.</strong><br/>
  A headless multi-vehicle flight simulator for <a href="https://ardupilot.org/">ArduPilot</a> SITL, built on
  <a href="https://github.com/jrouwe/JoltPhysics">Jolt Physics</a>. Gazebo's job, minus Gazebo.
</p>

<p align="center">
  <a href="https://skyhub.ai"><img alt="SkyHub" src="https://img.shields.io/badge/🚁_Built_for-SkyHub-2563eb?style=flat-square" /></a>
  <a href="https://idrobots.com"><img alt="ID Robots" src="https://img.shields.io/badge/By-ID_Robots-0f172a?style=flat-square" /></a>
  <a href="https://github.com/ID-Robots/skysim/actions/workflows/ci.yml"><img alt="CI" src="https://img.shields.io/github/actions/workflow/status/ID-Robots/skysim/ci.yml?branch=main&style=flat-square&label=CI" /></a>
  <img alt="Coverage" src="https://img.shields.io/badge/coverage-85%25%2B_lines-success?style=flat-square" />
</p>

<p align="center">
  <img alt="C++20" src="https://img.shields.io/badge/C%2B%2B20-00599C?style=flat-square&logo=cplusplus&logoColor=white" />
  <img alt="Jolt Physics" src="https://img.shields.io/badge/Jolt_Physics-ff6b35?style=flat-square" />
  <img alt="ArduPilot" src="https://img.shields.io/badge/ArduPilot-Copter_4.7.0-brightgreen?style=flat-square" />
  <img alt="Platform" src="https://img.shields.io/badge/Linux-Ubuntu_22.04_%7C_24.04-E95420?style=flat-square&logo=ubuntu&logoColor=white" />
</p>

<p align="center">
  <img src=".github/assets/skyhub-collision.png" alt="Two simulated drones collide over a 3D city model in the SkyHub dashboard — skysim detects the impact and the collision alert surfaces live in the operator UI" width="920" />
</p>

<p align="center"><sub><em>A real mid-air collision in a shared skysim world, surfaced live in <a href="https://skyhub.ai">SkyHub</a>.</em></sub></p>

---

## What is skysim?

skysim is a **drone swarm simulator**: a headless physics server that many unmodified
`arducopter` processes connect to over UDP, so an entire fleet flies in **one shared world**
where the aircraft can actually see and hit each other — and hit buildings.

It exists because the usual answer, Gazebo, is heavy for this shape of problem. skysim is
scoped to one job: **many quadcopters over a large scanned-city mesh**, with runtime
spawn/despawn, lockstep determinism when you want reproducibility, and wall-clock realtime
when operators are in the loop.

Vehicles are real, unmodified ArduPilot binaries speaking plain MAVLink, so a ground control
station — [SkyHub](https://skyhub.ai), Mission Planner, QGroundControl — connects to the
simulated fleet exactly as it would to real aircraft.

### Why it's built this way

| | |
|---|---|
| 🌍 **One world, many vehicles** | Every drone shares a single Jolt world, so **drone-vs-drone and drone-vs-building collisions are real physics**, not scripted events |
| ⚡ **~200 quads @ 800 Hz** | On a *single* physics core, with ~5× realtime headroom |
| 🎯 **Deterministic when you need it** | Strict lockstep mode: same inputs → byte-identical truth logs. Interactive mode ticks the wall clock and freezes stragglers instead of stalling the fleet |
| 🏙️ **Real city geometry** | Building meshes cooked offline into Jolt shapes, streamed in and out by vehicle proximity under a hard residency cap |
| 🔌 **Unmodified ArduPilot** | No forks, no patches — `arducopter --model json` and plain MAVLink |
| 🧩 **Runtime spawn/despawn** | REST control plane; add and remove aircraft while the world runs |
| 🪶 **Headless & cheap** | No renderer, no ROS, no Gazebo. One binary |

---

## How it works

Each `arducopter` outsources physics over UDP: it sends a servo packet (16 PWM values) to
`9002 + 10·I` and blocks until skysim replies. skysim turns PWM into motor thrust (spin-up
lag, X-quad mixer verified against the pinned ArduPilot source), steps one shared Jolt world
by a fixed dt, and replies with the truth state — IMU specific force, position, velocity,
attitude, optional rangefinder raycasts — as one JSON line whose timestamp **is** ArduPilot's
clock.

```
┌──────────────┐  servo PWM ──▶ ┌─────────────────────────────┐
│ arducopter 0 │ ◀── truth JSON │   skysim                    │
├──────────────┤                │                             │ ◀── REST :8642
│ arducopter 1 │ ◀────────────▶ │   one Jolt world, fixed dt  │     spawn / despawn
├──────────────┤                │   streamed city tiles       │     crash counters
│      ...     │ ◀────────────▶ │                             │     metrics
└──────────────┘                └─────────────────────────────┘
       │ MAVLink
       ▼
  SkyHub / QGroundControl / Mission Planner
```

Read in depth: [`docs/PROTOCOL.md`](docs/PROTOCOL.md) → [`docs/DESIGN.md`](docs/DESIGN.md) →
[`docs/SKYHUB_INTEGRATION.md`](docs/SKYHUB_INTEGRATION.md).

---

## Quick start

**Prerequisites** — Ubuntu 22.04/24.04, CMake ≥ 3.24, Ninja, GCC 12+ or Clang 16+, and an
ArduPilot checkout built for SITL:

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot ~/ardupilot
cd ~/ardupilot && git checkout Copter-4.7.0 && git submodule update --init --recursive
./waf configure --board sitl && ./waf copter
export ARDUPILOT_ROOT=~/ardupilot
```

Also: `pip install pymavlink` (harnesses), `gcovr` (coverage), `trimesh` (real-mesh pre-step).

**Build and test** — the test suite needs no SITL:

```bash
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j
ctest --test-dir build --output-on-failure   # all unit, happy-path, and performance suites
ctest --test-dir build -L unit               # fast C++ correctness tests
ctest --test-dir build -L happy-path         # real binary + lifecycle smoke tests
ctest --test-dir build -L performance        # 800 Hz and reply-path performance budgets
tools/coverage.sh                            # HTML/XML/JSON; gates lines, branches, functions
```

Pull requests into `main` must pass GCC and Clang unit tests, simulator happy paths,
performance budgets, and full `src/` coverage. The stable `CI required` check combines
those jobs so branch protection can block a merge if any one fails.

**Fly two drones over a city:**

```bash
# cook a demo map once
python3 tools/cooker/pretile.py build/demo_map_obj
./build/tile_cooker build/demo_map/tiles build/demo_map_obj/*.obj

./build/skysim \
  --vehicles 2 --base-instance 1 \        # endpoints on udp 9012, 9022
  --time-mode interactive \               # or: strict (CI/replays; barrier + abort)
  --tiles build/demo_map/tiles \          # streamed: --stream-radius / --stream-max
  --rangefinders 2 \                      # rng_1 down, rng_2 right
  --wind 4,1,0 --gust 1.5,2 --seed 42 \   # steady + seeded OU gusts
  --api-port 8642                         # REST control plane

# then, per vehicle:
$ARDUPILOT_ROOT/build/sitl/bin/arducopter --model json:127.0.0.1 -I 1 \
  --home 42.1403890,24.7645490,0,0 \
  --defaults $ARDUPILOT_ROOT/Tools/autotest/default_params/copter.parm,tools/harness/params/skysim.parm
```

### Build a world from real buildings

`tools/cooker/osm_buildings.py` pulls real footprints and heights from
[OpenStreetMap](https://www.openstreetmap.org/) (ODbL) and extrudes them into collision
geometry, georeferenced to a WGS84 anchor so the simulated city lines up with the map your
operators see:

```bash
python3 tools/cooker/osm_buildings.py build/city_obj \
  --lat 42.1403890 --lon 24.7645490 --radius 5000
./build/tile_cooker build/city/tiles --anchor 42.1403890,24.7645490,0 build/city_obj/*.obj
```

That produces ~19 000 buildings over a 5 km radius — a real city your fleet can crash into.

### Docker

```bash
docker build -t skysim .
docker run --rm -p 8642:8642 -p 9002-9202:9002-9202/udp \
  -v $(pwd)/build/city/tiles:/world:ro -e SKYSIM_TILES=/world skysim
```

---

## Control plane (`--api-port`)

| Endpoint | Effect |
|----------|--------|
| `POST /vehicles` `{"launch_process":true?}` | spawn at next free instance (optionally forks arducopter) → `{id, instance, json_port, mavlink_tcp}` |
| `DELETE /vehicles/{id}` | despawn, release instance, kill managed process |
| `GET /vehicles` | per-vehicle: connected, frozen, held_ticks, pos_ned, `midair_collisions`, `building_contacts` |
| `GET /metrics` | tick p50/p99 µs, straggler_events, freezes, resident_tiles |

All mutations execute at tick boundaries; reads come from per-tick snapshots. The SkyHub
gateway polls `GET /vehicles` and, on a collision-counter increase, emits a crash alert to
the dashboard.

## Useful flags

| Flag | What it does |
|---|---|
| `--truth-log out.csv` | Ground truth per tick — judge the physics with this, not the EKF |
| `--record-servo` / `--replay-servo` | Deterministic input tapes for reproducible runs |
| `--time-mode strict` | Lockstep barrier on every vehicle; aborts on a straggler (CI) |
| `--stream-radius` / `--stream-max` | City tile residency around each vehicle |
| `--physics-threads` | Opt a single huge world into Jolt's thread pool (only worth it past ~1000 bodies) |
| `--io-threads` | Fan the reply path across cores (helps past ~48 vehicles) |
| `--canned` | M1 protocol-debug mode |

Live acceptance harnesses (need `ARDUPILOT_ROOT`; use `--base-instance 1`+ if something
already owns the instance-0 SITL ports):

```bash
python3 tools/harness/conformance.py --base-instance 1 [--vehicles 10]   # mission gate
python3 tools/harness/churn.py --sim ./build/skysim                      # spawn/despawn 100×
PYTHONPATH=tools/harness python3 tools/harness/determinism.py --base-instance 1
PYTHONPATH=tools/harness python3 tools/harness/collision_demo.py --base-instance 1
```

## Performance

One world sustains **~200 quads at 800 Hz on a single physics core with ~5× realtime
headroom**; scale past that by sharding worlds across processes. Four optimizations got it
there: a single-threaded physics job system by default (Jolt's thread jitter is a net loss
below ~1000 bodies), an integer fixed-precision JSON formatter (6.3× faster than
`snprintf`), O(1) contact attribution, and a reply path that fans across `--io-threads`
cores above 48 vehicles. Measured tick breakdown in [`docs/DESIGN.md`](docs/DESIGN.md).

The performance suite fails if a 200-vehicle world misses its 1,250 µs tick budget, if the
protocol hot path exceeds 2,000 ns per vehicle, or if the pooled 100-vehicle reply path
misses the same 1,250 µs budget.

## Project status

Milestones M0–M6 are complete: protocol layer, single-quad flight conformance, wind /
rangefinders / determinism, multi-vehicle lifecycle, city tiles & streaming, and scale +
SkyHub integration. CI measures every first-party simulator source under `src/` and gates
at **85% lines, 70% branches, and 95% functions**. Detailed HTML, Cobertura XML, JSON, and
text reports are retained with each Actions run. History and acceptance evidence live in
[`docs/MILESTONES.md`](docs/MILESTONES.md).

Pending human sign-off on the flight model: `k_thrust=7.65e-6`, linear-only drag, and
`SIM_RATE_HZ=800` + `--dt 1/800` for conformance (ArduPilot pre-arm requires a gyro rate
≥ 1.8× the loop rate).

## Repo map

```
src/protocol/   servo packet ABI + state JSON emitter + UDP endpoints (no Jolt)
src/core/       frames.h (NED/FRD <-> Jolt), world.cpp (the ONLY Jolt-aware TU), metrics
src/vehicle/    motor lag + X-quad mixer + instance allocator / process manager
src/terrain/    OBJ -> MeshShape cooker + proximity tile streamer
src/api/        REST control plane (cpp-httplib)
tools/cooker/   pretile.py (demo city) + osm_buildings.py (real city) + tile_cooker CLI
tools/bench/    gated world, protocol, and UDP reply-path performance benchmarks
tools/harness/  conformance / determinism / straggler / churn / collision / corridor
tests/          unit tests + app_smoke.py driving the real binary
```

Notable interop findings baked into [`docs/PROTOCOL.md`](docs/PROTOCOL.md): ArduPilot's
`strstr` JSON parser rules (field ordering, compact booleans), the two-line handshake
(always reply, even to duplicates), and an upstream Copter-4.7.0 bug where rangefinder data
only flows when euler `attitude` accompanies the quaternion.

---

## Built for SkyHub

skysim is the simulation backend for **[SkyHub](https://skyhub.ai)** — a cloud fleet control
system for autonomous drone operations. SkyHub spawns simulated vehicles into one shared
skysim world, and collisions surface live on the operator's map alongside real aircraft
telemetry. See [`docs/SKYHUB_INTEGRATION.md`](docs/SKYHUB_INTEGRATION.md).

Made by **[ID Robots](https://idrobots.com)** — the team behind the Observer drone and the
NexusBox docking station.

[Jolt Physics]: https://github.com/jrouwe/JoltPhysics

## License

skysim is free software released under the **GNU General Public License v3.0 or later**
(GPL-3.0-or-later) — the same license as [ArduPilot], the autopilot it simulates.

See [LICENSE](LICENSE) for the full text.

You may use, study, modify and redistribute skysim. If you distribute a modified
version, or a work derived from it, that work must also be released under the GPL.

**Using skysim with proprietary software:** skysim runs as its own process and speaks
to the outside world only over the network — a REST control plane and UDP servo/state
packets (see `docs/PROTOCOL.md`). Software that merely talks to a skysim instance over
those interfaces is a separate program, not a derivative work, and is unaffected by
this license. Linking skysim's source or objects into another program is a different
matter and does place that program under the GPL.

[ArduPilot]: https://github.com/ArduPilot/ardupilot
