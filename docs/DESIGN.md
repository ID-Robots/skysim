# skysim design

## What this is / is not

A headless, CPU-optimized rigid-body server speaking the ArduPilot SITL JSON protocol.
Not a general robotics simulator: no plugins, no rendering in-process, no articulated bodies (v1).
Visualization is out-of-process (WS state stream → three.js/Cesium client, later SkyHub itself).

## Process model

One `skysim` process owns one world (one Jolt `PhysicsSystem`). Vehicles are external
`arducopter` processes (local or remote — protocol is plain UDP). Disjoint operating areas can
run as separate `skysim` processes (sharding) for near-linear scaling.

## Threading

```
[udp rx thread(s)] --latest-wins mailbox per vehicle--> ┐
                                                        │ tick thread (fixed dt):
[control-plane HTTP/WS thread] --command queue--------> ┤  1. drain command queue (spawn/despawn)
                                                        │  2. read mailboxes → per-vehicle inputs
                                                        │  3. parallel jobs: motor/aero forces
                                                        │  4. Jolt PhysicsSystem::Update (job system, N-1 threads)
                                                        │  5. publish state snapshot (seqlock/double buffer)
                                                        └─ 6. per-vehicle JSON replies (may fan out to io pool)
```

- Steps 3–4 are where the cores go: Jolt's island-based solver parallelizes contacts; force
  models are embarrassingly parallel across vehicles.
- Snapshot in step 5 is what the control plane, WS viewer, and logger read — never live state.

## Time policy (the one hard decision)

World advances at fixed `dt` (default 1/400 s; per-deployment). Modes:

- **strict** (CI, replays): barrier — tick when *every* registered vehicle's next input frame
  has arrived. Deterministic; one stalled SITL stalls the world. Timeout ⇒ abort with report.
- **interactive** (SkyHub operators in the loop): tick on schedule. A vehicle that missed the
  deadline gets its last PWM held for up to `k` ticks (default 3); beyond that it is **frozen**
  (kinematic hold, flagged in the API) until frames resume, and auto-despawned after a grace
  period. Replies are sent immediately after the tick that consumed each vehicle's input.

Spawn/despawn only ever happens at a tick boundary (step 1), so mid-step body creation never occurs.

## Vehicle model (v1: X-quad, parameterized)

- Rigid body: convex primitive (box or capsule), mass/inertia from config.
- Per motor: PWM 1000–2000 → normalized u ∈ [0,1] → first-order lag (τ ≈ 20–50 ms) → ω →
  thrust `kT·ω²` along −z body, torque `±kQ·ω²` yaw + arm moments. Params in a per-frame TOML.
- Aero: linear + quadratic body drag; wind = steady + gusts (seeded OU process), added as
  freestream velocity in the force model.
- Ground contact: Jolt contact + slight contact softness; clamp reported `accel_body` spikes
  (e.g. ±16 g) so touchdown doesn't blow up the EKF. Tune against stock-SITL landings.

## Terrain: scanned city

Offline pipeline (`tools/cooker` + Python pre-step):

1. Photogrammetry/LiDAR mesh (ODM / RealityCapture / municipal data) → clean + decimate
   (collision mesh is heavily decimated; keep the pretty mesh separately for the viewer).
2. Tile into ~250 m squares on the chosen ENU origin; record WGS84 anchor (must match SkyHub's
   map layer so missions replay 1:1).
3. C++ cooker loads each tile (OBJ/glTF), builds a Jolt `MeshShape`, `SaveBinaryState` to
   `tiles/<x>_<y>.jshape` + an index JSON (AABBs, origin, CRS).

Runtime: `tile_streamer` keeps tiles resident within radius R of any vehicle AABB, add/remove
static bodies at tick boundaries. Rangefinders/lidar = raycasts against the same BVH.

## Scaling notes

- The usual bottleneck is the fleet of ArduPilot processes, not physics — budget roughly a
  sizeable fraction of a core per SITL. 100 vehicles ⇒ spread SITL across machines, one
  physics host. `SIM_RATE_HZ=400` in the defaults file cuts cost ~3× vs copter's default.
- Metrics endpoint exports: tick time p50/p99, per-stage breakdown, straggler counts,
  per-vehicle frame lag, resident tiles. Watch tick p99 < dt.

### Measured performance (single host, one world, `tools/bench`)

Optimizations landed and where the tick budget went (RelWithDebInfo, x86-64):

1. **Physics job system single-threaded by default.** Jolt's `JobSystemThreadPool` loses to
   `JobSystemSingleThreaded` below ~1000 bodies — dispatch/barrier/wakeup jitter dwarfs the
   per-step work for a fleet of quads. `worker_threads<=0` (the default) picks the
   single-threaded system; `>0` opts back into the pool for huge single worlds. We scale by
   sharding worlds across processes, not threads within one (see above).
2. **Integer JSON formatter.** `build_state_json` was snprintf-bound on `%f` float formatting;
   the hand-rolled fixed-precision `append_fixed` (integer scale + round-half-away) is byte-
   identical to the golden output and cut the reply-build cost by a large multiple. This moved
   the reply stage from ~89% to ~14% of a core at 200 vehicles.
3. **O(1) contact attribution.** Contact-event drain looked up the two vehicles by linear scan
   (O(N·M)); a `BodyID→vehicle` map makes it O(1) per event.
4. **Parallel reply path.** `sendto` is kernel-bound (~2.5 µs remote peer, ~6 µs loopback) and
   does not batch, but it is embarrassingly parallel per vehicle. Above
   `kReplyParallelThreshold` (48) vehicles the build+send fans across `--io-threads` (default 3)
   cores; below it the serial path avoids dispatch overhead.

Result: one world sustains ~200 quads at 800 Hz with ~5× realtime headroom, physics on a single
core. Beyond that, shard. Re-measure with `world_bench` / `proto_bench` before tuning further.

## Control plane (SkyHub integration)

- `POST /vehicles {frame, home, params_file}` → allocates instance id, launches arducopter
  (or registers an external one), creates body → `{id, mavlink_tcp, json_port}`.
- `DELETE /vehicles/{id}`, `GET /vehicles`, `GET /metrics`, `WS /state` (snapshot stream).
- SkyHub connects to each vehicle's MAVLink exactly as it would a real aircraft — the sim is
  invisible to it. This is the digital-twin rehearsal mode.
