# skysim design

## What this is / is not

A headless, CPU-optimized rigid-body server speaking the ArduPilot SITL JSON protocol.
Not a general robotics simulator: no plugins, no articulated bodies (v1). Visualization is
out-of-process (WS state stream → three.js/Cesium client, later SkyHub itself). The only
in-process drawing is the vehicle cameras (below), and it never runs on the tick thread.

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

- **strict** (CI, replays): barrier — tick when the next input frame of every *connected*
  vehicle has arrived. Deterministic; one stalled SITL stalls the world. Timeout ⇒ abort with
  report. The timeout starts once something has connected: a world still waiting for its first
  SITL is not stalled, but a fleet that has all gone silent is, and aborts rather than hanging.
- **interactive** (SkyHub operators in the loop): the same barrier, paced to the wall clock,
  with an exit from it instead of an abort. A vehicle that missed the deadline stalls its own
  tick; once it has been silent for `--straggler-timeout` (default 2 s) it is **frozen**
  (kinematic hold, flagged in the API) until frames resume, and auto-despawned after a grace
  period. Freezing takes it out of the barrier entirely — the fleet stops both blocking on it
  and waiting out its share of the frame grace — while it stays polled so it can thaw.
  Replies are sent immediately after the tick that consumed each vehicle's input.

  A late vehicle is never stepped on its previous tick's PWM. ArduPilot derives its scheduler
  rate from the timestamp we return (`adjust_frame_time(1.0 / deltat)` in SIM_JSON.cpp), so a
  step it did not contribute to re-rates its control loop and hands it motion it never
  commanded — it crashed every takeoff. Lateness may cost wall-clock pace, never fidelity.

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

## Rendering: pilot camera and survey stills

Two cameras draw the same static world, off the tick thread:

- **Pilot camera** (`--camera-fps`, `RenderService`): a small frame (256x144) per vehicle at a
  fixed rate on its own thread and pool, served as `camera.jpg` / `camera.mjpg`.
- **Survey still** (`GET .../still.jpg`, `StillService`): one large frame on request. The HTTP
  route copies the vehicle out of the per-tick snapshot (position, attitude and sim time from
  the same tick — `VehicleInfo` carries all three) and hands it to the still service, which
  renders on one of `--still-concurrency` slots. Each slot owns a worker pool created at
  startup whose threads run at nice +10, so on a box short of cores the physics tick wins every
  contest for a CPU; the HTTP thread itself only waits. A request that finds every slot taken
  gets `503` + `Retry-After` at once — a queue would hold HTTP workers and sockets for as long
  as the backlog takes. Nothing in the path takes a lock the tick thread holds except the
  snapshot mutex, for the length of a vector copy.

**Scene.** `RasterScene` is built once from the tile directory (all tiles, not the streamed
set) and shared read-only by both cameras. On build it works out what each triangle belongs to:
buildings are connected components through shared corners, keyed by their lowest corner so a
building keeps its colours whatever order tiles load in; faces are coplanar edge-connected sets
(their extent places windows; edges no coplanar neighbour shares are the outline, which places
parapets); and a uniform 2D grid of every shadow-casting triangle, with the tallest point per
cell, carries shadow rays. The physics ground slab is dropped: the ground is the plane NED
z = 0, intersected analytically, so it renders where no tile is loaded and never vanishes
because a 4 km triangle reached behind the camera.

**Rasteriser.** Per band of rows: fill triangles (near-plane clipped) into a depth and
triangle-id buffer, then shade each pixel once at the exact intersection of its ray with the
winning triangle's plane (or the ground plane). Bands are dealt round-robin to the workers and
own their pixels, so the output does not depend on the thread count.

**Appearance.** `WorldTexture` is a pure function of the world position, the building key, the
face, and `--texture-seed`: roads on a 100 m grid offset 50 m (the demo city's streets) with
markings, Voronoi land cover with shrubs, per-building wall and roof materials, windows,
plinths, cornices, parapets and rooftop plant, detail to about 1 cm. Every pattern is
band-limited by the pixel footprint — octaves finer than two pixels fade to their mean, stripes
blend to their coverage — and the footprint is the exact area a pixel covers on the surface
(Jacobian of the ray–plane intersection), not a function of distance alone. On a plane facing
the camera squarely that area is the same for every pixel, so two nadir photos of the same
spot filter it identically and give it byte-identical colour (`tests/test_still.cpp`). Light
is a sky term plus a sun term (sun to the south-west, 55° up) with cast shadows from the grid;
haze starts at half the draw distance for stills, at the camera for the pilot view.

**Geotags.** Pose headers convert NED to lat/lon with ArduPilot's `Location::offset`
(sphere of 6 378 100 m, longitude scaled at the mid latitude; `src/core/geo.h`), from
`--spawn-home`, so a frame's geotag and the autopilot's own GPS agree.

**Costs** (demo city, Ryzen 7 5700X): a 1600x1200 nadir still is about 0.55 s on one core,
0.27 s on two (the default per render) and 0.15–0.2 s on four, plus ~40 ms of JPEG encode.
Under a sustained burst of stills (two rendering, the rest turned away with 503) tick p99 moved
from 12.8 to 15.0 µs and the world kept 800 Hz. The pilot camera's 256x144 frame went from
0.3 ms to about 2.5 ms on its four-thread pool with texture and shadows, which still holds
10 fps for some 40 vehicles.
