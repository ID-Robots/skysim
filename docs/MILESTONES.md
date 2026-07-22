# Milestones

Build strictly in order. A milestone is done when its acceptance criteria pass and the evidence
(test output / harness log) is pasted in the commit or PR description.

## M0 — Repo boots & constants verified
- CMake configure+build clean on Ubuntu (GCC and Clang), Jolt fetched and linked, `ctest` green in CI.
- ArduPilot pinned: tag recorded in `docs/PROTOCOL.md`; magics, packet layouts, and JSON field
  names verified against `SIM_JSON.{h,cpp}` and the JSON readme; `tests/test_packets.cpp`
  static_asserts sizes and round-trips a captured real servo packet (add a fixture).

## M1 — Protocol layer: single vehicle, canned physics
- Sim binds 9002, decodes servo packets, replies with a kinematic "sits on ground" state.
- Real `arducopter --model json` runs against it: clock slaved (no timestamp warnings spam),
  EKF origin set, heartbeat stable for 60 s, `frame_count` gap/reset handling unit-tested.

## M2 — Single quad flies (Jolt world, flat ground)
- X-quad model + ground contact. Conformance harness passes: arm → GUIDED takeoff 10 m ±0.5 m →
  4-waypoint square → RTL → land → disarm, no EKF errors/failsafes in the log.
- Landing produces no accel spikes beyond clamp; hover throttle within ~±10% of stock SITL.

## M3 — Environment & sensors
- Wind (steady + seeded gusts) affects trajectory; `rng_*` rangefinder raycasts emitted;
  determinism test: strict mode, fixed seed, two runs ⇒ identical trajectory hashes.

## M4 — Multi-vehicle lifecycle
- Control plane: spawn/despawn via REST while others fly; port/instance allocator leak-free
  across 100 spawn/despawn cycles. 10 vehicles fly the conformance mission concurrently.
- Interactive-mode straggler policy demonstrably works: SIGSTOP one SITL → world keeps ticking,
  vehicle freezes + flags, resumes cleanly on SIGCONT. Strict mode: same test aborts with report.

## M5 — Scanned city
- Cooker CLI: mesh → tiles → `.jshape` + index; streamer loads/unloads by vehicle proximity
  (assert memory bound). Quad flies a corridor between buildings; rangefinder tracks walls;
  collision with a building is stable (no tunneling at ≤ 25 m/s — check dt vs speed).

## M6 — Scale & SkyHub
- 50+ vehicles (SITL possibly on second machine), tick p99 < dt, metrics endpoint live.
- SkyHub connects to N simulated vehicles unmodified; a mission planned in SkyHub over the real
  city location replays in-sim with georeference matching the map layer.
