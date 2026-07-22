# skysim — CLAUDE.md

Headless multi-vehicle physics server for ArduPilot SITL (JSON backend), built on Jolt Physics.
Replaces Gazebo for SkyHub's use case: many quads over a large scanned-city static mesh,
runtime spawn/despawn, max multi-core CPU throughput, deterministic lockstep option.

## Read these before writing code

- `docs/PROTOCOL.md` — the ArduPilot JSON SITL wire protocol. This is an ABI. Never change struct layout.
- `docs/DESIGN.md` — threading model, time policy, vehicle model, terrain streaming.
- `docs/MILESTONES.md` — build in this order. Each milestone has acceptance criteria; do not start M(n+1) until M(n)'s criteria pass.

## Source of truth

ArduPilot is pinned via `ARDUPILOT_ROOT` (env var pointing at a local checkout, tag written in
`docs/PROTOCOL.md`). Protocol constants (magic numbers, packet layout, JSON field names) MUST be
verified against `$ARDUPILOT_ROOT/libraries/SITL/SIM_JSON.{h,cpp}` and
`$ARDUPILOT_ROOT/libraries/SITL/examples/JSON/readme.md` — not against memory or this repo's docs.
If the docs here disagree with the pinned source, the source wins; fix the docs in the same commit.

## Build & test

```bash
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j
ctest --test-dir build --output-on-failure

# Conformance against a real SITL (requires ARDUPILOT_ROOT and a built arducopter):
python3 tools/harness/conformance.py --binary $ARDUPILOT_ROOT/build/sitl/bin/arducopter
```

## Invariants — violating any of these is a bug

1. **Units**: SI everywhere. Radians, meters, seconds, m/s, rad/s. No degrees outside UI/CLI parsing.
2. **Frames**: world frame is NED anchored at a single ENU/WGS84 origin; body frame is FRD
   (x forward, y right, z down). Jolt's internal Y-up convention is isolated behind
   `core/world.h` conversion helpers — Jolt axes never leak above that layer.
3. **Fixed dt**: physics advances only in fixed steps. No wall-clock reads anywhere in the
   physics path. Wall clock is allowed only in the pacing layer (realtime mode) and logging.
4. **Determinism**: all randomness goes through a seeded PRNG owned by the world. Same seed +
   same inputs + strict time mode ⇒ bit-identical trajectories on the same build.
5. **Single writer**: world state is mutated only inside the tick, on the tick thread / job
   system. I/O threads only fill per-vehicle mailboxes (latest-wins) and read published snapshots.
6. **Protocol structs are packed ABI**: `src/protocol/packets.h` layouts match ArduPilot byte-for-byte.
   static_asserts on sizeof must stay. Little-endian assumed (x86/ARM64 targets).
7. **JSON reply**: one complete JSON object per UDP datagram, newline-terminated, sent to the
   sender's address:port. No allocation in the hot path — snprintf into a fixed buffer.
8. **Never block the world tick on the network.** Stragglers are handled by the time policy
   (see DESIGN.md), not by waiting inside the step.

## Jolt-specific rules

- Include `<Jolt/Jolt.h>` before any other Jolt header. Do not let clang-format or an editor
  reorder Jolt includes.
- Init order at startup: `RegisterDefaultAllocator()` → `Factory::sInstance = new Factory` →
  `RegisterTypes()` → create `TempAllocatorImpl` and `JobSystemThreadPool`
  (threads = hardware_concurrency − 1) → create `PhysicsSystem`.
- Static city geometry = `MeshShape` per tile. Mesh collision is one-sided: the cooker must
  guarantee outward winding. Dynamic bodies (vehicles) are convex primitives only.
- Broad-phase layers: `STATIC` (tiles) and `MOVING` (vehicles). Vehicles collide with both.
- Contact events go through a `ContactListener` that only enqueues (lock-free) — no world
  mutation from callbacks.

## Style

- C++20, `clang-format` (config in repo) before every commit.
- No exceptions in the tick path; return `expected`-style results or assert.
- Naming: `snake_case` functions/vars, `PascalCase` types, `k` prefix for constants.
- Python tools: ruff-clean, type-hinted, stdlib + pymavlink + trimesh only unless discussed.

## Workflow rules for Claude

- Work milestone-by-milestone. After each milestone, run the acceptance test yourself and paste
  the evidence (test output) before moving on.
- When touching `src/protocol/`, re-read `docs/PROTOCOL.md` first and re-verify constants
  against the pinned ArduPilot source.
- Prefer small commits with the milestone ID in the message, e.g. `M2: quad motor mixer + ground contact`.
- If a design decision isn't covered by `docs/DESIGN.md`, propose it in the PR/commit message
  rather than silently inventing it — flight-model fidelity and time policy changes need
  human sign-off (they get tuned against real flight logs).
