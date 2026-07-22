# skysim

Headless, CPU-parallel multi-vehicle physics server for ArduPilot SITL (`--model JSON`),
built on [Jolt Physics]. A Gazebo replacement scoped to one job: many quads/rovers over a
large scanned-city mesh, runtime spawn/despawn, lockstep determinism when you want it,
wall-clock realtime when operators are in the loop. Vehicles expose plain MAVLink, so
SkyHub connects to the fleet exactly as it would to real aircraft.

Read in order: `CLAUDE.md` → `docs/PROTOCOL.md` → `docs/DESIGN.md` → `docs/MILESTONES.md`.

## Prerequisites

- Ubuntu 22.04/24.04, CMake ≥ 3.24, Ninja, GCC 12+ or Clang 16+
- ArduPilot checkout built for SITL, exported as `ARDUPILOT_ROOT` (tag pinned in `docs/PROTOCOL.md`,
  currently **Copter-4.7.0**):

  ```bash
  git clone --recurse-submodules https://github.com/ArduPilot/ardupilot ~/ardupilot
  cd ~/ardupilot && git checkout Copter-4.7.0 && git submodule update --init --recursive
  ./waf configure --board sitl && ./waf copter
  export ARDUPILOT_ROOT=~/ardupilot   # add to your shell profile
  ```

- `pip install pymavlink` (harness), `pip install trimesh` (mesh pre-processing, M5)

## Build & smoke test

```bash
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j && ctest --test-dir build
python3 tools/harness/conformance.py --binary $ARDUPILOT_ROOT/build/sitl/bin/arducopter
```

## Working on this repo with Claude Code (Fable 5)

Start Claude Code at the repo root — it picks up `CLAUDE.md` automatically. Drive it
milestone-by-milestone; don't ask for everything at once. Suggested opening prompts:

1. **M0**: "Do milestone M0 from docs/MILESTONES.md. ARDUPILOT_ROOT is at ~/ardupilot on
   tag <tag>. Verify every constant in docs/PROTOCOL.md and src/protocol/packets.h against
   SIM_JSON.h, fix any discrepancies in both, capture a real servo datagram as a test
   fixture, and get CI green."
2. **M1**: "Implement the protocol layer per docs/PROTOCOL.md and M1's acceptance criteria:
   UDP endpoint, parse_servo_datagram, build_state_json, canned on-ground state. Then run a
   real arducopter against it and show me 60 s of stable heartbeat."
3. Continue with M2… pasting acceptance evidence each time.

Rule of thumb: anything touching flight-model constants or the time policy gets reviewed by a
human against real logs before merging (see CLAUDE.md, "Workflow rules").

[Jolt Physics]: https://github.com/jrouwe/JoltPhysics
