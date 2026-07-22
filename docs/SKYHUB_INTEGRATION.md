# skysim ↔ SkyHub integration plan (local server)

Goal: when SkyHub creates SITL drones, they join **one shared skysim world**; upload a
mission, fly it, and if vehicles crash (each other or buildings), the crash is **detected
live** and surfaced. Investigated 2026-07-22 against `develop` of skyhub_gateway_service
(`6f1d011`), skyhub_core (`726f120`), skyhub_dashboard (`8e25208`), plus the live local
deployment. All file:line refs below are from those trees.

## How SITLs work today (verified)

- Dashboard "Simulated" drone (`type='sitl'`, `drone.constant.ts:19`) → `POST /drone`
  (`gateway src/routes/drone_routes.py:34`) → `SITLDroneService.save()`
  (`src/service/sitl_drone_service.py:843`): the gateway (docker SDK, docker.sock mounted)
  runs **3 containers per drone** — SITL, Core (MAVROS+rosbridge), Gamepad — with
  **`network_mode="host"`** in local/EC2 mode (`sitl_drone_service.py:410,457,649`).
- The SITL container (image `skyhub-sitl:local`, built from `skyhub_core/sitl`) runs
  supervisord → `scripts/sitl.sh:71-77`:
  `arducopter --speedup $SIM_SPEEDUP -I $SITL_INSTANCE --home $HOME_LOCATION
  --model $MODEL --serial0 tcp:0 --defaults $DEFAULTS`
  with `MODEL` default **`quad`** (built-in physics), overridable via env `SITL_MODEL`
  (`sitl.sh:29`) — the gateway does not currently set it (`sitl_drone_service.py:361-374`).
- Instance/port math already matches skysim exactly: gateway allocates container number N
  (1..101, `sitl_drone_service.py:248`), sets `SITL_INSTANCE = N-1` → MAVLink TCP
  `5760+10·I`; skysim's JSON physics port is `9002+10·I`. **No new port scheme needed.**
- Mission flow: dashboard → gateway `POST /drone/action/start_mission`
  (`drone_control_service.py:703`): push mission via `/mavros/mission/push` → GUIDED →
  arm → takeoff (frontend flips AUTO). All physics-agnostic.
- Crash detection today: **none live**. Only post-flight dataflash log analysis
  (`log_analysis_service.py`, `failsafe_count` on `MissionExecution`). The gateway does
  not subscribe `/mavros/statustext`; the dashboard terminal shows the `LOG` stream.

Because the local SITL trio uses **host networking**, an arducopter started with
`--model json:127.0.0.1` reaches a skysim running on this host with zero network work.
(Prior art exists: an Isaac branch already ran `arducopter --model JSON` against an
external physics server — `skyhub_sitl@a616454 scripts/isaac_sim_startup.sh:34-43`.)

## Implementation plan

### 1. skysim (this repo)
- **[S1] Spawn at a requested instance**: extend `POST /vehicles` to accept
  `{"instance": N}` so the gateway's allocator stays authoritative (today skysim picks
  lowest-free itself). Reject if taken.
- **[S2] systemd unit / run script** for the local server:
  `skysim --vehicles 0 --time-mode interactive --dt 0.00125 --api-port 8642
  [--tiles <city> --rangefinders 1 --wind ...]`. Interactive mode is the right default
  with operators in the loop: a paused/killed SITL freezes instead of stalling the fleet.
- **[S3] (later, M6) crash push**: skysim already exposes `midair_collisions` +
  `static_contacts` per vehicle in `GET /vehicles`; optionally add a webhook/SSE so the
  gateway doesn't have to poll.

### 2. skyhub_core (`sitl/` image)
- **[C1] `scripts/sitl.sh`**: no change needed for the model (env `SITL_MODEL` already
  exists); add one conditional to append `/config/skysim.parm` to `DEFAULTS` (same
  pattern as `skyhub_logging.parm`, `sitl.sh:66-69`).
- **[C2] `config/skysim.parm`** (mounted like the logging parm): `SIM_RATE_HZ 800`
  (ArduPilot's prearm requires gyro rate ≥ 1.8× the 400 Hz copter loop; 800 pairs with
  skysim `--dt 1/800`). Optional: `RNGFND1_TYPE 100` etc. for rangefinder-equipped sims.

### 3. skyhub_gateway_service
- **[G1] Settings**: `SKYSIM_ENABLED`, `SKYSIM_URL` (default `http://127.0.0.1:8642`),
  `SKYSIM_MODEL_IP` (default `127.0.0.1` for host networking).
- **[G2] `SITLDroneService.start_container()`** (`sitl_drone_service.py:361-374`): when
  `SKYSIM_ENABLED`, add `SITL_MODEL=json:$SKYSIM_MODEL_IP` to the env dict, and before
  starting the container call skysim `POST /vehicles {"instance": sitl_instance}`; on
  drone deletion call `DELETE /vehicles/{id}`. Fail drone creation cleanly if skysim is
  unreachable.
- **[G3] Live crash detection** (net-new, two complementary sources):
  - *Physics truth*: a small poller (or [S3] push) on skysim `GET /vehicles`; when a
    drone's `midair_collisions`/`static_contacts` jumps mid-flight, emit a SocketIO
    `crash` event for that drone id and mark the active `MissionExecution` aborted
    (`execution` model already has `error_events`).
  - *Autopilot view*: subscribe `/mavros/statustext/recv` in `mavros_topics.py` and
    forward `Crash:`/`Failsafe`/`EKF` STATUSTEXTs on the existing telemetry SocketIO
    channel (they already reach the dashboard terminal as `LOG`; this makes them a
    typed alert).
- **[G4] Keep the post-flight path untouched** — dataflash log analysis keeps working and
  cross-checks the live detection.

### 4. skyhub_dashboard (optional polish)
- **[D1]** Toast/banner on the new `crash` SocketIO event (PrimeNG `MessageService`, same
  pattern as `vehicle-command.service.ts:97`) + a collision counter on the SITL drone
  card. Nothing else: mission upload/start UI already works unchanged.

## Test plan (this machine, staged)

**T0 — prereqs**: build skysim (`ctest` 11/11), keep the existing gateway compose stack
running. Free instance check: current deployment has `SKYHUB_SITL_1` on I0; either delete
it or start numbering from 2 (skysim [S1] honors the gateway's numbers either way).

1. **T1 — plumbing, no gateway**: start skysim (interactive, api 8642); spawn slot for
   I5 via curl; `docker run --network host -e SITL_MODEL=json:127.0.0.1
   -e SITL_INSTANCE=5 skyhub-sitl:local`; verify heartbeat on `tcp:5810` with pymavlink
   and EKF origin. (This is exactly what `tools/harness/m1_soak.py` proves outside
   docker.)
- **T2 — gateway-driven single drone**: with [G1][G2][C1][C2], create a Simulated drone
  in the dashboard → verify the trio starts, drone goes green, telemetry flows; upload a
  small mission, `start_mission`, watch it fly and land. Compare hover/behavior with a
  `--model quad` drone.
- **T3 — shared world**: create two Simulated drones → `GET /vehicles` on skysim shows
  both in one world; fly both missions concurrently (equivalent of our 10-vehicle
  conformance run, which passes).
- **T4 — crash detection (the end goal)**: upload two missions whose paths cross at the
  same altitude and time window (or reuse `tools/harness/crash_demo.py` logic through
  the gateway API); run both. Expect: skysim `midair_collisions` fires on both drones →
  SocketIO `crash` event → dashboard alert; ArduPilot reacts (EKF variance/failsafe/
  disarm — our live demo measured contact at exactly body-to-body distance, 0.36 m).
  Also verify a building crash on the demo map (`--tiles`) via `static_contacts` during
  flight (ground contacts while landed are normal — gate on "airborne" state).
- **T5 — regression**: post-flight log analysis on a crashed run still reports
  `failsafe_count`/`error_events`; a `--model quad` drone still works with
  `SKYSIM_ENABLED=false`.

## Risks / decisions to sign off

- **Instance authority**: [S1] makes the gateway's numbering win — keeps DB/port logic
  untouched. Agreed?
- **Speedup semantics**: with lockstep, `--speedup` is governed by skysim's pacing
  (interactive = wall clock). Fine for operator use; strict mode stays for CI.
- **Crash-vs-landing disambiguation**: `static_contacts` increments on every touchdown;
  the crash rule should be "contact while mission active + not in LAND/RTL descent" or
  simply rely on `midair_collisions` + ArduPilot failsafe for v1.
- **Fargate/remote-docker later**: awsvpc tasks can't reach a host-local skysim — there
  skysim must run as a 4th container in the task (its `INADDR_ANY` bind + reply-to-sender
  design already supports that). Local server first.
- One skysim world = one geographic origin: all SITLs in a world should share `--home`
  (the gateway currently uses one default home — compatible).

## Test results (2026-07-22, local stack)

Implemented and exercised through the real dashboard UI + gateway API.

**Proven working:**
- Dashboard "Simulated" drone create → gateway reserves the skysim slot
  (`skysim: reserved instance N`), sets `SITL_MODEL=json:127.0.0.1`, starts the SITL/core/
  gamepad trio (host networking). Two drones (sim-alpha, sim-bravo) created via the UI both
  **join one skysim world** — `GET /vehicles` shows both `connected`, spawned 10 m apart,
  physics frames exchanged (positions update). Instance numbering stays gateway-authoritative
  (skysim `POST /vehicles {"instance":N}`).
- Gateway control path drives the SITLs: `guided/arm/takeoff/goto_gps_location` all return 200
  via rosbridge→MAVROS.
- Crash monitor runs and reconciles drone identity from the DB after a gateway restart.
- Independently verified: an arducopter with the **exact container args** + a stable MAVLink
  link reaches EKF origin / GPS against skysim (strict and interactive); skysim reports
  `midair_collisions`; the gateway emits `crash_alert` + LOG telemetry.

**Open issue — SITL boot stability under skysim lockstep (not the integration wiring):**
The container arducopter, driven by skysim's lockstep JSON physics, often gets one physics
frame then stalls at boot, with `New/Closed connection on SERIAL0` churn — it never reaches
EKF. A **bare** arducopter with the same args + a *stable* MAVLink client boots fine, so the
trigger is the container's mavlink-router ↔ SERIAL0 cadence interacting with lockstep timing.
Two contributing factors, both diagnosed:
1. **skysim time policy for a dynamic fleet.** Strict mode's global barrier stalls every
   vehicle when *any* connected SITL is slow/stopped/crash-looping (fatal when the control
   plane starts/stops SITLs independently). Interactive mode's straggler freeze was far too
   aggressive (`hold_ticks` default 3 = ~2.5 ms at 1200 Hz), freezing a booting SITL
   repeatedly. Raising `--hold-ticks` removed the freeze thrash but not the SERIAL0 churn.
2. **Real-time margin.** skysim tick p99 (~1.7 ms) exceeds the 0.833 ms budget for 1200 Hz
   lockstep under this host's load (~11); the `SIM_RATE_HZ 800` overlay (doc [C2], skipped in
   this pass) gives headroom and should be applied.

**Recommended next steps (skysim side):**
- Add a **per-vehicle lockstep** time mode: advance each vehicle's own clock by dt per
  consumed frame on the shared world, with no global barrier and freeze-from-barrier for
  non-responders — the correct model for a dynamic, independently-managed SITL fleet.
- Apply the `SIM_RATE_HZ 800` container overlay ([C2]) and run skysim at `--dt 1/800`.
- Investigate the skyhub_sitl mavlink-router SERIAL0 hold under `--model json` (it is stable
  under `--model quad`; the long-lived quad SITL runs for days).

## Update — SITL boot root cause narrowed (2026-07-22, cont.)

The container arducopter under `--model json` receives the first physics frame
("JSON received") then stalls (~12% CPU = idle-blocked), never reaching EKF/GPS, so no
position telemetry reaches the dashboard map. Systematically ruled OUT as causes:
- skysim time mode (strict AND interactive both fail identically),
- lockstep (added `--no-lockstep`; ArduPilot confirms "lockstep DISABLED" but still stalls),
- host load (fails at load ~5 with a single SITL),
- the strict barrier (fails with one vehicle, alone),
- CPU cgroup limit (4 CPUs, using 12%),
- SERIAL0 "New/Closed connection" churn (NORMAL — the healthy 4-day `--model quad` SITL
  shows the identical pattern),
- a stable external GCS on SERIAL0.

**Established facts:** a *bare* arducopter with the EXACT container args (`--speedup 1
--serial0 tcp:0 --model json:127.0.0.1 --defaults copter.parm`) + an active MAVLink client
reaches EKF origin against skysim (proven repeatedly). The SAME container image with
`--model quad` runs healthy for days. Only `--model json` INSIDE the container stalls.

**Leading remaining hypothesis:** the json backend's boot needs an active MAVLink peer
early; in the bare test the pymavlink GCS connects and streams immediately, whereas in the
container MAVROS (separate core container) is slow to establish its FCU link through
mavlink-router, and the json autopilot stalls in that gap. Next debugging step: confirm
`/mavros/state` `connected=true` on the SITL, and/or have the SITL container hold a local
GCS/stream on SERIAL0 at boot (independent of the core container's MAVROS).

**What is fully working:** the integration plumbing end-to-end via the real dashboard —
Simulated-drone create → gateway reserves skysim slot + `SITL_MODEL=json` → SITL/core/gamepad
trio starts → both drones join one skysim world (`connected`, physics exchanged); gateway
crash monitor live. skysim standalone: bare SITL flies to EKF/GPS, mid-air + building
collisions detected. The gap is only the container arducopter completing EKF boot.
