# ArduPilot SITL JSON backend — wire protocol

Pinned ArduPilot: **Copter-4.7.0** (tagged 2026-07-21) · checkout at `$ARDUPILOT_ROOT`.
Authoritative sources (always re-verify against the pinned tree, they override this doc):

- `libraries/SITL/SIM_JSON.h` / `SIM_JSON.cpp`
- `libraries/SITL/examples/JSON/readme.md`
- `libraries/SITL/examples/JSON/pybullet/robot.py` (reference implementation, good for diffing behavior)

All `file:line` citations below are against the Copter-4.7.0 tag.

## Transport & ports

- UDP. **The simulator binds** `9002 + 10 × I` where `I` is the SITL instance number (`-I<n>`)
  (`SIM_OUT_PORT = 9002` and `simulator_port_out += _instance * 10`,
  `libraries/AP_HAL_SITL/SITL_cmdline.cpp:255,437-438`).
- ArduPilot sends servo packets to that port; the sim replies **to the sender's address:port**
  (`readme.md:7` — no port config needed in the sim).
- One JSON object per datagram, newline (`\n`) terminated. Don't fragment across datagrams.
- Related per-instance ports (raw `arducopter` binary): MAVLink SERIAL0 listens on TCP
  `5760 + 10 × I` (`BASE_PORT`, `SITL_cmdline.cpp:247,425-426`).

Launch (per vehicle):

```bash
$ARDUPILOT_ROOT/build/sitl/bin/arducopter \
  --model json:127.0.0.1 -I0 \
  --home 42.1354,24.7453,164,0 \
  --defaults $ARDUPILOT_ROOT/Tools/autotest/default_params/copter.parm
# or during bring-up: sim_vehicle.py -v ArduCopter -f json:127.0.0.1 --console --map
```

## Input: servo packet (ArduPilot → sim, binary, little-endian, packed)

Verified against `libraries/SITL/SIM_JSON.h:45-57` at Copter-4.7.0:

```c
struct servo_packet_16 {       // default (≤16 servo channels)
    uint16_t magic;            // 18458
    uint16_t frame_rate;       // physics rate ArduPilot wants, Hz (SIM_RATE_HZ)
    uint32_t frame_count;      // increments per *parsed* input frame (see below)
    uint16_t pwm[16];          // ~1000..2000 µs
};                             // sizeof == 40

struct servo_packet_32 {       // sent iff SERVO_32_ENABLE=1 (readme.md:23-33)
    uint16_t magic;            // 29569
    uint16_t frame_rate;
    uint32_t frame_count;
    uint16_t pwm[32];
};                             // sizeof == 72
```

- Which variant is sent is decided per-packet by `SRV_Channels::have_32_channels()`
  (`SIM_JSON.cpp:109`); the sim must accept both and dispatch on datagram size + magic.
- `frame_count` semantics: detect duplicates (don't re-step physics), drops (log, step anyway),
  and **reset to a lower value ⇒ ArduPilot rebooted** → reset that vehicle's protocol state
  (not its pose). Readme: "count will be reset when SITL is re-started" (`readme.md:37`).
- **Duplicates are normal at connection start, and the sim must still reply to them.**
  ArduPilot increments `frame_counter` only after successfully parsing a sim reply
  (`SIM_JSON.cpp:521`), and its line-based framing needs *two* buffered newline-terminated
  replies before the first parse succeeds (`SIM_JSON.cpp:339-347` — it parses the most recent
  complete line, delimited by the previous line's terminator). So the same `frame_count`
  arrives repeatedly during the handshake; a sim that stays silent on duplicates deadlocks it.
- If ArduPilot gets no reply for ~1 s it re-sends the servo packet without incrementing the
  counter (`SIM_JSON.cpp:321-331`; the code comment and `readme.md:37` say 10 s, the code says
  `wait_ms > 1000`).
- `frame_rate`: treat as a request; our tick policy (DESIGN.md) decides the actual dt and we may
  set `SIM_RATE_HZ` lower (e.g. 400) at scale via the defaults file. Default is **1200 Hz** on
  SITL builds (`SIM_RATE_HZ_DEFAULT`, `libraries/SITL/SITL.cpp:46-52`). Keep it above the
  vehicle loop rate — `SCHED_LOOP_RATE` defaults to 400 Hz on copter (`readme.md:35`,
  `AP_Scheduler.cpp:44`). Observed at fixture capture (Copter-4.7.0 SITL, stock `copter.parm`,
  no SIM_RATE_HZ override): **1200 Hz**, `frame_count=0`, 16-channel packet, re-sent unchanged
  ~1/s while the sim stayed silent (see `tests/fixtures/servo_frame.bin`).

## Output: state JSON (sim → ArduPilot)

Parsed via the keytable at `SIM_JSON.h:122-165`. Required there (`required == true`):

```json
{
  "timestamp": 12.3456,                      // physics time, seconds, monotonic (double)
  "imu": {
    "gyro": [p, q, r],                       // rad/s, body FRD
    "accel_body": [ax, ay, az]               // m/s², specific force, body FRD
  },
  "velocity": [vn, ve, vd]                   // m/s, NED
}
```

plus, enforced at runtime (`SIM_JSON.cpp:356-360`): **at least one of**

```json
  "attitude": [roll, pitch, yaw]             // rad
  "quaternion": [w, x, y, z]                 // preferred over attitude if both sent
                                             // (SIM_JSON.cpp:430-435)
```

Position: `"position": [north, east, down]` (m, from origin) is **not** marked required in the
keytable, but if absent ArduPilot falls back to a stale `state.position`
(`SIM_JSON.cpp:401-404`) — we always send it. Alternative: send all three of `"latitude"`,
`"longitude"` (deg), `"altitude"` (m AMSL) and ArduPilot derives NED from them, setting home on
the first message (`SIM_JSON.cpp:387-400`).

Remaining optional keys (verified keytable spelling; lat/lon/alt, position, attitude,
quaternion above are optional too):
`rng_1`…`rng_6` (rangefinder, m) · `velocity_wind` `[n,e,d]` (m/s, earth-frame wind vector) ·
`windvane {direction, speed}` (rad clockwise from nose / m/s, apparent) · `airspeed` (m/s) ·
`no_time_sync` (bool: opt out of clock slaving; forces default `AHRS_EKF_TYPE=10`,
`SIM_JSON.cpp:406-416`) · `no_lockstep` (bool: SITL free-runs, non-blocking receive,
`SIM_JSON.cpp:309-319,418-427`) · `rc {rc_1..rc_12}` (µs) · `battery {voltage, current}` (V, A).

### Upstream quirk: rangefinder copy is gated on the euler-attitude bit (Copter-4.7.0)

The loop that copies parsed `rng_*` values into `state.rangefinder_m[]`
(`SIM_JSON.cpp:465-470`) iterates received-bitmask bits 7..12 — stale indices from before
`latitude/longitude/altitude` were added to the keytable (rng_1 is actually bit 10,
`SIM_JSON.h:179`). Net effect: `rangefinder_m[0]` is updated only when the **euler
`attitude` key** (bit 7) is present. A quaternion-only sender's rangefinder data is
silently dropped. skysim therefore always emits `attitude` (derived from the quaternion)
alongside `quaternion`; ArduPilot still uses the quaternion for attitude when both are
present (`SIM_JSON.cpp:430-435`).

### ArduPilot's parser is strstr-based, not a real JSON parser (`SIM_JSON.cpp:177-297`)

Hard interop rules for our emitter:

- Keys are located with `strstr` from the start of the (last complete) line. Substring aliasing
  is real: emit `"velocity"` **before** `"velocity_wind"`, and `rc_1..rc_12` in ascending
  order, or the wrong value is parsed.
- The value is read at `key + strlen(key) + 2`, i.e. it assumes `"key":value`. Numeric parsers
  skip whitespace, but **booleans don't**: `"no_time_sync": true` (space after colon) parses as
  *false* (`SIM_JSON.cpp:281-292`). Emit compact JSON — no spaces — or send `1`/`0`.
- One complete JSON object per datagram, `\n`-terminated, single line. ArduPilot buffers
  datagrams and parses only the most recent complete line (latest-wins on its side too).

### Semantics that break the EKF if you get them wrong

- `accel_body` is the **accelerometer reading** (specific force), not coordinate acceleration:
  at rest on the ground it must read ≈ `[0, 0, -9.81]` (FRD, z down).
- `timestamp` drives ArduPilot's clock (lockstep): `deltat = timestamp - last` advances
  `time_now_us`, and the frame-time adjustment engages only for `0 < deltat < 0.1 s`
  (`SIM_JSON.cpp:499-521`). It must be strictly monotonic per vehicle and advance by exactly
  the dt whose inputs you consumed. Stalled timestamp ⇒ ArduPilot waits. A **backwards**
  timestamp is logged as "Detected physics reset" and treated as `deltat = 0` — it does not
  reboot ArduPilot (`SIM_JSON.cpp:500-504`).
- ArduPilot's own SITL layer synthesizes GPS/baro/mag from this truth state + `--home`, adding
  its own noise. We send clean truth (plus optional small IMU noise); we do NOT emit GPS.
- Contact events must not produce NaN/Inf or multi-thousand-g spikes; clamp and smooth impulses
  (see DESIGN.md ground-contact notes) or the EKF diverges on landing.

## Conformance

`tools/harness/conformance.py` launches N real SITL instances against the sim and asserts:
heartbeat, EKF origin set, arm, GUIDED takeoff to 10 m ± 0.5 m, square mission, RTL, disarm.
Run it after any protocol or vehicle-model change.
