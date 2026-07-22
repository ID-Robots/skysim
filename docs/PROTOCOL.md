# ArduPilot SITL JSON backend — wire protocol

Pinned ArduPilot: **<set tag here at M0, e.g. Copter-4.6.x>** · checkout at `$ARDUPILOT_ROOT`.
Authoritative sources (always re-verify against the pinned tree, they override this doc):

- `libraries/SITL/SIM_JSON.h` / `SIM_JSON.cpp`
- `libraries/SITL/examples/JSON/readme.md`
- `libraries/SITL/examples/JSON/pybullet_example.py` (reference implementation, good for diffing behavior)

## Transport & ports

- UDP. **The simulator binds** `9002 + 10 × I` where `I` is the SITL instance number (`-I<n>`).
- ArduPilot sends servo packets to that port; the sim replies **to the sender's address:port**.
- One JSON object per datagram, newline (`\n`) terminated. Don't fragment across datagrams.
- Related per-instance ports (raw `arducopter` binary): MAVLink SERIAL0 listens on TCP `5760 + 10 × I`.

Launch (per vehicle):

```bash
$ARDUPILOT_ROOT/build/sitl/bin/arducopter \
  --model json:127.0.0.1 -I0 \
  --home 42.1354,24.7453,164,0 \
  --defaults $ARDUPILOT_ROOT/Tools/autotest/default_params/copter.parm
# or during bring-up: sim_vehicle.py -v ArduCopter -f json:127.0.0.1 --console --map
```

## Input: servo packet (ArduPilot → sim, binary, little-endian, packed)

```c
struct servo_packet {          // 16-channel variant
    uint16_t magic;            // 18458
    uint16_t frame_rate;       // physics rate ArduPilot wants, Hz (SIM_RATE_HZ; copter default ~1200)
    uint32_t frame_count;      // increments every output frame
    uint16_t pwm[16];          // ~1000..2000 µs
};                             // sizeof == 40
```

- A 32-channel variant exists with a different magic (believed `29569`) — **verify both magics
  and layouts in the pinned `SIM_JSON.h` at M0** and encode them as constants with static_asserts.
- `frame_count` semantics: detect duplicates (ignore), drops (log, step anyway), and **reset to a
  lower value ⇒ ArduPilot rebooted** → reset that vehicle's protocol state (not its pose).
- `frame_rate`: treat as a request; our tick policy (DESIGN.md) decides the actual dt and we may
  set `SIM_RATE_HZ` lower (e.g. 400) at scale via the defaults file.

## Output: state JSON (sim → ArduPilot)

Required every reply:

```json
{
  "timestamp": 12.3456,                      // physics time, seconds, monotonic
  "imu": {
    "gyro": [p, q, r],                       // rad/s, body FRD
    "accel_body": [ax, ay, az]               // m/s², specific force, body FRD
  },
  "position": [north, east, down],           // m, from origin (--home)
  "velocity": [vn, ve, vd],                  // m/s, NED
  "attitude": [roll, pitch, yaw]             // rad  (alternatively "quaternion": [w,x,y,z])
}
```

Optional (verify exact names in pinned readme before emitting): `rng_1`…`rng_6` (rangefinder, m),
`windvane {direction, speed}`, `airspeed` (m/s), `"no_time_sync": 1` to opt out of lockstep.

### Semantics that break the EKF if you get them wrong

- `accel_body` is the **accelerometer reading** (specific force), not coordinate acceleration:
  at rest on the ground it must read ≈ `[0, 0, -9.81]` (FRD, z down).
- `timestamp` drives ArduPilot's clock (lockstep). It must be strictly monotonic per vehicle and
  advance by exactly the dt whose inputs you consumed. Stalled timestamp ⇒ ArduPilot waits.
- ArduPilot's own SITL layer synthesizes GPS/baro/mag from this truth state + `--home`, adding
  its own noise. We send clean truth (plus optional small IMU noise); we do NOT emit GPS.
- Contact events must not produce NaN/Inf or multi-thousand-g spikes; clamp and smooth impulses
  (see DESIGN.md ground-contact notes) or the EKF diverges on landing.

## Conformance

`tools/harness/conformance.py` launches N real SITL instances against the sim and asserts:
heartbeat, EKF origin set, arm, GUIDED takeoff to 10 m ± 0.5 m, square mission, RTL, disarm.
Run it after any protocol or vehicle-model change.
