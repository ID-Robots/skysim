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
- **[S4] Simulated radio field** (done): `PUT /radio` takes a radio spec (carriers, towers,
  coverage holes) and skysim then serves the ground truth — `GET /radio/sample`,
  `GET /radio/grid`, and a `radio` object per vehicle in `GET /vehicles`. Model and API:
  [`docs/RADIO.md`](RADIO.md).
- **[S5] Survey still camera** (done; 3D mapping contract, section 1):
  `GET /instances/<n>/still.jpg` (or `/vehicles/<id>/still.jpg`) renders one frame on demand
  from the vehicle's latest snapshot pose — independent of `SKYSIM_CAMERA_FPS`. Query (all
  optional): `w` 1600 (64..4096), `h` 1200 (64..4096), `hfov` 73.7 (10..120), `pitch` -90
  (-90..0), `yaw` 0 (relative to heading, ±360), `roll_stab` 1 (0 = rigid mount), `gimbal` 0
  (1 = where the vehicle's gimbal points, TASK-2548), `range` 600
  (50..3000), `q` 90 (1..100). Response `image/jpeg` with `X-Skysim-Time-Usec`,
  `X-Skysim-Lat`, `X-Skysim-Lon`, `X-Skysim-Alt-Amsl`, `X-Skysim-Alt-Rel`, `X-Skysim-Yaw`,
  `X-Skysim-Pitch`, `X-Skysim-Roll` (camera axes, degrees) and `X-Skysim-Hfov` (also
  `X-Skysim-Ned`, `-Vehicle`, `-Instance`). If `--spawn-home` did not parse (skysim warns at
  startup) there is no earth position to give: `X-Skysim-Lat`, `-Lon` and `-Alt-Amsl` are left
  out and `X-Skysim-Warning` says why. Errors are JSON: `400` for a bad query, `404` for an
  unknown instance/vehicle or a still camera that is off, `503` + `Retry-After: 1` when
  `SKYSIM_STILL_CONCURRENCY` (default 2) renders are already running, `500` if a render fails
  (out of memory on a huge frame, say). `SKYSIM_TEXTURE_SEED`
  (default 1) picks the world's texture. Meaning of the pose headers: docs/PROTOCOL.md.
- **[S6] Pilot camera as H.264 over RTSP** (done; TASK-2561): each vehicle's camera streams like
  the SIYI ZR30 on Observer 1 (`rtsp://<cam>:8554/main.264`), at
  `rtsp://<skysim>:8554/instances/<n>/main.264` — `<n>` is the instance, as in `camera.mjpg`.
  The SITL core can hand it to the browser / Janus H.264 room as it is, instead of decoding the
  MJPEG feed and encoding it again. Wire details (RTSP methods, SDP, RTP, RTCP):
  [`docs/PROTOCOL.md`](PROTOCOL.md#pilot-camera-video-rtsp--rtp-not-part-of-the-wire-protocol).
  - **Stream**: RTP/H.264 (RFC 6184, `packetization-mode=1`) over RTSP, TCP interleaved
    (`RTP/AVP/TCP`) or UDP unicast. Constrained Baseline — `profile-level-id=42e01f` for sizes up
    to 1280x720 at 30 fps; larger frames raise only the level (`42e028` for 1080p30) — no
    B-frames, x264 `zerolatency` (no lookahead: one picture in, one frame out), SPS and PPS in
    front of every IDR (and in the SDP's `sprop-parameter-sets`), 4:2:0 BT.601 limited range.
    ABR at 1200 kbit/s with a one-second VBV, an IDR every 1 s, picture size and rate from
    `SKYSIM_CAMERA_SIZE` / `SKYSIM_CAMERA_FPS`. RTP timestamps are the moment each picture was
    drawn (90 kHz); RTCP sender reports map them to wall-clock time.
  - **Encoder control** (control plane): `GET /instances/<n>/camera/encoder` →
    `{"bitrate_kbps":1200,"fps":10,"keyframe_interval_s":1,"width":256,"height":144,"max_fps":10,"clients":0}`;
    `POST` the same path with any of `bitrate_kbps` (64..8000), `fps` (1..`max_fps`, the camera
    rate) and `keyframe_interval_s` (0.1..10) → the values applied, after clamping; other fields
    are ignored, so a GET body can be sent back changed. A bitrate or keyframe interval change
    takes effect on the next frame; a new fps restarts the encoder (the next frame is an IDR,
    with new SPS/PPS). `POST /instances/<n>/camera/keyframe` → `{"keyframe":true,"clients":N}`
    makes the next frame an IDR. Settings belong to the vehicle: the next vehicle spawned at the
    instance starts from the defaults. `404` for an unknown instance or with RTSP off, `400`
    for a body that is not a JSON object or a field that is not a number.
  - **Cost**: a stream is encoded only while an RTSP session plays it; with none, nothing asks
    the renderer for that vehicle, which stops drawing it 3 s later (the same demand window as
    MJPEG) and frees its encoder after 30 s. Encoding runs on its own worker threads at
    background priority, never on the physics tick or the render thread: a picture the encoder
    has not taken yet is replaced by the next one (`/metrics` `video.frames_dropped`), and a
    receiver too slow for its stream loses frames up to the next IDR, not the connection.
  - **Flags / env**: `--rtsp-port` / `SKYSIM_RTSP_PORT` (8554; `0` = off), `--rtsp-bind` /
    `SKYSIM_RTSP_BIND` (default: the API bind), `--video-threads` / `SKYSIM_VIDEO_THREADS`
    (default one per four usable CPUs, 1..4). RTSP needs the camera (`SKYSIM_CAMERA_FPS`) and
    the control plane; a port that cannot be bound stops startup. `--camera-mjpeg` /
    `SKYSIM_CAMERA_MJPEG` (default `1`) keeps serving `camera.mjpg` beside it until the core
    reads RTSP; `0` turns both `.mjpg` routes into `404`s that name the RTSP path
    (`camera.jpg` stays). Each MJPEG part now carries `X-Skysim-Time-Usec` (sim time) and
    `X-Skysim-Capture-Unix-Usec` (wall clock when the picture was drawn).
  - `GET /metrics` has a `video` object (`null` when off): `rtsp_port`, `streams` and
    `clients` playing, encoder `threads`, mean `encode_ms`, `frames_encoded`,
    `frames_dropped`, `keyframes`, `bytes_sent`.

- **[S7] Observer 1's dock marker** (done; TASK-2562, [`docs/DOCK_MARKER.md`](DOCK_MARKER.md)):
  every vehicle's dock carries the e-ArUco marker (DICT_7X7_250, outer ID 228 at 1.00 m, inner ID
  11 at 0.09 m, `assets/markers/`) flat on the ground at the vehicle's home, heading north, drawn
  by the pilot camera (MJPEG and RTSP) and the survey stills at its real size. **A vehicle's own
  cameras draw its own dock alone** (TASK-2609): instance N's camera.jpg / camera.mjpg, its
  `main.264` RTSP stream and its still.jpg show instance N's dock and no other instance's, so
  aruco_landing, which takes the first detection of ID 228, cannot lock onto a neighbour's dock
  (every dock has the same IDs, and docks are 5 m apart). Before this fix, a dev SITL in slot 0
  that started 0.08 m above its own dock landed 9.58 m away, on slot 2's.
  `--dock-markers-visible all` / `SKYSIM_DOCK_MARKERS_VISIBLE=all` draws every dock in every camera
  again (default `own`). The ground truth, the intrinsics and the dock routes are unchanged.
  `GET /instances/<n>/camera/intrinsics` → `width`, `height`, `hfov_deg`, `fx`, `fy`, `cx`, `cy`,
  `camera_matrix`, `distortion` (zeros), the RTSP picture size and the gimbal's current pitch
  (`?w=&h=&hfov=` for a still's). `GET /instances/<n>/camera/dock` → the marker's ground truth:
  pose (NED, lat/lon with a geo anchor, yaw), both markers' corners in cv2.aruco's order, and the
  vehicle's offset from it (`horizontal_error_m` is the landing error). `POST` the same path with
  `lat`/`lon` or `north_m`/`east_m` and/or `yaw_deg` moves it, and `DELETE` puts it back home.
  Flags / env: `--dock-markers` / `SKYSIM_DOCK_MARKERS` (default 1), `--dock-marker-size` /
  `SKYSIM_DOCK_MARKER_SIZE` (1.0 m), `--dock-inner-size` / `SKYSIM_DOCK_INNER_SIZE` (0.09 m),
  `--dock-markers-visible` / `SKYSIM_DOCK_MARKERS_VISIBLE` (`own` or `all`, default `own`).
  For aruco_landing ("observer1-sim"): run the camera at 1920x1080 with HFOV 81
  (`SKYSIM_CAMERA_SIZE=1920x1080`, `SKYSIM_CAMERA_FOV=81`), take fx/fy/cx/cy from the intrinsics
  route, and drive the gimbal pitch on servo output 10. Read the findings in DOCK_MARKER.md before
  trusting inner-marker detection with OpenCV 4.5.4's default parameters.

- **[S8] One picture size per vehicle; camera modes change how it is drawn** (done; TASK-2661,
  [`docs/DOCK_MARKER.md`](DOCK_MARKER.md#full-resolution-at-the-dock-task-2661)): a 1920x1080
  picture drawn in full for every vehicle at 30 fps does not fit a task (about 0.8 s of CPU a frame
  before TASK-2661; 0.12-0.2 s now, with the lit-ground cache), and the landing needs it only near
  the dock. So every frame of a vehicle keeps its picture size, and a mode decides how it is drawn.
  - **Picture**: `SKYSIM_CAMERA_SIZE` (the fleet's), or the vehicle's own at spawn. It does not
    change by itself: OpenCV 4.6's FFmpeg capture (and the core image's 4.5) keeps returning the
    last frame of the old size when an RTSP picture changes size, so a size switch would blind a
    landing in progress.
  - **Normal mode**: drawn at `SKYSIM_CAMERA_DRAW_SIZE` (default the picture size) and scaled up to
    the picture, at `SKYSIM_CAMERA_FPS`.
  - **Landing mode**: `SKYSIM_CAMERA_LANDING_SIZE` (draw size, default the picture's),
    `SKYSIM_CAMERA_LANDING_FPS` (default the fleet's), `SKYSIM_CAMERA_LANDING_BITRATE` (kbit/s,
    default the stream's own), `SKYSIM_CAMERA_LANDING_RADIUS` (25 m), `SKYSIM_CAMERA_LANDING_HEIGHT`
    (40 m): drawn while a vehicle is within the radius of its dock (`GET .../camera/dock`), at most
    the height above the ground, and not at rest. Leaving takes 5 m more of either. A hover never
    counts as rest, so a SITL searching for the marker at 28.5 m keeps it.
  - **At rest**: `SKYSIM_CAMERA_REST_FPS` (default 0 = off): a vehicle still (2 cm, 0.5 deg) for 5 s
    on the ground is drawn — re-sent — at this rate. Its stream keeps its nominal frame rate (the
    encoder and SPS do not change); frames just come less often. A vehicle spawned on its dock
    starts at rest.
  - **Per vehicle**: `POST /vehicles {"camera":{...}}` at spawn, and `GET` / `POST` / `DELETE
    /instances/<n>/camera` (also `/vehicles/<id>/camera`) at run time: `width`/`height` (or
    `"size":"WxH"`) the picture; `draw_width`/`draw_height` (or `"draw":"WxH"`), `fps`,
    `bitrate_kbps` the normal mode; `"landing":{...}` its own landing mode; `"landing":false`
    turns it off for the vehicle. Bounds 16..3840 x 16..2160, 1..60 fps, 0 or 64..8000 kbit/s,
    and a draw size keeps the picture's aspect ratio (1 %); out of bounds is `400` and changes
    nothing (and spawns nothing). The settings belong to the vehicle and go when it despawns.
  - **What a consumer sees**: the same endpoints, formats and picture size. Switching between the
    normal and landing modes changes the frame rate (15 vs 30 fps): the stream's next frame is an
    IDR with SPS/PPS for the new rate, same picture size, same session. `GET .../camera/intrinsics`
    is the picture's pinhole always (`fx = width/2 / tan(hfov/2)`), plus `drawn` (`mode`, the draw
    `width` and `height`) and `fps` now. In the normal mode the picture is a 640x360 drawing scaled
    up, so it is soft; in the landing mode it is sharp.
  - **Cost**: [README, "What the camera costs"](../README.md#what-the-camera-costs-task-2661) and
    `tools/bench/camera_cost.py`. `GET /metrics` `camera.landing` counts vehicles in the landing
    mode; `camera.ground_cache_mb` is the lit-ground cache (`SKYSIM_CAMERA_CACHE_MB`, 512).

#### Using the still camera from a simulated survey (core, gateway)

- **core `SurveyCaptureHandler`** (contract section 2): with `SKYSIM_URL` and
  `SKYSIM_INSTANCE` set, fetch `{SKYSIM_URL}/instances/{SKYSIM_INSTANCE}/still.jpg?w=…&h=…&hfov=…&pitch=…`
  per CAMERA_FEEDBACK, retrying a `503` after `Retry-After` a bounded number of times. Do not
  draw on the frame — a banner breaks feature matching; mark it simulated in EXIF and the
  sidecar instead.
- **Which position geotags the frame.** A still is rendered from the vehicle's pose *when the
  request is served*, and its `X-Skysim-*` headers are that pose exactly. A request sent as the
  CAMERA_FEEDBACK arrives differs from the feedback by the distance flown in between (tens of
  centimetres for tens of milliseconds at survey speed); one that waited out a `503` was
  rendered further along the track (about 10 m after a 1 s retry at 10 m/s). Correlate the two before trusting the feedback for the
  geotag: compare the frame's position (`X-Skysim-Lat` / `-Lon` / `-Alt-Amsl`) with the
  feedback's. If they differ by more than the capture tolerance, geotag the simulated frame
  with the pose headers (they are the truth for that picture) or drop the capture, and keep
  the original CAMERA_FEEDBACK values in the sidecar either way. Compare positions, not
  timestamps: `X-Skysim-Time-Usec` is skysim's world clock, while CAMERA_FEEDBACK's
  `time_usec` is GPS epoch time (0 before a GPS time) or, with a feedback pin, the autopilot's
  boot clock (docs/PROTOCOL.md). The feedback position is the EKF's estimate, so allow for its
  error (well under a metre in SITL) in the tolerance.
- **Capacity**: a 1600x1200 frame takes about 0.55 s of one core, ~0.3 s on the two
  background-priority threads a render gets by default (`--still-threads`), so the default two
  slots keep up with two vehicles triggering every 2 s with room to spare. Raise `SKYSIM_STILL_CONCURRENCY` for more
  vehicles on a box with the cores for it; renders run at nice +10 and never slow the tick.
- **gateway SITL camera model**: "skysim survey camera", 1600x1200, hfov 73.7° — a 24 mm
  equivalent (36 mm film width), e.g. a 13.2 mm-wide sensor with an 8.81 mm lens (what
  `tools/survey/survey_fixture.py` writes into EXIF).
- **Reconstruction fixture**: `tools/survey/survey_fixture.py` renders a survey grid of the demo
  city without ArduPilot, with EXIF GPS and an ODM `geo.txt`; see the README.
- **ODM on simulated surveys** (gateway `odm` target, contract section 3): ODM 3.6's default
  incremental SfM reconstructed a nadir grid plus one oblique grid of the demo city to 1-6 cm,
  but on nadir + crosshatch (338 frames) it bootstrapped from two obliques seeing almost only
  flat ground, took the wrong one of a plane homography's two solutions, and diverged (camera
  positions ~100 m off, a scrambled orthophoto) — with or without GPS in the bundle, with fixed
  intrinsics, with 0.2 m GPS accuracy. The demo city is ~84 % flat ground; a scanned city is
  not. Expect a dev mapping run over the demo city to need either a scene with more relief or
  a check of ODM's GPS errors in its report before showing the result (a failed run looked like
  this: GPS errors ~100+ m, average GSD reported as 1 cm). Do not pass camera angles in
  geo.txt for `--sfm-algorithm triangulation`: ODM 3.6 turns them into rotations tens of
  degrees off at this latitude (`opendm/photo.py` compute_opk).

### 2. skyhub_core (`sitl/` image)
- **[C1] `scripts/sitl.sh`**: no change needed for the model (env `SITL_MODEL` already
  exists); add one conditional to append `/config/skysim.parm` to `DEFAULTS` (same
  pattern as `skyhub_logging.parm`, `sitl.sh:66-69`).
- **[C2] `config/skysim.parm`** (mounted like the logging parm): `SIM_RATE_HZ 800`
  (ArduPilot's prearm requires gyro rate ≥ 1.8× the 400 Hz copter loop; 800 pairs with
  skysim `--dt 1/800`). Optional: `RNGFND1_TYPE 100` etc. for rangefinder-equipped sims.

- **[C3] Camera from RTSP** (TASK-2561): pull `rtsp://$SKYSIM_HOST:8554/instances/$SITL_INSTANCE/main.264`
  over TCP (`rtspsrc protocols=tcp latency=0 ! rtph264depay ! h264parse ! ...`, or
  `ffmpeg -rtsp_transport tcp -i ... -c copy`) and forward it without transcoding, exactly as
  for the real camera; set the bitrate the link allows with `POST /instances/<n>/camera/encoder`
  and ask for an IDR with `POST /instances/<n>/camera/keyframe` when a viewer joins. Once no
  image reads `camera.mjpg` any more, deploy skysim with `SKYSIM_CAMERA_MJPEG=0`.
- **[C4] Infrastructure** (terraform): the skysim task must accept **TCP 8554** from the core
  and Janus tasks (security group ingress + container port mapping). TCP interleaved is the
  transport to use between tasks; UDP RTP leaves skysim from an ephemeral port pair, so the
  receiver's security group would have to admit UDP from skysim on its own client ports.

- **[C5] Precision landing at full resolution** (TASK-2661, needs [S8]): nothing in the camera
  input has to change for the picture, which keeps one size — but aruco_landing's calibration for
  the "observer1-sim" profile must be the picture skysim streams. Run skysim with
  `SKYSIM_CAMERA_SIZE=1920x1080` and `SKYSIM_CAMERA_FOV=81` (the ZR30 at 1x) and the profile's
  camera matrix must read fx = fy = 1124.016, cx = 959.5, cy = 539.5, no distortion — exactly
  `GET /instances/<n>/camera/intrinsics`. Better: take fx/fy/cx/cy from that route at start-up
  instead of a constant, so a vehicle spawned with its own picture size is calibrated too. If the
  core reads the stream with OpenCV's `VideoCapture`, never give a vehicle a new picture size
  (`POST .../camera {"size":...}`) while it is being read: OpenCV 4.5/4.6 keeps returning the last
  frame of the old size. The frame rate does change, 30 to 15 fps and back, at the landing zone's
  edge (an IDR with new SPS, same size): OpenCV 4.6 reads straight through it
  (`tools/render_probe/stream_check.py`); a Janus/WebRTC path was not tried.
- **[C6] Infrastructure** (terraform, the dev skysim task): the env of [S8] —
  `SKYSIM_CAMERA_SIZE=1920x1080 SKYSIM_CAMERA_DRAW_SIZE=640x360 SKYSIM_CAMERA_FOV=81
  SKYSIM_CAMERA_FPS=30 SKYSIM_CAMERA_LANDING_FPS=15 SKYSIM_CAMERA_LANDING_BITRATE=4000
  SKYSIM_CAMERA_REST_FPS=2` (and `SKYSIM_CAMERA_THREADS` left at its default, every CPU of the
  task) — and the task size the PR's cost table gives; keep 16 GB (the lit-ground cache is
  512 MB at most).

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
- **[G5] Simulated radio** (optional): when a simulated drone is created with a simulated
  radio spec and `SKYSIM_ENABLED` is set, the gateway may forward that spec with
  `PUT /radio` (wrapped `{"radio": {...}}` or bare) so the field the simulated modems report
  from is also queryable as ground truth in skysim. skysim validates it and answers 400 with
  the offending field on a bad spec; the gateway should treat any failure there as a
  warning, never as a failed drone creation. One skysim world holds one field, so the last
  spec PUT wins.

- **[G6] A camera per simulated drone** (optional, TASK-2661): the fleet's camera settings
  ([S8]) are enough for dock landings. To give one drone another picture size, rate or landing
  mode, add `"camera": {...}` to the `POST /vehicles` body ([G2]) — at spawn, before anything reads
  its stream; a bad one is a `400` and spawns nothing.

### 4. skyhub_dashboard (optional polish)
- **[D1]** Toast/banner on the new `crash` SocketIO event (PrimeNG `MessageService`, same
  pattern as `vehicle-command.service.ts:97`) + a collision counter on the SITL drone
  card. Nothing else: mission upload/start UI already works unchanged.
- **[D2]** Simulated video (TASK-2661): nothing to change. The picture keeps its size; with the
  dev settings of [S8] it is 1920x1080 (a 640x360 drawing scaled up outside the landing zone, so
  it looks soft there) and its stream's frame rate drops to 15 fps while a drone lands.

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

## RESOLVED — full end-to-end crash detection working in the UI (2026-07-22)

The SITL json-boot stall is **solved**. Root cause: ArduPilot's json backend needs a live
MAVLink **GCS peer** (heartbeats + data-stream request) to progress past early boot, but the
core container's MAVROS waits for the autopilot first — a deadlock, so neither the autopilot
nor MAVROS ever comes up. A GCS heartbeat pump on the SITL's GCS fan-out port
(`14551 + instance`) breaks it: the autopilot immediately reaches EKF origin / GPS.

Proven end-to-end through the real dashboard:
1. Two Simulated drones created (dashboard/gateway) → gateway sets `SITL_MODEL=json`,
   reserves skysim slots → both join ONE skysim world (`connected`).
2. GCS pump → both autopilots reach EKF/GPS, arm, take off (GUIDED) to 15 m.
3. Commanded to the same point → **physical mid-air collision** in the shared skysim world.
4. skysim counts `midair_collisions`; the gateway crash monitor emits `crash_alert` +
   LOG telemetry; the **dashboard shows a "Collision detected" toast for each drone AND a
   CRASH DETECTED stream in the terminal** (screenshots captured).

Gateway fixes made this reliable: `_wait_for_rosbridge` now uses `host.docker.internal`
(was `localhost` → always 30s timeout, ~60s/create) and the dashboard subscribes to
`crash_alert`.

**Productionize the GCS pump (last step):** the pump is currently an external helper
(`tools/harness`-style). Move it into the SITL container startup (a tiny background
heartbeat sender on `udpout` to its own MAVLink), OR have the gateway launch one per SITL
at creation. Either makes json SITLs self-boot with no external helper. Also worth fixing
upstream: have MAVROS not gate on the FCU before sending its initial heartbeat.
