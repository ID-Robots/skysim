# Observer 1's dock marker (TASK-2562)

skysim draws Observer 1's e-ArUco dock marker flat on the ground at every vehicle's dock, at its
real size. The pilot camera (camera.jpg / camera.mjpg / the H.264 RTSP stream) and the survey
stills both show it; a vehicle's own cameras show its own dock alone
([below](#which-docks-a-camera-draws-task-2609)). A SITL can then land on it with skyhub_core's unmodified aruco_landing
module, reading the vehicle's RTSP stream like the real camera. skysim also publishes the camera
intrinsics it renders with and the marker's ground truth, so the core side can build its
calibration and a test can measure the landing error.

## The marker

| | |
|---|---|
| family | nested e-ArUco, cv2.aruco `DICT_7X7_250` |
| outer | ID 228, 1.00 m black square (`--dock-marker-size`, `SKYSIM_DOCK_MARKER_SIZE`) |
| inner | ID 11, 0.09 m black square on a white square one outer cell across, in the outer marker's centre cell (`--dock-inner-size`, `SKYSIM_DOCK_INNER_SIZE`; at most 95 % of a cell, so a white ring stays around it; a larger size stops startup) |
| plate | the image's 2 cm white border around the outer marker: 1.0396 m square at 1.00 m |
| image | `assets/markers/e_aruco_228_11_100cm.png`, skyhub_core `utils/generate_e_aruco_marker.py --size 100 --outer-id 228 --inner-id 11 --dpi 100 --border 2.0`, unedited ([README](../assets/markers/README.md)) |
| where | each vehicle's home (where it spawned, where ArduPilot sets home when it arms), yaw 0 (the image upright with north up). It can be moved per vehicle (below) |
| switch | `--dock-markers 0` / `SKYSIM_DOCK_MARKERS=0` draws none |
| visible | `--dock-markers-visible own` / `SKYSIM_DOCK_MARKERS_VISIBLE=own` (the default): a vehicle's own cameras draw its own dock alone; `all`: every vehicle's cameras draw every dock (TASK-2609, [below](#which-docks-a-camera-draws-task-2609)) |

The sizes come from Observer 1: Yanko (15.11) gave the 1.00 m outer marker, and the flight log of
16.11 gave the 0.09 m inner marker and the 5.0 m outer-to-inner switch distance. The switch
distance is aruco_landing's setting. skysim draws both markers at every distance.

**How it is drawn.** The marker is a decal, not geometry. The rasteriser already intersects the
ground plane analytically per pixel, and a ground pixel that lands on a plate takes the marker's
albedo instead of the ground texture. A scanned tile's ground mesh at ground level carries it too,
and a raised face hides it (white 0.80, black 0.03, a matte print), lit and hazed like
the ground under it, building shadows included. The outer marker is drawn at the image's own
scale, so its cells are the generator's. The inner marker is stretched about its own centre to
the configured inner size, by exact area weights. The image's 349 px make 0.0886 m at 1.00 m, and
0.09 m is 1.5 % more. Sampling box-filters each pixel's footprint on the ground, which is what a
sensor pixel integrates. It reads the finest mip level coarser than the footprint, so the 7x7
cells stay crisp from 30 m (4 px a cell at 1920 px, HFOV 81) down to touchdown, where texels are
0.25 mm. The plate's edge is anti-aliased into the ground.

**Cost.** Nothing while no camera draws: the image is decoded and the 4096x4096 mip-mapped
texture (22 MB) is built the first time a frame has a marker in shot, or the first time the ground
truth is asked for. The log then says `skysim: dock marker texture built (...)`. A frame pays a
screen-box test per ground pixel for each marker in shot, and nothing for markers out of range or
off screen. A parked vehicle's picture is redrawn when a dock it shows moves (by default only its
own) and re-sent otherwise.
`tests/dock_marker_smoke.py` holds an idle server (camera on, a vehicle on its dock, nobody
watching) under 25 % of a core (1.3 % measured), and checks that the texture is never built there.

**Gimbal.** Nothing new: the pilot camera and `still.jpg?gimbal=1` follow the pitch ArduPilot
drives on the mount output (TASK-2548). A SITL that points the gimbal at -90 deg for docking looks
straight down at the marker.

## API

Both routes are by instance, like the camera's other settings, and both answer `404` for an
unknown instance.

### `GET /instances/<n>/camera/intrinsics`

The pinhole the pilot camera renders with, in OpenCV's convention: pixel centres at whole numbers,
with the top-left pixel's centre at (0, 0). skysim puts the optical axis through the middle of the
frame, so `cx = width/2 - 0.5` and `cy = height/2 - 0.5`. `fx = fy = (width/2) / tan(hfov/2)`.
There is no distortion.

```json
{"camera":"pilot","width":1920,"height":1080,"hfov_deg":81,"vfov_deg":51.33,
 "fx":1124.016,"fy":1124.016,"cx":959.5,"cy":539.5,
 "camera_matrix":[[1124.016,0,959.5],[0,1124.016,539.5],[0,0,1]],"distortion":[0,0,0,0,0],
 "fps":10,"stream":{"width":1920,"height":1080,"path":"/instances/5/main.264"},
 "gimbal":{"present":true,"pitch_deg":-90,"yaw_deg":0}}
```

`stream` is the H.264 picture, the camera's own cropped to even sizes at the right and bottom so
`cx` and `cy` hold in it. It is `null` with RTSP off. `gimbal` is where the camera looks now: the
vehicle's gimbal as its last servo frame drove it, or `present:false` with the fixed mount's
`SKYSIM_CAMERA_PITCH`. With `?w=&h=&hfov=` (each optional; the still defaults are 1600, 1200 and
73.7) the answer is a survey still's pinhole instead (`"camera":"still"`). `404` with the camera
off (`--camera-fps`), or with the still camera off for the still form. `400` for a bad query.

### `GET /instances/<n>/camera/dock`

The dock's ground truth.

```json
{"instance":5,"vehicle":3,"source":"home",
 "marker":{"family":"e-ArUco","dictionary":"DICT_7X7_250","outer_id":228,"inner_id":11,
           "outer_size_m":1,"inner_size_m":0.09,"cell_size_m":0.111,"plate_size_m":[1.0396,1.0396]},
 "pose":{"north_m":0,"east_m":50,"down_m":0,"yaw_deg":0,"lat":42.14038900,"lon":24.76515...,"alt_amsl_m":0},
 "home":{"north_m":0,"east_m":50},
 "outer":{"centre_ned":[0,50,0],"size_m":1,"corners_ned":[[0.5,49.5,0],[0.5,50.5,0],[-0.5,50.5,0],[-0.5,49.5,0]]},
 "inner":{"centre_ned":[0.000508,49.999492,0],"size_m":0.09,"corners_ned":[...]},
 "vehicle_pose":{"sim_time_s":12.5,"pos_ned":[0.01,50.02,-5.0],"offset_ned":[0.01,0.02,-5.0],
                 "horizontal_error_m":0.0224,"height_above_marker_m":5.0}}
```

- `pose` is the outer marker's centre on the ground and `yaw_deg`, the direction the image's top
  edge faces, clockwise from north. `lat`, `lon` and `alt_amsl_m` are present only with a geo
  anchor (`--spawn-home` lat,lon). They use ArduPilot's own flat-earth step (`core/geo.h`). For a
  dock past a pole, where the latitude clamps at 90 and stops naming the point, `geo` says so in
  their place, so the pose still posts back.
- `corners_ned` lists each black square's corners in the order cv2.aruco reports a marker's
  corners: the image's top-left, top-right, bottom-right and bottom-left. At yaw 0 those are
  north-west, north-east, south-east and south-west.
- The inner marker's centre is 0.5 mm north-west of the outer marker's. The generator lays the
  centre cell out in whole pixels; that is where the inner marker really is.
- `vehicle_pose` is the vehicle, which is the camera, against the outer marker's centre, from the
  same snapshot as `GET /vehicles`. After a landing, `horizontal_error_m` is the landing error.
- `source` is `home` until the dock is moved, then `set`.

`POST /instances/<n>/camera/dock` moves the dock. The body is a JSON object with `lat` and `lon`
(this needs the geo anchor) and/or `north_m` and `east_m`, and/or `yaw_deg`. Absent fields keep
their value, and the change is applied in one step, so concurrent partial changes cannot undo each
other. A GET's whole `pose` can be sent back with one field changed: when lat/lon and
north_m/east_m both come, they must name the same point (within 1 cm), and then north_m/east_m
win. A body that names no field changes nothing. It answers like GET. `400` for a body that is not
an object, a field that is not a number, lat/lon and north/east that disagree, lat without lon, a
dock more than 20 km from the world origin, or a yaw outside ±360. `DELETE` puts the dock back at the vehicle's home. The dock belongs to the
vehicle: it goes when the vehicle despawns, and the next vehicle at that instance starts at its own
home. A vehicle's own cameras draw its own dock alone, by default (next section). `404` with dock
markers off (`--dock-markers 0`, or neither the camera nor the still camera on).

## Which docks a camera draws (TASK-2609)

Every dock carries the same IDs, 228 with 11 inside, and vehicles spawn 5 m apart (`--spacing`).
When every camera drew every dock, a vehicle's camera showed several identical markers at once:
from 30 m up a 640x360 HFOV 78 pilot camera spans 48.6 m of ground across, and a 1920x1080 HFOV 81
one 51.2 m. skyhub_core's aruco_landing takes the first detection with its ID, and which one that
is changed from frame to frame. On dev (Mike, 16.11) a SITL in slot 0 that started 0.08 m above
its own dock flew sideways and landed 9.58 m away, on slot 2's dock.

So a vehicle's own cameras draw that vehicle's dock alone. This covers instance N's pilot camera
(camera.jpg, camera.mjpg and the `main.264` RTSP stream, which are the same pictures) and its
survey stills (`still.jpg`, any size, lens or attitude): they draw instance N's dock and no other
instance's. `--dock-markers-visible all` (`SKYSIM_DOCK_MARKERS_VISIBLE=all`) gives back the old
behaviour, every dock in every camera. `own` is the default, and the startup log names the
setting. Nothing else changes: the marker, its sizes, the intrinsics, `POST`/`DELETE
.../camera/dock` and the ground truth `GET` (`pose`, `vehicle_pose.horizontal_error_m`) are as above.
Every vehicle still has its dock, and the dock routes still move each one.

In the code, `DockBoard::seen_by(vehicle)` is what a vehicle's cameras draw (`render/dock_marker.h`).
It is a list of one under `own` and every dock under `all`. `DockBoard::markers()` stays every dock,
for a view that is not a vehicle's own camera. skysim serves no such view today: every picture route
is a vehicle's camera, and `tools/survey/survey_still` draws no docks. A vehicle's list keeps its
pointer while other vehicles' docks arrive, move or go, so under `own` a parked vehicle's picture is
re-sent, not redrawn, when a neighbour's dock moves.

**The proof.** `tests/dock_visibility_smoke.py` (ctest `dock_visibility_smoke`, label happy-path)
uses real renders throughout. It starts skysim with the pilot camera at 1920x1080 HFOV 81 and RTSP
on, and no visibility flag (so the default is what is tested). Slot 0 is flown up over its dock by
dock_marker_smoke's fake SITL, with the gimbal at -90 deg. Slot 1 is parked on its dock 5 m east.
At each height a slot 2 is spawned on its dock 10 m east, captured with, and despawned. At 30, 10 and
5 m, with two vehicles and then three, it takes slot 0's camera.mjpg (a part drawn at that hover),
ten RTSP frames, a `still.jpg?gimbal=1` at the pilot camera's size and lens, and from 10 and 5 m a
still at 640x360 HFOV 78 (the dev camera; from 30 m the marker is 13 px there, too small to decode).
It decodes every picture with cv2.aruco (`DICT_7X7_250`, default `DetectorParameters`, the detector
dock_marker_smoke builds). Each detection belongs to the dock whose ground truth projects nearest
the detected quad's centre, and for slot 0's dock solvePnP's lateral error is the landing error. It
then runs the same flight to 30 and 10 m with `--dock-markers-visible all`. CI's coverage build
renders at -O0, so there it runs `--quick`: `own` and `all` at 30 m only, two RTSP frames each.

- `own`: each still and camera.mjpg picture, and 9 RTSP frames of 10 (H.264 blurs a cell out of a
  frame now and then, as in dock_marker_smoke), finds exactly one 228, slot 0's, with the lateral
  error within 10 cm (dock_marker_smoke's bound). No picture and no RTSP frame finds a second one,
  or any other dock.
- `all`: each still and camera.mjpg picture, and 9 RTSP frames of 10, finds more than one 228. Each
  sits on its own dock, its centre within 3 px of that dock's projected centre. So the test can
  see the difference.

The run for the PR, OpenCV 4.6.0 (Ubuntu 24.04 apt, what CI's ctest has). Each cell gives how many
228s were found (RTSP: frames x count), then slot 0's worst lateral (landing) error, then the worst
distance between a detected centre and its dock's projected centre. Values are rounded, so 0.0 cm
is under 0.5 mm:

| mode | height | vehicles | still 1920x1080 | still 640x360 | camera.mjpg | RTSP frames |
|---|---|---|---|---|---|---|
| own | 30 m | 2 | 1; 0.0 cm; 0.0 px | - | 1; 0.0 cm; 0.0 px | 10x1; 0.8 cm; 0.25 px |
| own | 30 m | 3 | 1; 0.0 cm; 0.0 px | - | 1; 0.0 cm; 0.0 px | 10x1; 2.1 cm; 0.75 px |
| own | 10 m | 2 | 1; 0.0 cm; 0.0 px | 1; 0.0 cm; 0.0 px | 1; 0.0 cm; 0.0 px | 10x1; 0.4 cm; 0.25 px |
| own | 10 m | 3 | 1; 0.0 cm; 0.0 px | 1; 0.0 cm; 0.0 px | 1; 0.0 cm; 0.0 px | 10x1; 0.4 cm; 0.25 px |
| own | 5 m | 2 | 1; 0.2 cm; 0.35 px | 1; 0.0 cm; 0.0 px | 1; 0.6 cm; 0.25 px | 10x1; 1.0 cm; 1.46 px |
| own | 5 m | 3 | 1; 0.8 cm; 0.79 px | 1; 0.0 cm; 0.0 px | 1; 0.5 cm; 0.25 px | 10x1; 0.9 cm; 1.41 px |
| all | 30 m | 2 | 2; 0.0 cm; 0.14 px | - | 2; 0.0 cm; 0.15 px | 10x2; 1.0 cm; 0.42 px |
| all | 30 m | 3 | 3; 0.0 cm; 0.18 px | - | 3; 0.0 cm; 0.18 px | 10x3; 1.0 cm; 0.35 px |
| all | 10 m | 2 | 2; 0.0 cm; 0.11 px | 2; 0.0 cm; 0.03 px | 2; 0.0 cm; 0.21 px | 10x2; 0.4 cm; 0.26 px |
| all | 10 m | 3 | 2; 0.0 cm; 0.01 px | 2; 0.0 cm; 0.08 px | 2; 0.0 cm; 0.49 px | 10x2; 0.4 cm; 0.51 px |

With `all`, from 10 m the third dock, 10 m east, is out of frame: 2 of 3 is the geometry. The same
pictures re-checked with other OpenCV versions:
[4.5.4](dock_marker/visibility-opencv-4.5.4.md) (Ubuntu 22.04 apt, the SkyHub core image's) gave
the same table. [4.10.0](dock_marker/visibility-opencv-4.10.0.md) (pip, `ArucoDetector`) gave the
same table except at 5 m, where the 1920x1080 pictures decode the inner 11 instead of the outer 228
(as in the proof below). There the test asks for exactly one of the two, and it is slot 0's.
`--analyse DIR` re-checks a capture with another OpenCV; CI re-checks its frames with Ubuntu
22.04's 4.5.4.

## Full resolution at the dock (TASK-2661)

On dev (18.11) every SITL's aruco_landing took the RTL over at 30 m within 15 m of home, searched
20 s and held at about 28.5 m: the pilot camera ran at 640x360, where a cell of the outer marker is
1.5 px from 30 m. The real Observer camera is 1920x1080. Pixels per cell of the 1.00 m outer
marker (9 cells across with its border, 0.111 m), straight down, `cell * fx / height`:

| camera | fx | 30 m | 20 m | 10 m | 3 m |
|---|---|---|---|---|---|
| 640x360, HFOV 78 (dev until now) | 395.1 | 1.46 | 2.20 | 4.39 | 14.64 |
| 1280x720, HFOV 81 | 749.3 | 2.78 | 4.16 | 8.33 | 27.75 |
| 1920x1080, HFOV 81 (Observer 1, the landing camera) | 1124.0 | 4.16 | 6.24 | 12.49 | 41.63 |

**Probe pictures.** `tools/render_probe/aruco_check.py` renders the view straight down onto the dock
from each height with `tools/render_probe --dock` (the rasteriser and the marker exactly as the
server draws them, ground cache on), saves the pictures and runs cv2.aruco on them (`DICT_7X7_250`,
default `DetectorParameters`, the observer1-sim dictionary as this repo's proofs use it). OpenCV
4.6.0, bench city, marker centred and 10 m off-centre (east, so still in the picture):

| camera | 30 m | 30 m, 10 m off | 20 m | 20 m, 10 m off | 10 m | 3 m |
|---|---|---|---|---|---|---|
| 1920x1080 HFOV 81 | 228 (37.0 px of 37.5) | 228 | 228 (55.0 / 56.2) | 228 | 228 (111.0 / 112.4) | 228 (404 / 375) |
| 1280x720 HFOV 81 | 228 | 228 | 228 | 228 | 228 | 228 and 11 |
| 640x360 HFOV 78 | 228 (12.0 / 13.2) | no detect | 228 | 228 | 228 | 228 and 11 |

The 1920x1080 pictures (JPEG copies of the lossless pictures the detector read) and the whole run
are in [docs/dock_marker/](dock_marker/) (`probe_1920x1080_*.jpg`,
[probe-opencv-4.6.0.md](dock_marker/probe-opencv-4.6.0.md)). To make them again:

```bash
python3 tools/render_probe/aruco_check.py --probe build/render_probe --tiles build/bench_city/tiles \
    --out /tmp/probe --report docs/dock_marker/probe-opencv-4.6.0.md
# the committed JPEGs are those PNGs at quality 90:
python3 -c "import cv2; [cv2.imwrite(f'docs/dock_marker/probe_1920x1080_{h}m.jpg', \
    cv2.imread(f'/tmp/probe/dock_1920x1080_hfov81_{h}m.png'), [cv2.IMWRITE_JPEG_QUALITY, 90]) for h in (30, 20, 10, 3)]"
```

(The bench city is `python3 tools/cooker/pretile.py build/bench_city_obj --extent 1000` cooked with
`tile_cooker`.)

So 1920x1080 is enough at 30 m, with the marker under the camera or 10 m to the side (the two
placements tested): nothing about the marker
needed changing in the renderer (it is drawn from its own mip-mapped texture, box-filtered per
pixel, untouched by the ground cache; the same table comes out with the cache off). 640x360
decodes a centred, lossless picture at 1.5 px a cell and nothing else; what a consumer gets is an
H.264 stream (below). The 3 m side of 404 px against 375 is finding 2 above (the quad framing the
white border on darker ground); ID 11 is finding 1 (OpenCV 4.5/4.6's `minMarkerDistanceRate`).

**Through the stream.** What aruco_landing gets is H.264 over RTSP, and
`tools/render_probe/stream_check.py` reads it the way a consumer would: one OpenCV `VideoCapture`
(FFmpeg backend, TCP) open from the ground up while a stand-in SITL climbs over the dock with the
gimbal at -90 deg and holds at each height for 3 s. OpenCV 4.6.0, bench city, frames with ID 228:

| skysim camera | dock | 30 m | 20 m | 10 m | 3 m | picture sizes decoded |
|---|---|---|---|---|---|---|
| dev until now: `640x360` @ 30, HFOV 78, 1200 kbit/s | under | 89 of 89 | 90 of 90 | 90 of 90 | 90 of 90 | 640x360 |
| the same | 10 m east | **0 of 90** | 90 of 90 | out of shot | out of shot | 640x360 |
| landing mode: `1920x1080` picture, drawn at 640x360 @ 30 normally and full size @ 15, 4000 kbit/s, near the dock | under | 45 of 45 | 45 of 45 | 45 of 45 | 30 of 45 | 1920x1080 only |
| the same | 10 m east | 45 of 45 | 45 of 45 | out of shot | out of shot | 1920x1080 only |

So 640x360 is marginal, not blind: straight over the marker the stream decodes, and with the dock
10 m to the side from 30 m — a landing that takes over within 15 m of home — it does not. At
1920x1080 it does. At 3 m the full-size picture is perfect (the marker is 375 px) and OpenCV
4.6's legacy detector misses a third of the frames, as it does in the stills (findings 1 and 2
below). The first version of the landing camera switched the stream's picture size; OpenCV's
FFmpeg capture then kept returning the last 640x360 frame (`Slice parameters 0, 1088 are invalid`)
and found nothing. That is why a vehicle's picture keeps one size and a mode only changes how it
is drawn (README; `docs/SKYHUB_INTEGRATION.md` [S8], [C5]).

## The proof

`tests/dock_marker_smoke.py` (ctest `dock_marker_smoke`, label happy-path) starts skysim with the
pilot camera at 1920x1080 and HFOV 81 deg, the ZR30 at 1x zoom that aruco_landing assumes. A fake
SITL flies a vehicle in JSON lockstep straight up over its dock with the gimbal output at -90 deg.
At 30, 21.5, 15, 5, 2, 1.2 and 0.7 m the test takes a `still.jpg?gimbal=1` and ten frames of the
H.264 RTSP stream (default 1200 kbit/s). It decodes them with cv2.aruco and runs solvePnP with the
published intrinsics against the ground truth above. It also decodes the committed image itself.

What it holds skysim to, whatever the OpenCV version:

- **Geometry, detector-free.** Each black square's side, measured off the pixels, is within 2 px
  (or 1 %) of what the published intrinsics and ground truth project. The worst seen was 1.0 px in
  a still and 1.9 px (of 934) in an H.264 frame.
- **Lateral (landing) error.** For every marker the detector finds, tvec (the marker's centre
  relative to the camera) across the optical axis is within 10 cm of the truth. The worst seen
  over the runs was 1.6 cm, at 21.5 m.
- **Scale.** The marker size implied by the true distance and the solved one is within 5 %. Two
  detector biases get wider bounds, and they are the findings below: 8 % where the quad frames the
  white border's outline, and 12 % for a marker under 30 px (the inner marker at 5 m is 20 px,
  2.2 px a cell), whose side cv2.aruco's integer corners get a pixel or two wrong.
- **Decoding**, with the expectations for the installed OpenCV generation (below): in the still
  and in 9 RTSP frames of 10 (8 for a marker under 30 px). H.264 at its default 1200 kbit/s for
  1080p blurs a cell out of a frame now and then (once in CI, the outer marker at 21.5 m), and the
  report lists which frames missed. A marker under 30 px, the inner at 5 m with 2.2 px a cell, is
  at the stream's limit and needs the still and at least one RTSP frame: 7 to 10 of 10 locally, 1
  of 3 in CI's -O0 coverage run. At 4000 kbit/s it decoded in 10 of 10
  ([proof-opencv-4.6.0-4000kbps.md](dock_marker/proof-opencv-4.6.0-4000kbps.md),
  `--bitrate 4000`), so raise the bitrate for a 1080p stream if the link allows. The lateral and
  scale bounds hold for each detection on its own, and `--analyse` refuses a capture that is
  missing a height or a frame.

The run used for the PR is in [docs/dock_marker/](dock_marker/). The main proof is
[proof-opencv-4.5.4.md](dock_marker/proof-opencv-4.5.4.md): OpenCV 4.5.4, read inside the
skyhub-core 0.2.0 image (`c82109f`, Ubuntu 22.04 apt `python3-opencv`, legacy
`cv2.aruco.detectMarkers`). Comparison runs on the same frames:
[4.6.0](dock_marker/proof-opencv-4.6.0.md) (Ubuntu 24.04 apt, what CI's ctest has) and
[4.10.0](dock_marker/proof-opencv-4.10.0.md) (pip, `ArucoDetector`). CI runs the ctest under 4.6
and then re-analyses the same frames with 4.5.4 from apt on an Ubuntu 22.04 container
(`--analyse`).

### Findings (production, not skysim)

1. **Inner ID 11 drops out with OpenCV's default `minMarkerDistanceRate = 0.05`, in the legacy
   detector (4.5.4 and 4.6).** The generator puts a 10 % white margin between the inner marker and
   its white cell, so the inner marker's corners lie 0.044 x its perimeter from the cell's corners.
   The detector treats the two squares as "too close" and keeps the larger one, the white cell,
   which then fails to decode. On the committed image, ID 11 is lost at full resolution and at
   2048 and 1024 px, and decodes again once the inner marker is under about 60 px. In the flight,
   with the defaults, ID 11 decodes only at 5 m (20 px: in the still, and in 7 to 10 of 10 RTSP
   frames over the runs), and at 2, 1.2 and 0.7 m it is not found.
   At 0.7 m the outer marker no longer fits in the frame, so nothing decodes at all. With
   `minMarkerDistanceRate <= 0.04`, ID 11 decodes in every frame from 5 m to 0.7 m. OpenCV 4.7+
   (`ArucoDetector`, a new candidate filter) keeps the inner marker with the defaults, from 5 m
   down, and then drops the outer marker.
   **This disagrees with Observer 1's flight log** (switch at 5 m, inner marker decodable from 5 m
   to 1.2 m). There are two explanations, and I have not picked one: Observer 1 runs a different
   cv2 than the core image's 4.5.4 (4.7+ behaves exactly like the log), or aruco_landing's
   detector parameters on the drone differ from the defaults (any `minMarkerDistanceRate <= 0.04`
   decodes the inner marker). The frames are in [docs/dock_marker/](dock_marker/). Optical blur is
   not the reason: blurring the stills by sigma 0.5 to 3 px did not bring ID 11 back under 4.6.
2. **The outer marker's quad can be the white border's outline.** The generator's border is only
   2 cm, and on ground darker than the print the detector also finds the border's outline as a
   candidate that decodes to 228. The "too close" filter keeps the larger one, so the corners land
   on the border's outside and the marker looks 5 to 7 % larger than it is, at 15 to 1.2 m with the
   legacy detector (in the stills; in H.264 frames at 15 m it framed the black square). The distance
   is underestimated by the same amount, and the lateral error stays under 1.6 cm. On a white surround the same frames give the exact black square (563 px against
   562 px projected at 2 m). Whether the real dock's surround is darker than the print decides
   whether Observer 1 sees this too.
3. **OpenCV 4.5/4.6's `SOLVEPNP_IPPE_SQUARE`**, which `estimatePoseSingleMarkers` uses, returns a
   flipped rotation when the four corners are exactly symmetric about the image centre: a
   perfectly level camera exactly over the marker, heading exactly along it. A fake SITL can hover
   like that and a real flight cannot. Its tvec, the part a landing uses, stays correct
   (`ippe_lateral_err_m` in the report), and the proof solves with `SOLVEPNP_ITERATIVE`.
4. **From 5 m, legacy cv2.aruco misses 228 in some stills by a twentieth of a millimetre.** This is
   the white border's outline from finding 2. On this dock's ground (gravel, with centimetre
   speckle), that outline is the quad the detector decodes. Where the speckle puts its corners,
   it decodes with 0-1 bit errors or with 5, and with the default correction it allows 4. On a
   1920x1080 HFOV 81 still straight down, 228 is missed at camera heights of 5.0036, 5.0039,
   5.0044 and 5.0049 m, and found in every 0.05 mm step from 4.9985 to 5.0020 m. The same pixels
   come out of the renderers of TASK-2562 and of TASK-2661. A uniform surround decides it one way
   for good: a light pad around the plate (albedo 0.6 or 0.8) decodes at every height in
   between, a dark one (0.3) at none. So this is finding 2 again, and on Observer 1 it depends on
   the real dock's surface.
   The proof settled its hover to 1 cm, which left the vehicle closing in at about a millimetre
   a second. Its 5 m still landed at 5.0048-5.0049 m depending on tens of milliseconds of
   scheduling, and failed now and then (TASK-2661 CI). It now settles to 1 mm and 2 mm/s
   (`FakeSitl.hold`), so the pictures are taken at the height named.

Regenerate the proof:

```bash
python3 tests/dock_marker_smoke.py build/skysim --frames /tmp/dock_frames --report docs/dock_marker
# the same frames with another OpenCV, for example the core image's:
docker run --rm --network none -v $PWD:/w:ro -v /tmp/dock_frames:/f:ro -v /tmp/out:/out \
  --entrypoint python3 <skyhub-core image> /w/tests/dock_marker_smoke.py --analyse /f --report /out
```
