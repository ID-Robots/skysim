# Observer 1's dock marker (TASK-2562)

skysim draws Observer 1's e-ArUco dock marker flat on the ground at every vehicle's dock, at its
real size. The pilot camera (camera.jpg / camera.mjpg / the H.264 RTSP stream) and the survey
stills both show it. A SITL can then land on it with skyhub_core's unmodified aruco_landing
module, reading the vehicle's RTSP stream like the real camera. skysim also publishes the camera
intrinsics it renders with and the marker's ground truth, so the core side can build its
calibration and a test can measure the landing error.

## The marker

| | |
|---|---|
| family | nested e-ArUco, cv2.aruco `DICT_7X7_250` |
| outer | ID 228, 1.00 m black square (`--dock-marker-size`, `SKYSIM_DOCK_MARKER_SIZE`) |
| inner | ID 11, 0.09 m black square on a white square one outer cell across, in the outer marker's centre cell (`--dock-inner-size`, `SKYSIM_DOCK_INNER_SIZE`) |
| plate | the image's 2 cm white border around the outer marker: 1.0396 m square at 1.00 m |
| image | `assets/markers/e_aruco_228_11_100cm.png`, skyhub_core `utils/generate_e_aruco_marker.py --size 100 --outer-id 228 --inner-id 11 --dpi 100 --border 2.0`, unedited ([README](../assets/markers/README.md)) |
| where | each vehicle's home (where it spawned, where ArduPilot sets home when it arms), yaw 0 (the image upright with north up). It can be moved per vehicle (below) |
| switch | `--dock-markers 0` / `SKYSIM_DOCK_MARKERS=0` draws none |

The sizes come from Observer 1: Yanko (15.11) gave the 1.00 m outer marker, and the flight log of
16.11 gave the 0.09 m inner marker and the 5.0 m outer-to-inner switch distance. The switch
distance is aruco_landing's setting. skysim draws both markers at every distance.

**How it is drawn.** The marker is a decal, not geometry. The rasteriser already intersects the
ground plane analytically per pixel, and a ground pixel that lands on a plate takes the marker's
albedo instead of the ground texture (white 0.80, black 0.03, a matte print), lit and hazed like
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
off screen. A parked vehicle's picture is redrawn when its dock moves and re-sent otherwise.
`tests/dock_marker_smoke.py` holds an idle server (camera on, a vehicle on its dock, nobody
watching) under 25 % of a core (1.0 % measured), and checks that the texture is never built there.

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
  anchor (`--spawn-home` lat,lon). They use ArduPilot's own flat-earth step (`core/geo.h`).
- `corners_ned` lists each black square's corners in the order cv2.aruco reports a marker's
  corners: the image's top-left, top-right, bottom-right and bottom-left. At yaw 0 those are
  north-west, north-east, south-east and south-west.
- The inner marker's centre is 0.5 mm north-west of the outer marker's. The generator lays the
  centre cell out in whole pixels; that is where the inner marker really is.
- `vehicle_pose` is the vehicle, which is the camera, against the outer marker's centre, from the
  same snapshot as `GET /vehicles`. After a landing, `horizontal_error_m` is the landing error.
- `source` is `home` until the dock is moved, then `set`.

`POST /instances/<n>/camera/dock` moves the dock. The body is a JSON object with `lat` and `lon`
(this needs the geo anchor) or `north_m` and `east_m`, and/or `yaw_deg`. Absent fields keep their
value, so a GET's pose can be sent back with one field changed. A body that names no field changes
nothing. It answers like GET. `400` for a body that is not an object, a field that is not a number,
lat/lon mixed with north/east, lat without lon, a dock more than 20 km from the world origin, or a
yaw outside ±360. `DELETE` puts the dock back at the vehicle's home. The dock belongs to the
vehicle: it goes when the vehicle despawns, and the next vehicle at that instance starts at its own
home. Every camera draws every vehicle's dock, as a camera would see them in the real world.
`404` with dock markers off (`--dock-markers 0`, or neither the camera nor the still camera on).

## The proof

`tests/dock_marker_smoke.py` (ctest `dock_marker_smoke`, label happy-path) starts skysim with the
pilot camera at 1920x1080 and HFOV 81 deg, the ZR30 at 1x zoom that aruco_landing assumes. A fake
SITL flies a vehicle in JSON lockstep straight up over its dock with the gimbal output at -90 deg.
At 30, 21.5, 15, 5, 2, 1.2 and 0.7 m the test takes a `still.jpg?gimbal=1` and ten frames of the
H.264 RTSP stream (default 1200 kbit/s). It decodes them with cv2.aruco and runs solvePnP with the
published intrinsics against the ground truth above. It also decodes the committed image itself.

What it holds skysim to, whatever the OpenCV version:

- **Geometry, detector-free.** Each black square's side, measured off the pixels, is within 2 px
  (or 1 %) of what the published intrinsics and ground truth project. The worst seen was 0.9 px in
  a still and 1.9 px (of 934) in an H.264 frame.
- **Lateral (landing) error.** For every marker the detector finds, tvec (the marker's centre
  relative to the camera) across the optical axis is within 10 cm of the truth. The worst seen
  was 1.2 cm, at 1.2 m.
- **Scale.** The marker size implied by the true distance and the solved one is within 5 %. Two
  detector biases get wider bounds, and they are the findings below: 8 % where the quad frames the
  white border's outline, and 12 % for a marker under 30 px (the inner marker at 5 m is 20 px,
  2.2 px a cell), whose side cv2.aruco's integer corners get a pixel or two wrong.
- **Decoding**, with the expectations for the installed OpenCV generation (below).

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
   with the defaults, ID 11 decodes only at 5 m (20 px), and at 2, 1.2 and 0.7 m it is not found.
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
   is underestimated by the same amount, and the lateral error stays under 1.3 cm. On a white surround the same frames give the exact black square (563 px against
   562 px projected at 2 m). Whether the real dock's surround is darker than the print decides
   whether Observer 1 sees this too.
3. **OpenCV 4.5/4.6's `SOLVEPNP_IPPE_SQUARE`**, which `estimatePoseSingleMarkers` uses, returns a
   flipped rotation when the four corners are exactly symmetric about the image centre: a
   perfectly level camera exactly over the marker, heading exactly along it. A fake SITL can hover
   like that and a real flight cannot. Its tvec, the part a landing uses, stays correct
   (`ippe_lateral_err_m` in the report), and the proof solves with `SOLVEPNP_ITERATIVE`.

Regenerate the proof:

```bash
python3 tests/dock_marker_smoke.py build/skysim --frames /tmp/dock_frames --report docs/dock_marker
# the same frames with another OpenCV, for example the core image's:
docker run --rm --network none -v $PWD:/w:ro -v /tmp/dock_frames:/f:ro -v /tmp/out:/out \
  --entrypoint python3 <skyhub-core image> /w/tests/dock_marker_smoke.py --analyse /f --report /out
```
