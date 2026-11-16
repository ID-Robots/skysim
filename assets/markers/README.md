# Dock marker image (TASK-2562)

`e_aruco_228_11_100cm.png` is Observer 1's e-ArUco dock marker, exactly as skyhub_core's
generator writes it. The file is not edited. It is compiled into skysim (`CMakeLists.txt`, "the
dock marker's image"), and the pilot camera and the survey stills draw it on the ground at every
vehicle's dock (`src/render/dock_marker.h`, `docs/DOCK_MARKER.md`).

Generated with skyhub_core `utils/generate_e_aruco_marker.py`:

```
python3 utils/generate_e_aruco_marker.py --size 100 --outer-id 228 --inner-id 11 --dpi 100 --border 2.0 \
    --output e_aruco_228_11_100cm.png
```

| | |
|---|---|
| image | 4093 x 4093 px, 8-bit grey, only 0 and 255; sha256 `8be43526…4646fc` |
| dictionary | cv2.aruco `DICT_7X7_250`, both markers |
| outer marker | ID 228, 3937 px black square (100 cm at 100 dpi), from px 78 to 4014 |
| white border | 78 px (2 cm) on each side |
| cell | outer / 9, in whole px: 437 px (11.10 cm) |
| inner marker | ID 11, `int(cell * 0.8)` = 349 px (8.86 cm). It sits on a white square one cell across in the centre cell (row and column 4), 44 px in from that cell's edges, from px 1870 to 2218 |

skysim draws the outer marker at the image's own scale, so its cells are the generator's. It
stretches the inner marker about its own centre to the configured inner size. The default is
0.09 m, the inner marker's size in Observer 1's flight log of 16.11. That is 1.5 % larger than the
image's 8.86 cm and still sits on its white cell. `--dock-marker-size` and `--dock-inner-size` set
the two sizes.

The generator lays the centre cell out in whole pixels (`4 * (3937 // 9)` from the marker's edge),
so the inner marker's centre is 2 px (0.5 mm) up and left of the outer marker's centre. skysim
keeps it there, and `GET /instances/<n>/camera/dock` reports where the inner marker really is.

## Detection with OpenCV's default parameters (a production finding)

`tests/dock_marker_smoke.py` decodes this file with cv2.aruco. Outer ID 228 decodes at every
scale. With OpenCV's default `DetectorParameters`, which is what skyhub_core's aruco_landing uses
(`detector.py`), inner ID 11 drops out at most scales, because of the default
`minMarkerDistanceRate = 0.05`. The inner marker's corners lie 44 px x sqrt(2) = 62 px from the
corners of the white cell it sits on, which is 0.044 of its 1396 px perimeter. When the detector
also takes the white cell's square outline as a candidate, it treats the inner marker as "too
close" to that larger square and drops it. Decoding the inner marker on its own (cropped to its
white cell) works. So does any `minMarkerDistanceRate <= 0.04`. The test records where it drops
out. The image is the generator's output, unchanged.
