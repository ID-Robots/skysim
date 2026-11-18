#!/usr/bin/env python3
"""Can aruco_landing see the dock from here? Probe pictures through cv2.aruco (TASK-2661).

Renders the pilot camera's view straight down onto Observer 1's dock marker with
tools/render_probe (the rasteriser and the marker exactly as the server draws them) from each
height, at each camera size, saves the pictures, and runs OpenCV's ArUco detector on them with
the dictionary skyhub_core's observer1-sim profile uses (DICT_7X7_250: outer ID 228 at 1.00 m,
inner ID 11 at 0.09 m) and default DetectorParameters, as aruco_landing builds it.

For every picture it reports which IDs were found, the outer marker's side in pixels against the
side the pinhole projects, and the pixels one cell of the outer marker spans (a 7x7 marker is 9
cells across with its border: 0.111 m at 1.00 m). "--offset" also puts the marker that far east
of the point below the camera, as when the landing takes over within 15 m of home.

  python3 tools/render_probe/aruco_check.py --probe build/render_probe --tiles build/demo_map/tiles \\
      --out /tmp/aruco_probe --report /tmp/aruco_probe/report.md

Pictures go to --out (a new private temporary directory when it is not given) as PNG,
dock_<W>x<H>_hfov<F>_<height>m[_east<offset>m].png. Exit status 1 when a 1920x1080 picture with the
marker centred, from any height asked for between 3 and 30 m, misses the outer marker, ID 228.
"""
from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
import tempfile

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover - reported, not crashed on
    cv2 = None
    np = None

MARKER_M = 1.0
CELLS = 9  # 7x7 data cells plus a one-cell black border on each side


def detector():
    params = cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, "DetectorParameters_create") \
        else cv2.aruco.DetectorParameters()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_250)
    if hasattr(cv2.aruco, "ArucoDetector"):
        det = cv2.aruco.ArucoDetector(dictionary, params)
        return lambda grey: det.detectMarkers(grey)[:2]
    return lambda grey: cv2.aruco.detectMarkers(grey, dictionary, parameters=params)[:2]


def focal_px(width: int, hfov_deg: float) -> float:
    return width * 0.5 / math.tan(math.radians(hfov_deg) * 0.5)


def side_px(quad: np.ndarray) -> float:
    return float(np.mean([np.linalg.norm(quad[k] - quad[(k + 1) % 4]) for k in range(4)]))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--probe", default="build/render_probe")
    ap.add_argument("--tiles", required=True)
    ap.add_argument("--out", default="", help="where the pictures go (default: a new private temporary directory)")
    ap.add_argument("--report", default="")
    ap.add_argument("--heights", default="30,20,10,3")
    ap.add_argument("--sizes", default="1920x1080@81,1280x720@81,640x360@78",
                    help="WxH@HFOV per camera; 640x360@78 is the dev pilot camera's lens")
    ap.add_argument("--offset", type=float, default=10.0, help="also the marker this many metres east; 0 = off")
    ap.add_argument("--cache", type=float, default=256.0,
                    help="ground cache MB (the pilot camera's ground, as the server draws it); 0 = direct")
    ap.add_argument("--threads", type=int, default=4)
    args = ap.parse_args()
    if cv2 is None:
        print("aruco_check: needs python3 OpenCV (cv2) and numpy", file=sys.stderr)
        return 2
    if not args.out:
        args.out = tempfile.mkdtemp(prefix="aruco_probe_")  # private: nobody else's directory
    os.makedirs(args.out, exist_ok=True)
    print(f"aruco_check: pictures in {args.out}", flush=True)
    detect = detector()
    heights = [float(h) for h in args.heights.split(",") if h]
    offsets = [0.0] + ([args.offset] if args.offset > 0 else [])
    rows: list[dict] = []
    for spec in args.sizes.split(","):
        size, _, hfov = spec.partition("@")
        w, _, h = size.partition("x")
        width, height, hfov_deg = int(w), int(h), float(hfov or 81)
        f = focal_px(width, hfov_deg)
        for alt in heights:
            for off in offsets:
                if off > 0 and off + 0.6 > alt * math.tan(math.radians(hfov_deg) * 0.5):
                    continue  # the marker would be (partly) out of the picture at this height
                name = f"dock_{width}x{height}_hfov{hfov_deg:g}_{alt:g}m" + (f"_east{off:g}m" if off else "")
                ppm = os.path.join(args.out, name + ".ppm")
                cmd = [args.probe, "--tiles", args.tiles, "--raster", "--size", f"{width}x{height}", "--fov",
                       f"{hfov_deg:g}", "--pitch", "-90", "--alt", f"{alt:g}", "--dock", f"0,{off:g}",
                       "--threads", str(args.threads), "--out", ppm]
                if args.cache > 0:
                    cmd += ["--cache", f"{args.cache:g}"]
                subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
                image = cv2.imread(ppm, cv2.IMREAD_COLOR)
                cv2.imwrite(os.path.join(args.out, name + ".png"), image)
                os.remove(ppm)  # the PNG is the same pixels, a fraction of the size
                grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                corners, ids = detect(grey)
                found = {} if ids is None else {int(i): c.reshape(4, 2) for c, i in zip(corners, ids.ravel())}
                expected = MARKER_M * f / alt
                row = {
                    "picture": name + ".png",
                    "size": f"{width}x{height}",
                    "hfov": hfov_deg,
                    "height_m": alt,
                    "offset_m": off,
                    "px_per_cell": round(expected / CELLS, 2),
                    "ids": sorted(found),
                    "outer_228": 228 in found,
                    "inner_11": 11 in found,
                    "side_px": round(side_px(found[228]), 1) if 228 in found else None,
                    "side_px_expected": round(expected, 1),
                }
                rows.append(row)
                print(row, flush=True)
    lines = [f"OpenCV {cv2.__version__}, DICT_7X7_250, default DetectorParameters; marker 1.00 m (228) "
             f"with 0.09 m (11) inside; camera straight down; ground cache {'on' if args.cache > 0 else 'off'}.",
             "",
             "| camera | height | marker offset | px per cell | 228 (outer) | 11 (inner) | 228 side px (projected) |",
             "|---|---|---|---|---|---|---|"]
    for r in rows:
        side = f"{r['side_px']} ({r['side_px_expected']})" if r["side_px"] is not None else \
            f"- ({r['side_px_expected']})"
        lines.append(f"| {r['size']} HFOV {r['hfov']:g} | {r['height_m']:g} m | {r['offset_m']:g} m | "
                     f"{r['px_per_cell']} | {'detected' if r['outer_228'] else 'no detect'} | "
                     f"{'detected' if r['inner_11'] else 'no detect'} | {side} |")
    table = "\n".join(lines) + "\n"
    print(table)
    if args.report:
        with open(args.report, "w") as fh:
            fh.write(table)
    misses = [r for r in rows if r["size"] == "1920x1080" and r["offset_m"] == 0 and 3 <= r["height_m"] <= 30
              and not r["outer_228"]]
    return 1 if misses else 0


if __name__ == "__main__":
    sys.exit(main())
