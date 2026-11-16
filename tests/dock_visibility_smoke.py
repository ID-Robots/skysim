#!/usr/bin/env python3
"""A vehicle's own cameras draw its own dock alone (TASK-2609), proved on real skysim renders.

Every dock carries the same e-ArUco IDs (DICT_7X7_250 228 with 11 inside) and vehicles spawn 5 m
apart. A camera that drew every dock showed several identical markers in one picture, and
skyhub_core's aruco_landing, which takes the first detection of its ID, flew a dev SITL that
started 0.08 m above its own dock sideways onto slot 2's dock, 9.58 m away.

1. own (the default, no flag given): skysim with the pilot camera at 1920x1080 HFOV 81 (Observer 1's
   ZR30, as tests/dock_marker_smoke.py) and RTSP on. Slot 0 is flown by dock_marker_smoke's fake
   SITL straight up over its dock with the gimbal at -90 deg; slot 1 is parked on its dock 5 m
   east, and at every height a slot 2 is spawned on its dock 10 m east and despawned again. At 30,
   10 and 5 m, with two vehicles and then three, slot 0's pictures are captured: the pilot camera
   as camera.mjpg serves it and as H.264 RTSP frames, the survey still (still.jpg, gimbal=1) at
   the pilot camera's size and lens, and from 10 and 5 m a still at 640x360 HFOV 78, the dev pilot
   camera where the bug was seen. Each picture is decoded with cv2.aruco, DICT_7X7_250, default
   DetectorParameters (the detector dock_marker_smoke builds), and each 228 found is solved with
   cv2.solvePnP against the published intrinsics. In every picture exactly one 228 is found, and
   it is slot 0's dock: its lateral error against slot 0's ground truth (GET .../camera/dock) is
   within dock_marker_smoke's 10 cm. No picture shows a second 228. H.264 blurs a cell out of a
   frame now and then, so 9 RTSP frames in 10 must find it, and none may find two.
2. all (--dock-markers-visible all): the same flight to 30 and 10 m. More than one 228 is found
   in the stills and the pilot camera's pictures, each at a dock's ground truth: the test can
   see the difference.
3. args: bad --dock-markers-visible values stop startup with exit code 2, the startup log names
   the setting, and docker-entrypoint.sh maps SKYSIM_DOCK_MARKERS_VISIBLE to the flag.

OpenCV >= 4.7 finds the inner 11 rather than the outer 228 from 5 m down (dock_marker_smoke);
with that, 5 m asks for exactly one of the two, slot 0's. OpenCV 4.5 and 4.6, the versions the
dock tests run, find the outer at all three heights.

The captures stay in --frames DIR with a manifest; --analyse DIR re-checks them with another
OpenCV (CI: Ubuntu 22.04's 4.5.4, the SkyHub core image's). Exits 77 (skipped) without OpenCV's
aruco module or ffmpeg.

Usage: python3 tests/dock_visibility_smoke.py <path-to-skysim> [--frames DIR] [--analyse DIR]
       [--report FILE] [--camera-threads N] [--quick]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import stat
import subprocess
import sys
import tempfile
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dock_marker_smoke as dms  # its helpers: the detector, View, FakeSitl, ...

if dms.HAVE_CV:
    import cv2
    import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
# Ports and instances outside every other suite's (see the project's run notes).
OWN_API_PORT, OWN_RTSP_PORT, OWN_BASE = 18690, 18691, 40
ALL_API_PORT, ALL_RTSP_PORT, ALL_BASE = 18692, 18693, 44
WIDTH, HEIGHT, HFOV = 1920, 1080, 81.0
DEV_WIDTH, DEV_HEIGHT, DEV_HFOV = 640, 360, 78.0
SPACING_M = 5.0
PROFILES = {"full": {"own": [30.0, 10.0, 5.0], "all": [30.0, 10.0], "rtsp_frames": 10},
            "quick": {"own": [30.0, 10.0, 5.0], "all": [30.0, 10.0], "rtsp_frames": 3}}
DEV_HEIGHTS = [10.0, 5.0]  # where the dev camera resolves the marker (13 px from 30 m)
OUTER, INNER = 228, 11
# A detection is a dock's when its centre lies this close to where the dock's ground truth projects:
# 8 cm from 30 m at 1920 px, against docks 5 m (190 px) apart.
CENTRE_TOL_PX = 3.0

FAILURES: list[str] = []


def check(cond: bool, what: str) -> None:
    print(f"dock_visibility_smoke: {'ok  ' if cond else 'FAIL'} {what}", flush=True)
    if not cond:
        FAILURES.append(what)


def pinhole(width: int, height: int, hfov_deg: float) -> list[list[float]]:
    """The camera matrix GET .../camera/intrinsics publishes for this size and lens."""
    f = width / 2 / math.tan(math.radians(hfov_deg / 2))
    return [[f, 0.0, width / 2 - 0.5], [0.0, f, height / 2 - 0.5], [0.0, 0.0, 1.0]]


# --- capture -----------------------------------------------------------------------------------


def vehicles(port: int) -> list[dict]:
    return dms.api(port, "GET", "/vehicles")


def wait_fleet(port: int, instances: set[int], seconds: float = 10.0) -> bool:
    """Until GET /vehicles lists exactly these instances (spawn and despawn both land a tick late)."""
    deadline = time.time() + seconds
    while time.time() < deadline:
        if {v["instance"] for v in vehicles(port)} == instances:
            return True
        time.sleep(0.05)
    return False


def mjpeg_frame(port: int, instance: int, not_before_s: float, tries: int = 30) -> tuple[bytes, float] | None:
    """The first part of /instances/<n>/camera.mjpg drawn from a pose at or after sim time
    `not_before_s` (X-Skysim-Time-Usec), so never a picture left over from an earlier height."""
    url = f"http://127.0.0.1:{port}/instances/{instance}/camera.mjpg"
    with urllib.request.urlopen(url, timeout=30) as resp:
        for _ in range(tries):
            headers: dict[str, str] = {}
            line = resp.readline()
            while line and not line.startswith(b"--skysimframe"):
                line = resp.readline()
            if not line:
                return None
            while True:
                line = resp.readline().strip()
                if not line:
                    break
                key, _, value = line.decode().partition(":")
                headers[key.strip().lower()] = value.strip()
            data = resp.read(int(headers["content-length"]))
            sim_s = int(headers["x-skysim-time-usec"]) / 1e6
            if sim_s >= not_before_s:
                return data, sim_s
    return None


def capture(port: int, rtsp_port: int, base: int, height: float, frames_dir: str, tag: str,
            rtsp_frames: int) -> dict:
    """Slot 0's pictures at this hover, with every present dock's ground truth."""
    present = sorted(v["instance"] for v in vehicles(port))
    docks = {str(i): dms.api(port, "GET", f"/instances/{i}/camera/dock") for i in present}
    before = docks[str(base)]["vehicle_pose"]
    entry: dict = {"height_m": height, "instances": present, "docks": docks, "pictures": []}

    def keep(name: str, data: bytes) -> str:
        with open(os.path.join(frames_dir, name), "wb") as fh:
            fh.write(data)
        return name

    for kind, (w, h, hfov) in (("still", (WIDTH, HEIGHT, HFOV)), ("still_dev", (DEV_WIDTH, DEV_HEIGHT, DEV_HFOV))):
        if kind == "still_dev" and height not in DEV_HEIGHTS:
            continue
        jpeg, headers = dms.still(port, f"/instances/{base}/still.jpg?w={w}&h={h}&hfov={hfov}&gimbal=1&q=95")
        angles = [float(headers[f"x-skysim-{a}"]) for a in ("yaw", "pitch", "roll")]
        check(abs(angles[1] + 90.0) < 0.2 and headers.get("x-skysim-gimbal") == "1",
              f"{tag}: {kind} {w}x{h} drawn on the gimbal looking straight down (pitch {angles[1]:.2f})")
        entry["pictures"].append({"kind": kind, "file": keep(f"{tag}_{kind}.jpg", jpeg),
                                  "camera_ned": [float(x) for x in headers["x-skysim-ned"].split(",")],
                                  "angles_deg": angles, "k": pinhole(w, h, hfov)})
    # The pilot camera looks along the same gimbal from the vehicle's position.
    angles = entry["pictures"][0]["angles_deg"]
    frame = mjpeg_frame(port, base, before["sim_time_s"])
    check(frame is not None, f"{tag}: a camera.mjpg part drawn at this hover")
    if frame is not None:
        # The vehicle's position just after that part was drawn; the drift check below spans it.
        drawn = dms.api(port, "GET", f"/instances/{base}/camera/dock")["vehicle_pose"]
        entry["pictures"].append({"kind": "mjpeg", "file": keep(f"{tag}_mjpeg.jpg", frame[0]),
                                  "camera_ned": drawn["pos_ned"], "angles_deg": angles,
                                  "k": pinhole(WIDTH, HEIGHT, HFOV), "sim_time_s": frame[1]})
    files = dms.grab_rtsp(f"rtsp://127.0.0.1:{rtsp_port}/instances/{base}/main.264",
                          os.path.join(frames_dir, f"{tag}_rtsp"), rtsp_frames)
    after = dms.api(port, "GET", f"/instances/{base}/camera/dock")["vehicle_pose"]
    drift = math.dist(before["pos_ned"], after["pos_ned"])
    check(len(files) == rtsp_frames and drift < 0.01,
          f"{tag}: {len(files)} RTSP frames (slot 0 moved {drift * 1000:.1f} mm from before the camera.mjpg "
          "part to after the RTSP frames)")
    entry["pictures"] += [{"kind": "rtsp", "file": os.path.relpath(f, frames_dir), "camera_ned": after["pos_ned"],
                           "angles_deg": angles, "k": pinhole(WIDTH, HEIGHT, HFOV)} for f in files]
    return entry


def fly(sim: str, mode: str, frames_dir: str, threads: int, profile: dict) -> dict:
    """One skysim, slot 0 flown over its dock through the heights; `mode` is "own" (no flag: the
    default) or "all"."""
    own = mode == "own"
    port, rtsp_port, base = ((OWN_API_PORT, OWN_RTSP_PORT, OWN_BASE) if own else
                             (ALL_API_PORT, ALL_RTSP_PORT, ALL_BASE))
    cmd = [sim, "--vehicles", "0", "--base-instance", str(base), "--time-mode", "interactive",
           "--api-port", str(port), "--rtsp-port", str(rtsp_port), "--camera-fps", "10",
           "--camera-size", f"{WIDTH}x{HEIGHT}", "--camera-fov", str(HFOV), "--straggler-timeout", "10"]
    if not own:
        cmd += ["--dock-markers-visible", "all"]
    if threads > 0:
        cmd += ["--camera-threads", str(threads), "--still-threads", str(threads), "--video-threads", "1"]
    log_path = os.path.join(frames_dir, f"skysim_{mode}.log")
    run: dict = {"mode": mode, "base_instance": base, "heights": []}
    with open(log_path, "w") as log:
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
    sitl = None
    try:
        check(dms.wait_up(port, proc), f"{mode}: skysim up, 1920x1080 HFOV 81 camera and RTSP")
        for i in (base, base + 1):
            dms.api(port, "POST", "/vehicles", json.dumps({"instance": i}).encode())
        check(wait_fleet(port, {base, base + 1}), f"{mode}: slots 0 and 1 spawned on their docks")
        intr = dms.api(port, "GET", f"/instances/{base}/camera/intrinsics")
        check(bool(np.allclose(intr["camera_matrix"], pinhole(WIDTH, HEIGHT, HFOV), atol=1e-6)),
              f"{mode}: the published camera matrix is the pinhole this test solves with")
        east = [dms.api(port, "GET", f"/instances/{i}/camera/dock")["pose"]["east_m"] for i in (base, base + 1)]
        check(east == [0.0, SPACING_M], f"{mode}: docks {SPACING_M:g} m apart (east {east})")
        sitl = dms.FakeSitl(9002 + 10 * base)
        sitl.start()
        for height in profile[mode]:
            check(sitl.hold(height), f"{mode}: slot 0 hovering at {height:g} m")
            tag = f"{mode}_{height:g}m"
            run["heights"].append(capture(port, rtsp_port, base, height, frames_dir, f"{tag}_2v",
                                          profile["rtsp_frames"]))
            dms.api(port, "POST", "/vehicles", json.dumps({"instance": base + 2}).encode())
            check(wait_fleet(port, {base, base + 1, base + 2}), f"{tag}: slot 2 spawned on its dock 10 m east")
            run["heights"].append(capture(port, rtsp_port, base, height, frames_dir, f"{tag}_3v",
                                          profile["rtsp_frames"]))
            third = next(v["id"] for v in vehicles(port) if v["instance"] == base + 2)
            dms.api(port, "DELETE", f"/vehicles/{third}")
            check(wait_fleet(port, {base, base + 1}), f"{tag}: slot 2 despawned")
        sitl.hold(0.0, timeout=1.0)
    finally:
        if sitl is not None:
            sitl.done.set()
            sitl.join(timeout=5)
        check(dms.stop(proc) == 0, f"{mode}: clean shutdown")
    with open(log_path) as fh:
        log = fh.read()
    check(f"(--dock-markers-visible {mode})" in log, f"{mode}: the startup log names the setting")
    return run


# --- analysis ----------------------------------------------------------------------------------


def detections(grey, view, docks: dict, base: int) -> list[dict]:
    """Every 228 and 11 in the picture. Which dock it is: the one whose ground-truth centre projects
    nearest the detected quad's centre (px), which a range error cannot move. For slot 0's dock,
    the landing error too: tvec across the optical axis against the truth, as dock_marker_smoke.
    (Off the optical axis tvec's lateral part carries the range error times the angle — 7 to 12 cm
    for a dock 5 to 10 m off-axis from 30 m — so it does not say which dock a marker is.)"""
    if None not in dms.DETECTORS:
        dms.DETECTORS[None] = dms.detector(None)  # default DetectorParameters, as aruco_landing
    corners, ids = dms.DETECTORS[None](grey)
    out = []
    for c, i in zip(corners or [], [] if ids is None else ids.ravel().tolist()):
        if i not in (OUTER, INNER):
            continue
        key = "outer" if i == OUTER else "inner"
        quad = c.reshape(4, 2).astype(np.float64)
        centre = quad.mean(axis=0)
        px = {int(inst): float(np.linalg.norm(centre - np.array(view.project(d[key]["centre_ned"]))))
              for inst, d in docks.items()}
        nearest = min(px, key=px.get)
        t = dms.solve(quad, docks[str(base)][key]["size_m"], view.k, cv2.SOLVEPNP_ITERATIVE)
        lateral = float(np.linalg.norm(t[:2] - view.to_camera(docks[str(base)][key]["centre_ned"])[:2]))
        out.append({"id": int(i), "dock": nearest, "centre_err_px": round(px[nearest], 2),
                    "slot0_lateral_err_m": round(lateral, 4) if nearest == base else None})
    return out


def judge(run: dict, frames_dir: str, report: dict, rtsp_frames: int) -> None:
    mode, base = run["mode"], run["base_instance"]
    for h in run["heights"]:
        docks = h["docks"]
        n = len(h["instances"])
        height = h["height_m"]
        tag = f"{mode} {height:g} m, {n} vehicles"
        # Every picture this height should have, on disk, before any is judged: a capture with one
        # missing must fail rather than pass on the pictures it has.
        pictures = h.get("pictures", [])
        want = {"still": 1, "mjpeg": 1, "rtsp": rtsp_frames, "still_dev": 1 if height in DEV_HEIGHTS else 0}
        have = {k: sum(1 for p in pictures if p["kind"] == k) for k in want}
        on_disk = all(os.path.isfile(os.path.join(frames_dir, p["file"])) for p in pictures)
        check(have == want and on_disk, f"{tag}: every picture captured and on disk ({have}, want {want})")
        rows: dict[str, list[list[dict]]] = {}
        for p in pictures:
            grey = cv2.imread(os.path.join(frames_dir, p["file"]), cv2.IMREAD_GRAYSCALE)
            if grey is None:
                check(False, f"{tag}: {p['file']} reads as an image")
                continue
            view = dms.View(p["camera_ned"], p["angles_deg"], np.array(p["k"], dtype=np.float64))
            rows.setdefault(p["kind"], []).append(detections(grey, view, docks, base))
        summary: dict = {"mode": mode, "height_m": height, "vehicles": n}
        for kind, found in rows.items():
            outer = [[d for d in f if d["id"] == OUTER] for f in found]
            inner = [[d for d in f if d["id"] == INNER] for f in found]
            counts = [len(o) for o in outer]
            own_hits = [d["slot0_lateral_err_m"] for f in found for d in f if d["dock"] == base]
            summary[kind] = {"frames": len(found), "outer_228_counts": counts,
                             "inner_11_counts": [len(i) for i in inner],
                             "worst_lateral_err_m": max(own_hits, default=None),
                             "worst_centre_err_px": max((d["centre_err_px"] for f in found for d in f), default=None),
                             "docks_seen": sorted({d["dock"] for f in found for d in f})}
            what = f"{tag}: {kind}"
            s = summary[kind]
            if mode == "own":
                # The outer must be the one found, except where OpenCV >= 4.7 takes the inner (5 m).
                outer_due = not dms.NEW_DETECTOR or height > 5.0
                good = [all(d["dock"] == base and d["slot0_lateral_err_m"] <= dms.POS_TOL_M for d in o + i) and
                        len(o) <= 1 and len(i) <= 1 and (len(o) == 1 if outer_due else len(o) + len(i) == 1)
                        for o, i in zip(outer, inner)]
                need = len(found) if kind != "rtsp" else math.ceil(0.9 * len(found))
                check(max(counts, default=0) <= 1 and s["docks_seen"] in ([], [base]),
                      f"{what}: never more than one 228, never another dock (counts {counts}, "
                      f"docks {s['docks_seen']})")
                check(sum(good) >= need,
                      f"{what}: exactly one {'228' if outer_due else '228 or 11'}, slot 0's dock within "
                      f"{dms.POS_TOL_M * 100:g} cm, in {sum(good)}/{len(found)} (need {need}); worst lateral "
                      f"{s['worst_lateral_err_m']} m, centre {s['worst_centre_err_px']} px")
            else:
                # Every dock in shot is drawn: more than one 228, each on its own dock — its centre
                # within CENTRE_TOL_PX of where that dock's ground truth projects — and slot 0's at
                # its landing tolerance.
                many = [len(o) > 1 and len({d["dock"] for d in o}) == len(o) and
                        all(d["centre_err_px"] <= CENTRE_TOL_PX for d in o) and
                        all(d["slot0_lateral_err_m"] <= dms.POS_TOL_M for d in o if d["dock"] == base)
                        for o in outer]
                need = len(found) if kind != "rtsp" else math.ceil(0.9 * len(found))
                check(sum(many) >= need,
                      f"{what}: more than one 228, each at its own dock (centre within {CENTRE_TOL_PX:g} px), in "
                      f"{sum(many)}/{len(found)} (need {need}); counts {counts}, docks seen {s['docks_seen']}, "
                      f"worst centre {s['worst_centre_err_px']} px, slot 0 lateral {s['worst_lateral_err_m']} m")
        report.setdefault("rows", []).append(summary)


def write_report(report: dict, path: str) -> None:
    lines = ["# A vehicle's cameras draw its own dock alone (TASK-2609)", "",
             "Generated by `tests/dock_visibility_smoke.py --report`. Do not edit by hand.", "",
             (f"OpenCV {report['opencv']} ({report['detector']}), DICT_7X7_250, default DetectorParameters. "
              f"Pilot camera {WIDTH}x{HEIGHT} HFOV {HFOV:g}; dev still {DEV_WIDTH}x{DEV_HEIGHT} HFOV {DEV_HFOV:g}. "
              f"Docks {SPACING_M:g} m apart. Each cell: how many 228s were found (RTSP: frames x count), slot "
              "0's worst lateral (landing) error, the worst detected centre against its dock's projected one."),
             "", "| mode | height | vehicles | still 1920x1080 | still 640x360 | camera.mjpg | RTSP frames |",
             "|---|---|---|---|---|---|---|"]

    def cell(r: dict, kind: str) -> str:
        s = r.get(kind)
        if not s:
            return "-"
        counts = s["outer_228_counts"]
        found = ", ".join(f"{counts.count(c)}x{c}" for c in sorted(set(counts))) if kind == "rtsp" else str(counts[0])
        err = s["worst_lateral_err_m"]
        return f"{found}; {'-' if err is None else f'{err * 100:.1f} cm'}; {s['worst_centre_err_px']} px"

    for r in report.get("rows", []):
        lines.append(f"| {r['mode']} | {r['height_m']:g} m | {r['vehicles']} | {cell(r, 'still')} | {cell(r, 'still_dev')} "
                     f"| {cell(r, 'mjpeg')} | {cell(r, 'rtsp')} |")
    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")


# --- args --------------------------------------------------------------------------------------


def case_args(sim: str) -> None:
    for bad in (["--dock-markers-visible", "none"], ["--dock-markers-visible", "OWN"],
                ["--dock-markers-visible", ""], ["--dock-markers-visible", "1"], ["--dock-markers-visible"]):
        rc = subprocess.run([sim, "--vehicles", "0", *bad], capture_output=True, timeout=30, check=False)
        check(rc.returncode == 2 and b"--dock-markers-visible" in rc.stderr,
              f"{' '.join(bad) or '(empty)'} refused ({rc.stderr.decode().strip()})")
    # docker-entrypoint.sh maps the variable to the flag, and leaves the default alone when unset.
    with tempfile.TemporaryDirectory() as stub:
        fake = os.path.join(stub, "skysim")
        with open(fake, "w") as fh:
            fh.write('#!/bin/sh\nprintf "%s\\n" "$@"\n')
        os.chmod(fake, os.stat(fake).st_mode | stat.S_IXUSR)
        env = {k: v for k, v in os.environ.items() if not k.startswith("SKYSIM_")}
        env["PATH"] = f"{stub}:{env.get('PATH', '')}"
        entry = os.path.join(ROOT, "docker-entrypoint.sh")
        for value, flag in (("all", ["--dock-markers-visible", "all"]), ("own", ["--dock-markers-visible", "own"]),
                            (None, [])):
            e = dict(env)
            if value is not None:
                e["SKYSIM_DOCK_MARKERS_VISIBLE"] = value
            out = subprocess.run(["bash", entry], capture_output=True, text=True, env=e, timeout=30, check=False)
            argv = out.stdout.splitlines()
            got = argv[argv.index("--dock-markers-visible"):argv.index("--dock-markers-visible") + 2] \
                if "--dock-markers-visible" in argv else []
            check(out.returncode == 0 and got == flag,
                  f"docker-entrypoint.sh: SKYSIM_DOCK_MARKERS_VISIBLE={value} -> {got or 'no flag (the default)'}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("sim", nargs="?", default="./build/skysim")
    parser.add_argument("--frames", help="keep the captured pictures and their manifest here")
    parser.add_argument("--analyse", metavar="DIR", help="no skysim: re-check pictures captured with --frames")
    parser.add_argument("--report", help="write the results table (markdown) here")
    parser.add_argument("--camera-threads", type=int, default=0, help="render threads (0: skysim's default)")
    parser.add_argument("--quick", action="store_true", help="3 RTSP frames a picture (the coverage build)")
    args = parser.parse_args()
    if not dms.HAVE_CV or (not args.analyse and shutil.which("ffmpeg") is None):
        print("dock_visibility_smoke: SKIP — needs OpenCV with cv2.aruco (python3-opencv) and ffmpeg")
        return 77
    report: dict = {"opencv": cv2.__version__,
                    "detector": "cv2.aruco.ArucoDetector" if dms.NEW_DETECTOR else "cv2.aruco.detectMarkers (legacy)"}
    print(f"dock_visibility_smoke: OpenCV {cv2.__version__} ({cv2.__file__}), {report['detector']}")
    with tempfile.TemporaryDirectory() as tmp:
        if args.analyse:
            frames_dir = args.analyse
            with open(os.path.join(frames_dir, "manifest.json")) as fh:
                manifest = json.load(fh)
        else:
            frames_dir = args.frames or os.path.join(tmp, "frames")
            shutil.rmtree(frames_dir, ignore_errors=True)  # never analyse an earlier run's pictures
            os.makedirs(frames_dir)
            case_args(args.sim)
            profile_name = "quick" if args.quick else "full"
            profile = PROFILES[profile_name]
            manifest = {"profile": profile_name, "runs": [fly(args.sim, m, frames_dir, args.camera_threads, profile)
                                                          for m in ("own", "all")]}
            with open(os.path.join(frames_dir, "manifest.json"), "w") as fh:
                json.dump(manifest, fh, indent=1)
        profile = PROFILES.get(manifest.get("profile", ""))
        runs = {r["mode"]: r for r in manifest.get("runs", [])}
        whole = profile is not None and set(runs) == {"own", "all"} and all(
            [h["height_m"] for h in runs[m]["heights"]] == [x for x in profile[m] for _ in (2, 3)] for m in runs)
        check(whole, f"the capture is whole: both runs, every height with 2 and 3 vehicles ({manifest.get('profile')})")
        if whole:
            for m in ("own", "all"):
                judge(runs[m], frames_dir, report, profile["rtsp_frames"])
    if args.report:
        write_report(report, args.report)
    for r in report.get("rows", []):
        print(f"dock_visibility_smoke: {r['mode']:>3} {r['height_m']:>4g} m {r['vehicles']} vehicles: " +
              "; ".join(f"{k} 228 {r[k]['outer_228_counts']} worst {r[k]['worst_lateral_err_m']} m"
                        for k in ("still", "still_dev", "mjpeg", "rtsp") if k in r))
    if FAILURES:
        print(f"dock_visibility_smoke: {len(FAILURES)} failure(s)")
        return 1
    print("dock_visibility_smoke: all passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
