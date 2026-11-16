#!/usr/bin/env python3
"""Observer 1's e-ArUco dock marker in skysim, end to end: the TASK-2562 proof.

1. asset: decodes the committed generator image (assets/markers/) with cv2.aruco. Outer ID 228
   must decode at every scale and inner ID 11 at close range (cropped to its white cell). The run
   also records where ID 11 drops out with OpenCV's default DetectorParameters, which is what
   skyhub_core's aruco_landing uses (minMarkerDistanceRate 0.05), next to minMarkerDistanceRate
   0.04 (assets/markers/README.md).
2. flight: starts skysim with the pilot camera at 1920x1080 and HFOV 81 deg (Observer 1's ZR30 at
   1x zoom, the CAMERA_HFOV aruco_landing assumes) and RTSP on. A fake SITL flies one vehicle
   straight up over its dock in JSON lockstep. All four motors get one altitude-hold PWM and the
   gimbal pitch output is held at -90 deg (straight down, as for docking). At each height the
   test pulls still.jpg (gimbal=1) and frames from the H.264 RTSP stream. It decodes them with
   cv2.aruco and solves each marker's pose with cv2.solvePnP and the intrinsics from
   GET /instances/<n>/camera/intrinsics, then compares the result with the ground truth from
   GET /instances/<n>/camera/dock.
3. idle: with the camera on, a vehicle on its dock and nobody watching, skysim stays near idle
   and never builds the marker's texture. It builds it once the first frame is asked for.

The main proof uses OpenCV's default DetectorParameters, as aruco_landing does. The 0.04 column
is a comparison. Which marker the defaults find depends on the OpenCV version, because 4.7
replaced the candidate filter:
  - OpenCV >= 4.7 (cv2.aruco.ArucoDetector): outer 228 at 30, 21.5 and 15 m, inner 11 from 5 m
    down to 0.7 m, never both at once. This matches Observer 1's flight log (switch at 5 m).
  - OpenCV 4.6 (legacy cv2.aruco.detectMarkers): outer 228 from 30 m down to 1.2 m, and inner 11
    only with minMarkerDistanceRate <= 0.04 (from 5 m down to 0.7 m).
For each marker the installed version should find (EXPECT), in the still and in the RTSP frames:
  - the marker decodes, in the still and in 9 RTSP frames of 10 (at least one for a marker under
    30 px, which is at the stream's limit): H.264 at its default 1200 kbit/s for 1080p blurs a
    cell out of a frame now and then;
  - its lateral error is within 10 cm. This is tvec, the marker's centre relative to the camera,
    across the optical axis: what a landing controller steers on;
  - the recovered scale (the marker size implied by the true distance and the solved one) is
    within 5 %, or within 8 % where the quad frames the 2 cm white border's outline rather than
    the black square (outer 228 on ground darker than the border), or within 12 % for a marker
    under 30 px, whose side cv2.aruco's integer corners get a pixel or two wrong;
  - detector-free: each black square's side, measured off the pixels, is within 2 px (or 1 %) of
    what the published intrinsics and ground truth project.

Exits 77 (skipped) without OpenCV's aruco module or ffmpeg.

Usage: python3 tests/dock_marker_smoke.py <path-to-skysim> [--report DIR] [--camera-threads N] [--quick]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request

try:
    import cv2
    import numpy as np

    HAVE_CV = hasattr(cv2, "aruco")
except ImportError:  # pragma: no cover - the skip path
    HAVE_CV = False

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
ASSET = os.path.join(ROOT, "assets", "markers", "e_aruco_228_11_100cm.png")
API_PORT = 18674
RTSP_PORT = 18675
IDLE_API_PORT = 18676
IDLE_RTSP_PORT = 18677
INSTANCE = 85
IDLE_INSTANCE = 86
OFF_API_PORT = 18678
OFF_RTSP_PORT = 18679
OFF_INSTANCE = 87
WIDTH, HEIGHT, HFOV = 1920, 1080, 81.0
MODE = "full"
HEIGHTS = [30.0, 21.5, 15.0, 5.0, 2.0, 1.2, 0.7]
# The two captures this script makes: every height with 10 RTSP frames, or (--quick, the coverage
# build) three heights with 3. A manifest must be one of them to be analysed.
PROFILES = {"full": ([30.0, 21.5, 15.0, 5.0, 2.0, 1.2, 0.7], 10), "quick": ([30.0, 5.0, 0.7], 3)}

# Which marker each detector setting must find at which heights, by OpenCV generation.
NEW_DETECTOR = HAVE_CV and hasattr(cv2.aruco, "ArucoDetector")
HIGH, LOW = [30.0, 21.5, 15.0], [5.0, 2.0, 1.2, 0.7]
if NEW_DETECTOR:
    EXPECT = {"default": {"228": HIGH, "11": LOW}, "rate_0.04": {"228": HIGH, "11": LOW}}
else:
    EXPECT = {"default": {"228": [*HIGH, 5.0, 2.0, 1.2], "11": []},
              "rate_0.04": {"228": [*HIGH, 5.0, 2.0, 1.2], "11": LOW}}
RTSP_FRAMES = 10
SIZE_TOL = 0.05
POS_TOL_M = 0.10
RATES = {"default": None, "rate_0.04": 0.04}

FAILURES: list[str] = []


def check(cond: bool, what: str) -> None:
    print(f"dock_marker_smoke: {'ok  ' if cond else 'FAIL'} {what}", flush=True)
    if not cond:
        FAILURES.append(what)


def api(port: int, method: str, path: str, body: bytes | None = None) -> dict | list:
    req = urllib.request.Request(f"http://127.0.0.1:{port}{path}", data=body, method=method)
    with urllib.request.urlopen(req, timeout=30) as resp:
        return json.loads(resp.read().decode() or "{}")


def wait_up(port: int, proc: subprocess.Popen, seconds: float = 20.0) -> bool:
    deadline = time.time() + seconds
    while time.time() < deadline and proc.poll() is None:
        try:
            api(port, "GET", "/metrics")
            return True
        except (urllib.error.URLError, ConnectionError):
            time.sleep(0.1)
    return False


def listed(port: int, count: int) -> list:
    """POST /vehicles answers before the tick publishes the snapshot, so poll the list."""
    deadline = time.time() + 10
    vehicles: list = []
    while time.time() < deadline:
        vehicles = api(port, "GET", "/vehicles")
        if len(vehicles) >= count:
            break
        time.sleep(0.05)
    return vehicles


def stop(proc: subprocess.Popen) -> int:
    proc.send_signal(signal.SIGINT)
    try:
        return proc.wait(timeout=30)
    except subprocess.TimeoutExpired:
        proc.kill()
        return -1


def process_cpu_s(pid: int) -> float:
    with open(f"/proc/{pid}/stat") as f:
        fields = f.read().rsplit(")", 1)[1].split()
    return (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")


# --- detection and pose ------------------------------------------------------------------------


def detector(rate: float | None):
    params = cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, "DetectorParameters_create") \
        else cv2.aruco.DetectorParameters()
    if rate is not None:
        params.minMarkerDistanceRate = rate
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_250)
    if hasattr(cv2.aruco, "ArucoDetector"):
        det = cv2.aruco.ArucoDetector(dictionary, params)
        return lambda grey: det.detectMarkers(grey)[:2]
    return lambda grey: cv2.aruco.detectMarkers(grey, dictionary, parameters=params)[:2]


DETECTORS = {}


def detect(grey, rate: float | None) -> dict[int, np.ndarray]:
    if rate not in DETECTORS:
        DETECTORS[rate] = detector(rate)
    corners, ids = DETECTORS[rate](grey)
    found: dict[int, np.ndarray] = {}
    if ids is not None:
        for c, i in zip(corners, ids.ravel().tolist()):
            found.setdefault(int(i), c.reshape(4, 2).astype(np.float64))
    return found


class View:
    """A frame's camera: where it is (NED), its axes (Z-Y-X yaw/pitch/roll, degrees, as
    X-Skysim-Yaw/-Pitch/-Roll give them) and the published intrinsics."""

    def __init__(self, cam_ned: list[float], angles: list[float], k: np.ndarray) -> None:
        y, p, r = (math.radians(a) for a in angles)
        rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
        ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
        rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
        rot = rz @ ry @ rx
        self.fwd, self.right, self.down = rot[:, 0], rot[:, 1], rot[:, 2]
        self.cam = np.array(cam_ned, dtype=np.float64)
        self.k = k

    def to_camera(self, ned: list[float]) -> np.ndarray:
        """A NED point in OpenCV's camera frame: x right, y down, z along the optical axis."""
        d = np.array(ned, dtype=np.float64) - self.cam
        return np.array([d @ self.right, d @ self.down, d @ self.fwd])

    def project(self, ned: list[float]) -> tuple[float, float]:
        c = self.to_camera(ned)
        return (self.k[0, 0] * c[0] / c[2] + self.k[0, 2], self.k[1, 1] * c[1] / c[2] + self.k[1, 2])


def dock_point(dock: dict, square: dict, u: float, v: float) -> list[float]:
    """A point on a square of the marker, (u, v) in its own side lengths from its centre (u to the
    image's right, v down), in NED."""
    yaw = math.radians(dock["pose"]["yaw_deg"])
    n0, e0 = square["centre_ned"][0], square["centre_ned"][1]
    s = square["size_m"]
    return [n0 - u * s * math.sin(yaw) - v * s * math.cos(yaw), e0 + u * s * math.cos(yaw) - v * s * math.sin(yaw),
            0.0]


def measured_side(grey, view: View, dock: dict, square: dict) -> tuple[float | None, float] | None:
    """The square's black side in the frame, measured from pixels alone: the dark run along its
    bottom border row (black in every ArUco marker), against the side the camera model projects.
    None when the row is not wholly in the frame or not resolved; a measurement of None when it
    should be and the pixels show no dark run there (a failure, not a skip)."""
    v = 0.5 - 0.5 / 9.0
    (xa, ya), (xb, _) = view.project(dock_point(dock, square, -0.5, v)), view.project(dock_point(dock, square, 0.5, v))
    h, w = grey.shape
    row = round(ya)
    if not (0 <= row < h and 2 <= min(xa, xb) and max(xa, xb) < w - 2):
        return None
    if abs(xb - xa) < 9.0:
        return None  # under a pixel a cell: the border row is not resolved
    line = grey[row].astype(np.float64)
    white = float(np.percentile(line[max(int(min(xa, xb)) - 40, 0):int(max(xa, xb)) + 40], 99))
    mid = (white + 8.0) / 2.0
    x = round((xa + xb) / 2.0)
    if line[x] >= mid:
        return None, float(abs(xb - xa))
    a = x
    while a > 0 and line[a - 1] < mid:
        a -= 1
    b = x
    while b < w - 1 and line[b + 1] < mid:
        b += 1
    return float(b - a + 1), float(abs(xb - xa))


def solve(corners: np.ndarray, size: float, k: np.ndarray, flag: int) -> np.ndarray:
    """The marker's centre in the camera frame (OpenCV: x right, y down, z forward): tvec."""
    h = size / 2.0
    obj = np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])
    _, _, tvec = cv2.solvePnP(obj, corners, k, np.zeros(5), flags=flag)
    return tvec.ravel()


def evaluate(grey, view: View, dock: dict) -> dict:
    """Per marker: the geometry measured off the pixels against the camera model, then for each
    detector setting whether cv2.aruco found it and how far solvePnP's pose is from the truth.

    The pose compared is tvec, the marker's centre relative to the camera, which is what a
    landing controller steers on. lateral_err_m is its error across the optical axis (the
    landing error). size_err_pct is the scale error: the marker size implied by the true distance
    and the solved one. Solved with SOLVEPNP_ITERATIVE. SOLVEPNP_IPPE_SQUARE, which
    cv2.aruco.estimatePoseSingleMarkers uses, is reported next to it. OpenCV 4.6's IPPE_SQUARE
    returns a flipped rotation when the four corners are exactly symmetric about the image centre,
    which only happens with a perfectly level camera exactly over the marker (a fake SITL can
    hover like that). Its tvec, the part a landing uses, is compared as well."""
    out: dict = {"geometry": {}}
    for mid, key in ((228, "outer"), (11, "inner")):
        m = measured_side(grey, view, dock, dock[key])
        if m is not None:
            out["geometry"][str(mid)] = {"measured_px": m[0], "projected_px": round(m[1], 2),
                                         "err_px": None if m[0] is None else round(m[0] - m[1], 2)}
    for name, rate in RATES.items():
        found = detect(grey, rate)
        per: dict = {}
        for mid, key in ((228, "outer"), (11, "inner")):
            if mid not in found:
                per[str(mid)] = {"found": False}
                continue
            square = dock[key]
            truth = view.to_camera(square["centre_ned"])
            corners = found[mid]
            t = solve(corners, square["size_m"], view.k, cv2.SOLVEPNP_ITERATIVE)
            t_ippe = solve(corners, square["size_m"], view.k, cv2.SOLVEPNP_IPPE_SQUARE)
            scale = float(np.linalg.norm(truth) / np.linalg.norm(t))
            side_px = float(np.mean(np.linalg.norm(corners - np.roll(corners, 1, axis=0), axis=1)))
            r = {
                "found": True,
                "side_px": round(side_px, 1),
                "size_est_m": round(square["size_m"] * scale, 5),
                "size_err_pct": round(abs(scale - 1.0) * 100.0, 3),
                "lateral_err_m": round(float(np.linalg.norm(t[:2] - truth[:2])), 4),
                "distance_err_m": round(float(np.linalg.norm(t) - np.linalg.norm(truth)), 4),
                "ippe_lateral_err_m": round(float(np.linalg.norm(t_ippe[:2] - truth[:2])), 4),
            }
            r["within"] = r["size_err_pct"] <= SIZE_TOL * 100 and r["lateral_err_m"] <= POS_TOL_M
            per[str(mid)] = r
        out[name] = per
    return out


# --- 1. the committed image -------------------------------------------------------------------


def case_asset(report: dict) -> None:
    img = cv2.imread(ASSET, cv2.IMREAD_GRAYSCALE)
    check(img is not None and img.shape == (4093, 4093), "the committed marker image reads, 4093x4093")
    if img is None:
        return
    check(set(np.unique(img).tolist()) == {0, 255}, "it is pure black and white, as the generator writes it")
    rows = []
    # Whole image at falling resolutions, then the inner marker cropped with ever more of the outer
    # marker around it: its own white cell (44 px each side), one cell, three cells.
    views = [("full", s, img, s) for s in (4093, 2048, 1024, 600, 300, 200)]
    for name, margin in (("inner+cell", 44), ("inner+1 cell", 44 + 437), ("inner+3 cells", 44 + 3 * 437)):
        crop = img[1870 - margin:2219 + margin, 1870 - margin:2219 + margin]
        views += [(name, s, crop, s) for s in (crop.shape[0], 600, 200, 100)]
    for name, scale, src, size in views:
        view = src if src.shape[0] == size else cv2.resize(src, (size, size), interpolation=cv2.INTER_AREA)
        # How wide the inner marker is in this view: under ~45 px (5 px a cell) nothing decodes it.
        row = {"view": name, "px": scale, "inner_px": round(349 * size / src.shape[0])}
        for rname, rate in RATES.items():
            row[rname] = sorted(detect(view, rate))
        rows.append(row)
    report["asset"] = rows
    print("dock_marker_smoke: asset decode table (view, px, inner px, default ids, rate-0.04 ids):")
    for r in rows:
        print(f"    {r['view']:<14} {r['px']:>5} {r['inner_px']:>5}  {r['default']!s:<10} {r['rate_0.04']!s}")
    full = [r for r in rows if r["view"] == "full"]
    resolved = [r for r in rows if r["inner_px"] >= 45]
    if NEW_DETECTOR:
        # OpenCV >= 4.7 keeps one marker of a nested pair: the inner once it is a candidate at all.
        check(all(r["default"] in ([228], [11]) for r in full),
              "the whole image decodes as exactly one of 228 / 11 at every scale (defaults)")
        check(all(r["default"] == [228] for r in full if r["inner_px"] < 20),
              "outer 228 decodes from the whole image while the inner is too small to be a candidate")
    else:
        check(all(228 in r["default"] for r in full),
              "outer 228 decodes from the whole image at every scale (defaults)")
    check(all(11 in r["default"] for r in rows if r["view"] == "inner+cell"),
          "inner 11 decodes at close range, cropped to its white cell (defaults)")
    check(all(11 in r["rate_0.04"] for r in resolved),
          "inner 11 decodes wherever it is 45 px or more, with minMarkerDistanceRate 0.04")
    dropped = [f"{r['view']}@{r['px']}" for r in resolved if 11 not in r["default"]]
    report["inner_dropped_with_defaults"] = dropped
    print(f"dock_marker_smoke: inner 11 NOT decoded with OpenCV defaults at: {', '.join(dropped) or 'none'}")


# --- 2. flight --------------------------------------------------------------------------------


class FakeSitl(threading.Thread):
    """ArduPilot's side of the JSON lockstep: one servo frame out, one state back, next frame. All
    four motors on one PWM from an altitude hold, the gimbal pitch output (10) at -90 deg."""

    MASS, K_THRUST, OMEGA_MAX, G = 1.5, 7.65e-6, 1200.0, 9.81

    def __init__(self, port: int) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)
        self.target = 0.0
        self.state: dict | None = None
        self.lock = threading.Lock()
        self.done = threading.Event()
        self.integral = 0.0

    def pwm(self) -> int:
        if self.state is None or self.target <= 0.0:
            return 1000
        h = -self.state["position"][2]
        vz = -self.state["velocity"][2]
        v_cmd = max(-2.5, min(2.5, 1.2 * (self.target - h)))
        err = v_cmd - vz
        self.integral = max(-2.0, min(2.0, self.integral + err / 800.0))
        acc = max(-3.0, min(3.0, 2.5 * err + 0.8 * self.integral))
        thrust = max(self.MASS * (self.G + acc), 0.0)
        u = math.sqrt(thrust / (4.0 * self.K_THRUST)) / self.OMEGA_MAX
        return round(1000 + 1000 * max(0.0, min(1.0, u)))

    def run(self) -> None:
        fc = 0
        while not self.done.is_set():
            motor = self.pwm()
            channels = [motor] * 4 + [1000] * 5 + [1100, 1500] + [1000] * 5
            self.sock.sendto(struct.pack("<HHI16H", 18458, 800, fc, *channels), ("127.0.0.1", self.port))
            try:
                reply = self.sock.recv(4096)
            except TimeoutError:
                continue  # the same frame again: skysim re-answers a duplicate
            fc += 1
            try:
                state = json.loads(reply.decode())
            except ValueError:
                continue
            with self.lock:
                self.state = state

    def snapshot(self) -> dict | None:
        with self.lock:
            return dict(self.state) if self.state else None

    def hold(self, target: float, timeout: float = 90.0) -> bool:
        """Fly to `target` metres up and wait until settled there."""
        self.target = target
        deadline = time.time() + timeout
        steady_since = None
        while time.time() < deadline:
            s = self.snapshot()
            if s is not None:
                h, vz = -s["position"][2], -s["velocity"][2]
                if abs(h - target) < 0.01 and abs(vz) < 0.01:
                    steady_since = steady_since or time.time()
                    if time.time() - steady_since > 1.0:
                        return True
                else:
                    steady_since = None
            time.sleep(0.05)
        return False


def grab_rtsp(url: str, out_dir: str, frames: int) -> list[str]:
    """`frames` frames of the stream as PNGs, in a directory emptied first so a failed pull can
    never pass on an earlier run's frames; none when ffmpeg fails."""
    shutil.rmtree(out_dir, ignore_errors=True)
    os.makedirs(out_dir)
    pattern = os.path.join(out_dir, "f_%02d.png")
    rc = subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-rtsp_transport", "tcp", "-i", url,
                         "-frames:v", str(frames), "-start_number", "0", pattern],
                        capture_output=True, timeout=120, check=False)
    if rc.returncode != 0:
        print(f"dock_marker_smoke: ffmpeg failed ({rc.returncode}): {rc.stderr.decode()[-300:]}")
        return []
    return sorted(os.path.join(out_dir, f) for f in os.listdir(out_dir) if f.startswith("f_"))


def still(port: int, path: str) -> tuple[bytes, dict]:
    with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=60) as resp:
        return resp.read(), {k.lower(): v for k, v in resp.headers.items()}


def summarise(results: list[dict], rate: str, mid: str) -> dict:
    hits = [r[rate][mid] for r in results if r[rate][mid]["found"]]
    out = {"frames": len(results), "found": len(hits),
           "missed": [i for i, r in enumerate(results) if not r[rate][mid]["found"]]}
    if hits:
        out["within"] = sum(1 for h in hits if h["within"])
        for key in ("size_err_pct", "lateral_err_m", "ippe_lateral_err_m"):
            out[f"max_{key}"] = max(h[key] for h in hits)
        out["side_px"] = round(sum(h["side_px"] for h in hits) / len(hits), 1)
    return out


def geometry_err(results: list[dict], mid: str) -> float | None:
    """The worst error over the frames that resolve the square; inf when one of them should
    measure it and cannot."""
    errs = [r["geometry"][mid]["err_px"] for r in results if mid in r["geometry"]]
    if any(e is None for e in errs):
        return math.inf
    return max(abs(e) for e in errs) if errs else None


def framed(entry: dict, rate: str, mid: str, view_geometry: dict) -> bool:
    """Whether cv2.aruco's quad is the black square itself (its side within 2 %, or the 1.5 px
    its integer corners can lose, of the projected one) rather than the outline of the 2 cm white
    border around it (the second finding)."""
    g = view_geometry.get(mid)
    if not (entry[rate][mid]["found"] and g):
        return False
    return bool(abs(entry[rate][mid]["side_px"] - g["projected_px"]) <= max(0.02 * g["projected_px"], 1.5))


def case_flight(sim: str, frames_dir: str, threads: int, bitrate_kbps: int = 0) -> dict:
    """Fly the heights and capture: per height a still.jpg and RTSP frames, with the pose they
    were drawn from and the dock's ground truth, into `frames_dir` and its manifest.json. What
    needs skysim live (the gimbal, the drift, the texture log) is checked here; what the detector
    makes of the frames is analyse()'s, which can also run on a saved manifest with another OpenCV."""
    log_path = os.path.join(frames_dir, "skysim.log")
    # A manifest left from an earlier run must not outlive a failed one: --analyse would read it.
    manifest_path = os.path.join(frames_dir, "manifest.json")
    if os.path.exists(manifest_path):
        os.remove(manifest_path)
    cmd = [sim, "--vehicles", "0", "--base-instance", str(INSTANCE), "--time-mode", "interactive",
           "--api-port", str(API_PORT), "--rtsp-port", str(RTSP_PORT), "--camera-fps", "10",
           "--camera-size", f"{WIDTH}x{HEIGHT}", "--camera-fov", str(HFOV), "--straggler-timeout", "10"]
    if threads > 0:
        cmd += ["--camera-threads", str(threads), "--still-threads", str(threads), "--video-threads", "1"]
    manifest: dict = {"camera": {"width": WIDTH, "height": HEIGHT, "hfov_deg": HFOV}, "mode": MODE,
                      "heights_m": HEIGHTS, "rtsp_frames": RTSP_FRAMES, "heights": []}
    with open(log_path, "w") as log:
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
    sitl = None
    try:
        check(wait_up(API_PORT, proc), "skysim up with a 1920x1080 HFOV 81 camera and RTSP")
        api(API_PORT, "POST", "/vehicles", json.dumps({"instance": INSTANCE}).encode())
        check(len(listed(API_PORT, 1)) == 1, "vehicle spawned on its dock")
        intr = api(API_PORT, "GET", f"/instances/{INSTANCE}/camera/intrinsics")
        f = WIDTH / 2 / math.tan(math.radians(HFOV / 2))
        check(intr["width"] == WIDTH and abs(intr["fx"] - f) < 1e-6 and intr["cx"] == WIDTH / 2 - 0.5,
              f"intrinsics published: fx=fy={intr['fx']:.3f} cx={intr['cx']} cy={intr['cy']}, zero distortion")
        dock = api(API_PORT, "GET", f"/instances/{INSTANCE}/camera/dock")
        check(dock["source"] == "home" and dock["marker"]["outer_size_m"] == 1.0 and
              dock["marker"]["inner_size_m"] == 0.09, "dock at the vehicle's home: 1.00 m outer, 0.09 m inner")
        manifest["intrinsics"] = intr
        if bitrate_kbps > 0:
            enc = api(API_PORT, "POST", f"/instances/{INSTANCE}/camera/encoder",
                      json.dumps({"bitrate_kbps": bitrate_kbps}).encode())
            check(enc["bitrate_kbps"] == bitrate_kbps, f"RTSP bitrate set to {bitrate_kbps} kbit/s")
        manifest["rtsp_bitrate_kbps"] = api(API_PORT, "GET", f"/instances/{INSTANCE}/camera/encoder")["bitrate_kbps"]

        sitl = FakeSitl(9002 + 10 * INSTANCE)
        sitl.start()
        for height in HEIGHTS:
            check(sitl.hold(height), f"hovering at {height} m")
            gimbal = api(API_PORT, "GET", f"/instances/{INSTANCE}/camera/intrinsics")["gimbal"]
            check(gimbal["present"] and abs(gimbal["pitch_deg"] + 90.0) < 0.2,
                  f"camera follows the commanded gimbal pitch ({gimbal['pitch_deg']:.2f} deg) at {height} m")
            tag = f"{height:g}m".replace(".", "_")
            # The survey still camera, on the gimbal, at the pilot camera's size and lens.
            jpeg, headers = still(API_PORT, f"/instances/{INSTANCE}/still.jpg?w={WIDTH}&h={HEIGHT}&hfov={HFOV}"
                                            "&gimbal=1&q=95")
            with open(os.path.join(frames_dir, f"still_{tag}.jpg"), "wb") as fh:
                fh.write(jpeg)
            cam = [float(x) for x in headers["x-skysim-ned"].split(",")]
            angles = [float(headers[f"x-skysim-{a}"]) for a in ("yaw", "pitch", "roll")]
            check(abs(angles[1] + 90.0) < 0.2 and headers.get("x-skysim-gimbal") == "1",
                  f"still drawn on the gimbal, looking straight down (pitch {angles[1]:.2f}) at {height} m")
            dock = api(API_PORT, "GET", f"/instances/{INSTANCE}/camera/dock")
            # The H.264 RTSP stream the SITL core forwards: a run of frames. The camera's attitude
            # is the still's (the same gimbal, the same hover).
            before = dock["vehicle_pose"]["pos_ned"]
            files = grab_rtsp(f"rtsp://127.0.0.1:{RTSP_PORT}/instances/{INSTANCE}/main.264",
                              os.path.join(frames_dir, f"rtsp_{tag}"), RTSP_FRAMES)
            after = api(API_PORT, "GET", f"/instances/{INSTANCE}/camera/dock")["vehicle_pose"]["pos_ned"]
            drift = math.dist(before, after)
            check(len(files) == RTSP_FRAMES and drift < 0.01,
                  f"{len(files)} RTSP frames at {height} m (vehicle moved {drift * 1000:.1f} mm meanwhile)")
            manifest["heights"].append({
                "height_m": height, "dock": dock, "camera_angles_deg": angles,
                "still": {"file": f"still_{tag}.jpg", "camera_ned": cam},
                "rtsp": {"files": [os.path.relpath(p, frames_dir) for p in files], "camera_ned": after}})
        sitl.hold(0.0, timeout=1.0)
    finally:
        if sitl is not None:
            sitl.done.set()
            sitl.join(timeout=5)
        check(stop(proc) == 0, "flight skysim clean shutdown")
    with open(log_path) as fh:
        check("dock marker texture built" in fh.read(), "the marker's texture was built for the first frame")
    with open(manifest_path, "w") as fh:
        json.dump(manifest, fh, indent=1)
    return manifest


def complete(manifest: dict, frames_dir: str) -> bool:
    """Whether a capture is whole: the intrinsics, every height it set out to fly, and at each the
    still and all the RTSP frames it declared, on disk. A partial one fails the run rather than
    being analysed as far as it goes."""
    heights = manifest.get("heights", [])
    flown = [h.get("height_m") for h in heights]
    files_ok = all(
        os.path.isfile(os.path.join(frames_dir, h["still"]["file"])) and
        len(h["rtsp"]["files"]) == manifest.get("rtsp_frames") and
        all(os.path.isfile(os.path.join(frames_dir, f)) for f in h["rtsp"]["files"]) for h in heights)
    profile = PROFILES.get(manifest.get("mode", ""))
    declared_ok = profile is not None and list(profile[0]) == manifest.get("heights_m") and \
        profile[1] == manifest.get("rtsp_frames")
    whole = bool(manifest.get("intrinsics")) and declared_ok and flown == manifest.get("heights_m") and files_ok
    check(whole, f"the capture is whole and a {'/'.join(PROFILES)} one: mode {manifest.get('mode')}, "
                 f"heights {flown} of {manifest.get('heights_m')}, "
                 f"{manifest.get('rtsp_frames')} RTSP frames each, intrinsics {'present' if manifest.get('intrinsics') else 'missing'}")
    return whole


def analyse(manifest: dict, frames_dir: str, report: dict) -> None:
    """What the installed OpenCV makes of the captured frames, against the ground truth."""
    intr = manifest["intrinsics"]
    k = np.array(intr["camera_matrix"], dtype=np.float64)
    report["intrinsics"] = intr
    report["rtsp_bitrate_kbps"] = manifest.get("rtsp_bitrate_kbps")
    report["heights"] = []
    for h in manifest["heights"]:
        height = h["height_m"]
        dock, angles = h["dock"], h["camera_angles_deg"]
        entry: dict = {"height_m": height, "camera_ned": h["still"]["camera_ned"], "camera_angles_deg": angles}
        grey = cv2.imread(os.path.join(frames_dir, h["still"]["file"]), cv2.IMREAD_GRAYSCALE)
        entry["still"] = evaluate(grey, View(h["still"]["camera_ned"], angles, k), dock)
        rtsp_view = View(h["rtsp"]["camera_ned"], angles, k)
        rtsp = [evaluate(cv2.imread(os.path.join(frames_dir, f), cv2.IMREAD_GRAYSCALE), rtsp_view, dock)
                for f in h["rtsp"]["files"]]
        entry["rtsp"] = {rate: {mid: summarise(rtsp, rate, mid) for mid in ("228", "11")} for rate in RATES}
        entry["rtsp"]["geometry_max_err_px"] = {mid: geometry_err(rtsp, mid) for mid in ("228", "11")}
        for rate in RATES:
            for mid in ("228", "11"):
                key = f"framed_{rate}_{mid}"
                entry["rtsp"][key] = sum(1 for r in rtsp if framed(r, rate, mid, r["geometry"]))
                entry["still"][key] = framed(entry["still"], rate, mid, entry["still"]["geometry"])
        report["heights"].append(entry)

        # What skysim draws: each marker's black square, measured off the pixels, where the
        # published intrinsics and ground truth put it — within 2 px (or 1 %). Wherever the still
        # resolves a square, so does every RTSP frame (the same pose): a frame that cannot be
        # measured there fails rather than drops out.
        for mid in ("228", "11"):
            g = entry["still"]["geometry"].get(mid)
            if not g:
                continue
            limit = max(2.0, 0.01 * g["projected_px"])
            check(g["err_px"] is not None and abs(g["err_px"]) <= limit,
                  f"still {height} m: {mid}'s black square is {g['measured_px']} px, projected {g['projected_px']:.1f}")
            unmeasured = sum(1 for r in rtsp if mid not in r["geometry"] or r["geometry"][mid]["err_px"] is None)
            worst = entry["rtsp"]["geometry_max_err_px"][mid]
            check(unmeasured == 0 and worst is not None and worst <= limit + 1.0,
                  f"rtsp {height} m: {mid}'s black square within {worst} px of its projection in every frame "
                  f"({unmeasured} unmeasured)")
        # What the detector makes of it.
        for rate, mid in (("default", "228"), ("default", "11"), ("rate_0.04", "228"), ("rate_0.04", "11")):
            if height not in EXPECT[rate][mid]:
                continue
            st = entry["still"][rate][mid]
            rs = entry["rtsp"][rate][mid]
            # The still, and 9 RTSP frames in 10. H.264 at its default 1200 kbit/s for 1080p blurs a
            # cell out of a frame now and then — once in a CI run, the outer at 21.5 m (6 px a
            # cell) — and a landing loop reads the next frame. A marker under 30 px (2 to 3 px a
            # cell: the inner at 5 m) is at the stream's limit there: it must decode in the still
            # and in at least one RTSP frame, and how many is reported (7 of 10 locally, 1 of 3 in
            # CI's -O0 coverage run). Which frames missed is in the report.
            need = math.ceil(0.9 * rs["frames"]) if st.get("side_px", 0.0) >= 30.0 else 1
            check(st["found"] and rs["frames"] > 0 and rs["found"] >= need,
                  f"{height} m: {mid} decoded ({rate}) in the still and {rs['found']}/{rs['frames']} RTSP frames "
                  f"(need {need}; missed frames {rs.get('missed', [])})")
            if not st["found"] or rs["found"] == 0:
                continue
            # Each detection on its own (the still, then every RTSP frame): lateral within 10 cm, and
            # the scale within its own bound — 5 % where the detector framed the black square and
            # it is 30 px or more; 8 % where the quad frames the 2 cm white border's outline instead
            # (outer 228 on darker ground, the second finding); 12 % for a marker under 30 px (the
            # inner at 5 m, about 20 px), whose side cv2.aruco's integer corners (no corner
            # refinement by default) get a pixel or two wrong. A rendering change cannot hide in them.
            over = []
            for i, frame in enumerate([entry["still"], *rtsp]):
                d = frame[rate][mid]
                if not d["found"]:
                    continue
                bound = 12.0 if d["side_px"] < 30.0 else (5.0 if framed(frame, rate, mid, frame["geometry"]) else 8.0)
                where = "still" if i == 0 else f"rtsp frame {i - 1}"
                if d["lateral_err_m"] > POS_TOL_M:
                    over.append(f"{where}: lateral {d['lateral_err_m'] * 100:.1f} cm")
                if d["size_err_pct"] > bound:
                    over.append(f"{where}: size {d['size_err_pct']:.2f} % > {bound:g} %")
            check(not over, f"{height} m: {mid} ({rate}) lateral <= {rs['max_lateral_err_m'] * 100:.1f} cm (limit "
                            f"10 cm), size <= {max(st['size_err_pct'], rs['max_size_err_pct']):.2f} % within each "
                            f"detection's bound {over or ''}")
        for rate in RATES:
            print(f"dock_marker_smoke: {height:>5} m {rate:<9} still 228 {entry['still'][rate]['228']} "
                  f"11 {entry['still'][rate]['11']}")
            print(f"dock_marker_smoke: {height:>5} m {rate:<9} rtsp  228 {entry['rtsp'][rate]['228']} "
                  f"11 {entry['rtsp'][rate]['11']}")
        print(f"dock_marker_smoke: {height:>5} m geometry still {entry['still']['geometry']} rtsp max err "
              f"{entry['rtsp']['geometry_max_err_px']}; 228 framed (default): still "
              f"{entry['still']['framed_default_228']}, rtsp {entry['rtsp']['framed_default_228']}/{len(rtsp)}")


# --- 3. idle ----------------------------------------------------------------------------------


def case_idle(sim: str, tmp: str, report: dict) -> None:
    log_path = os.path.join(tmp, "idle.log")
    with open(log_path, "w") as log:
        proc = subprocess.Popen([sim, "--vehicles", "1", "--base-instance", str(IDLE_INSTANCE), "--time-mode",
                                 "interactive", "--dt", "0.0025", "--api-port", str(IDLE_API_PORT),
                                 "--rtsp-port", str(IDLE_RTSP_PORT), "--camera-fps", "15",
                                 "--camera-size", "640x360", "--dock-marker-size", "1.0",
                                 "--dock-inner-size", "0.09"], stdout=log, stderr=subprocess.STDOUT)
    try:
        check(wait_up(IDLE_API_PORT, proc), "idle skysim up")
        vehicle = listed(IDLE_API_PORT, 1)[0]
        time.sleep(1.0)
        cpu0, wall0 = process_cpu_s(proc.pid), time.monotonic()
        time.sleep(3.0)
        idle_pct = 100.0 * (process_cpu_s(proc.pid) - cpu0) / (time.monotonic() - wall0)
        report["idle_cpu_pct"] = round(idle_pct, 2)
        check(idle_pct < 25.0, f"camera on, a vehicle on its dock, nobody watching: {idle_pct:.1f}% of a core")
        with open(log_path) as fh:
            check("dock marker texture built" not in fh.read(), "the marker's texture is not built while idle")
        with urllib.request.urlopen(f"http://127.0.0.1:{IDLE_API_PORT}/vehicles/{vehicle['id']}/camera.jpg",
                                    timeout=30) as resp:
            check(resp.status == 200, "a viewer gets a frame")
        time.sleep(0.5)
    finally:
        check(stop(proc) == 0, "idle skysim clean shutdown")
    with open(log_path) as fh:
        log = fh.read()
    check("e-ArUco dock marker" in log, "startup says the dock markers are on")
    check("dock marker texture built" in log, "the texture is built once a camera draws the dock")


def case_args(sim: str) -> None:
    """The dock flags: sizes that cannot be drawn stop startup; --dock-markers 0 draws none and
    the dock route says so, while the intrinsics stay."""
    for bad in (["--dock-inner-size", "0.2"], ["--dock-marker-size", "1m"], ["--dock-marker-size", "50"],
                ["--dock-markers", "2"]):
        rc = subprocess.run([sim, "--vehicles", "0", *bad], capture_output=True, timeout=30, check=False)
        check(rc.returncode == 2 and b"dock" in rc.stderr, f"{' '.join(bad)} refused ({rc.stderr.decode().strip()})")
    proc = subprocess.Popen([sim, "--vehicles", "1", "--base-instance", str(OFF_INSTANCE), "--time-mode",
                             "interactive", "--api-port", str(OFF_API_PORT), "--rtsp-port", str(OFF_RTSP_PORT),
                             "--camera-fps", "5", "--dock-markers", "0"],
                            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    try:
        check(wait_up(OFF_API_PORT, proc), "skysim up with --dock-markers 0")
        listed(OFF_API_PORT, 1)
        try:
            api(OFF_API_PORT, "GET", f"/instances/{OFF_INSTANCE}/camera/dock")
            code = 200
        except urllib.error.HTTPError as e:
            code = e.code
        check(code == 404, "--dock-markers 0: GET .../camera/dock answers 404")
        intr = api(OFF_API_PORT, "GET", f"/instances/{OFF_INSTANCE}/camera/intrinsics")
        check(intr["width"] == 256 and intr["stream"]["width"] == 256, "the intrinsics stay without markers")
    finally:
        check(stop(proc) == 0, "--dock-markers 0 clean shutdown")


def cell(r: dict, frames: bool = False) -> str:
    """One table cell for a detection: found, size error, lateral error."""
    if frames:
        if not r["found"]:
            return f"0/{r['frames']}"
        return (f"{r['found']}/{r['frames']}, {r['max_size_err_pct']:.1f} %, "
                f"{r['max_lateral_err_m'] * 100:.1f} cm")
    if not r["found"]:
        return "no"
    return f"{r['side_px']:.0f} px, {r['size_err_pct']:.1f} %, {r['lateral_err_m'] * 100:.1f} cm"


def write_markdown(report: dict, path: str) -> None:
    intr = report.get("intrinsics", {})
    lines = [
        "# Dock marker proof (TASK-2562)",
        "",
        "Generated by `tests/dock_marker_smoke.py --report`. Do not edit by hand.",
        "",
        (f"OpenCV {report['opencv']} ({report['detector']}; `{report['opencv_module']}`), DICT_7X7_250, "
        "default DetectorParameters unless the column says otherwise. "
        f"Pilot camera {WIDTH}x{HEIGHT}, HFOV {HFOV:g} deg: "
        f"fx = fy = {intr.get('fx', 0):.3f}, cx = {intr.get('cx')}, cy = {intr.get('cy')}, zero distortion, as "
        "`GET /instances/<n>/camera/intrinsics` published it. The vehicle hovers straight above its dock with the "
        "gimbal at -90 deg. It is flown by a fake SITL in JSON lockstep."),
        "",
        ("In each cell: `found`, then the scale error (the marker size implied by the true distance and the one "
        "solvePnP found), then the lateral error (tvec across the optical axis, i.e. the landing error). RTSP "
        f"cells cover {RTSP_FRAMES} H.264 frames (at {report.get('rtsp_bitrate_kbps')} kbit/s) and give the worst of them. *Geometry* is detector-free: the "
        "black square's side measured off the pixels minus the side the published intrinsics and ground truth "
        "project, in px."),
        "",
        ("| height | geometry 228 / 11 (still) | 228 default, still | 228 default, RTSP | 11 default, still "
        "| 11 default, RTSP | 11 rate 0.04, still | 11 rate 0.04, RTSP |"),
        "|---|---|---|---|---|---|---|---|",
    ]
    for e in report.get("heights", []):
        g = e["still"]["geometry"]
        geo = " / ".join(f"{g[m]['err_px']:+.1f}" if m in g else "-" for m in ("228", "11"))
        lines.append(
            f"| {e['height_m']:g} m | {geo} | {cell(e['still']['default']['228'])} | "
            f"{cell(e['rtsp']['default']['228'], True)} | {cell(e['still']['default']['11'])} | "
            f"{cell(e['rtsp']['default']['11'], True)} | {cell(e['still']['rate_0.04']['11'])} | "
            f"{cell(e['rtsp']['rate_0.04']['11'], True)} |")
    lines += ["", "## The committed image", "", "| view | px | inner px | default | rate 0.04 |", "|---|---|---|---|---|"]
    for r in report.get("asset", []):
        lines.append(f"| {r['view']} | {r['px']} | {r['inner_px']} | {r['default']} | {r['rate_0.04']} |")
    lines += ["", (f"Idle CPU, with the camera on, one vehicle on its dock and nobody watching: "
              f"{report.get('idle_cpu_pct', 'n/a')} % of a core."), ""]
    with open(path, "w") as fh:
        fh.write("\n".join(lines))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("sim", nargs="?", default="./build/skysim")
    parser.add_argument("--frames", help="keep the captured frames and their manifest here")
    parser.add_argument("--analyse", metavar="DIR",
                        help="no skysim: analyse frames captured earlier with --frames (any OpenCV)")
    parser.add_argument("--report", help="write proof.json, proof.md and three frames here")
    parser.add_argument("--camera-threads", type=int, default=0, help="render threads (0: skysim's default)")
    parser.add_argument("--bitrate", type=int, default=0,
                        help="RTSP bitrate, kbit/s, set over the encoder route (0: skysim's default, 1200)")
    parser.add_argument("--quick", action="store_true",
                        help="30, 5 and 0.7 m and 3 RTSP frames each (the coverage build, at -O0)")
    args = parser.parse_args()
    if args.quick:
        global MODE, HEIGHTS, RTSP_FRAMES
        MODE = "quick"
    HEIGHTS, RTSP_FRAMES = list(PROFILES[MODE][0]), PROFILES[MODE][1]
    if not HAVE_CV or (not args.analyse and shutil.which("ffmpeg") is None):
        print("dock_marker_smoke: SKIP — needs OpenCV with cv2.aruco (python3-opencv) and ffmpeg")
        return 77
    report: dict = {"opencv": cv2.__version__, "opencv_module": cv2.__file__,
                    "detector": "cv2.aruco.ArucoDetector" if NEW_DETECTOR else "cv2.aruco.detectMarkers (legacy)",
                    "camera": {"width": WIDTH, "height": HEIGHT, "hfov_deg": HFOV},
                    "criteria": {"size_tol": SIZE_TOL, "lateral_tol_m": POS_TOL_M}, "expect": EXPECT}
    print(f"dock_marker_smoke: OpenCV {cv2.__version__} ({cv2.__file__}), {report['detector']}")
    if args.report:
        os.makedirs(args.report, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        case_asset(report)
        if args.analyse:
            frames_dir = args.analyse
            with open(os.path.join(frames_dir, "manifest.json")) as fh:
                manifest = json.load(fh)
        else:
            case_args(args.sim)
            case_idle(args.sim, tmp, report)
            frames_dir = args.frames or os.path.join(tmp, "frames")
            os.makedirs(frames_dir, exist_ok=True)
            manifest = case_flight(args.sim, frames_dir, args.camera_threads, args.bitrate)
        if complete(manifest, frames_dir):
            analyse(manifest, frames_dir, report)
        if args.report:
            for h in manifest.get("heights", []):
                tag = f"{h['height_m']:g}m".replace(".", "_")
                shutil.copy(os.path.join(frames_dir, h["still"]["file"]), args.report)
                if h["rtsp"]["files"]:
                    shutil.copy(os.path.join(frames_dir, h["rtsp"]["files"][-1]),
                                os.path.join(args.report, f"rtsp_{tag}.png"))
    if args.report:
        with open(os.path.join(args.report, "proof.json"), "w") as fh:
            json.dump(report, fh, indent=1)
        write_markdown(report, os.path.join(args.report, "proof.md"))
    if FAILURES:
        print(f"dock_marker_smoke: {len(FAILURES)} failure(s)")
        return 1
    print("dock_marker_smoke: all passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
