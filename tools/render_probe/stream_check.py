#!/usr/bin/env python3
"""Does aruco_landing see the dock through the stream it really gets? (TASK-2661)

tools/render_probe/aruco_check.py looks at lossless renders. This looks at what a SITL's landing
consumer is handed: skysim's H.264 RTSP stream, decoded by OpenCV's VideoCapture (its FFmpeg
backend over TCP), with the camera arguments a deployment runs. It starts skysim with those
arguments, flies one vehicle straight up over its dock with an altitude hold (gimbal at -90 deg),
and at each height reads the stream for a few seconds and runs cv2.aruco (DICT_7X7_250, default
DetectorParameters) on every frame.

One VideoCapture stays open for the whole flight, from the ground up, as a consumer's would, and
the run lists every picture size it decoded: with a landing mode the stream must keep one size
(OpenCV's FFmpeg capture keeps returning the last frame of the old size when it changes).
--offset moves the dock that far east of the vehicle's climb, as when the landing takes over
within 15 m of home.

  python3 tools/render_probe/stream_check.py --sim build/skysim --label dev \\
      --args "--camera-fps 30 --camera-size 640x360 --camera-fov 78"
  python3 tools/render_probe/stream_check.py --sim build/skysim --label landing \\
      --args "--camera-fps 30 --camera-size 1920x1080 --camera-draw-size 640x360 --camera-fov 81 \\
              --camera-landing-fps 15 --camera-landing-bitrate 4000 --camera-rest-fps 2"
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shlex
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
import urllib.request

try:
    import cv2
except ImportError:  # pragma: no cover - reported, not crashed on
    cv2 = None

API_PORT = 18704
RTSP_PORT = 18705
INSTANCE = 110

# What holding an altitude means, for hold() to start a measurement and for every reply after it,
# until the measurement ends: telemetry that is no more than FRESH_S old (replies still coming),
# within HOLD_BAND_M of the height and moving at less than HOLD_SPEED_MS.
FRESH_S = 0.2
HOLD_BAND_M = 0.05
HOLD_SPEED_MS = 0.05


def api(path: str, body: dict | None = None, method: str = "GET"):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(f"http://127.0.0.1:{API_PORT}{path}", data=data, method=method)
    with urllib.request.urlopen(req, timeout=15) as resp:
        return json.loads(resp.read().decode() or "{}")


class FakeSitl(threading.Thread):
    """JSON lockstep with an altitude hold on all four motors and the gimbal at -90 deg (the
    altitude hold tests/dock_marker_smoke.py flies with)."""

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
        self.received = 0.0  # when the last state reply arrived (monotonic)
        self.window: dict | None = None  # what the replies since hold() returned looked like

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
                continue
            fc += 1
            try:
                state = json.loads(reply.decode())
            except ValueError:
                continue
            with self.lock:
                self.state = state
                now = time.monotonic()
                w = self.window
                if w is not None:
                    try:
                        w["gap"] = max(w["gap"], now - w["last"])
                        w["deviation"] = max(w["deviation"], abs(-state["position"][2] - w["target"]))
                        w["speed"] = max(w["speed"], abs(state["velocity"][2]))
                        w["replies"] += 1
                    except (KeyError, IndexError, TypeError):
                        w["malformed"] += 1  # a reply without a position or velocity is not telemetry
                    w["last"] = now
                self.received = now

    def height(self) -> float:
        with self.lock:
            return -self.state["position"][2] if self.state else 0.0

    def hold(self, target: float, timeout: float = 90.0) -> bool:
        self.target = target
        deadline = time.time() + timeout
        steady = None
        while time.time() < deadline:
            with self.lock:
                s = self.state
                fresh = time.monotonic() - self.received < FRESH_S  # replies still coming
            if (s is not None and fresh and abs(-s["position"][2] - target) < HOLD_BAND_M
                    and abs(s["velocity"][2]) < HOLD_SPEED_MS):
                steady = steady or time.time()
                if time.time() - steady > 1.0:
                    # Held — and the replies from here on are watched (end_window) until whoever
                    # measures has finished: a hold that was true when this returned says nothing
                    # about the seconds the stream is read for.
                    with self.lock:
                        self.window = {"target": target, "last": self.received, "gap": 0.0, "deviation": 0.0,
                                       "speed": 0.0, "replies": 0, "malformed": 0}
                    return True
            else:
                steady = None
            time.sleep(0.05)
        return False

    def end_window(self) -> list[str]:
        """Stops watching what hold() started to, and says why the hold did not last: nothing when
        telemetry kept coming (never a gap of FRESH_S, and one that is that fresh now), and the
        vehicle stayed inside the band and under the speed hold() asked for, the whole time."""
        with self.lock:
            w, self.window = self.window, None
            now = time.monotonic()
            last = self.received
        if w is None:
            return ["no hold was being watched"]
        gap = max(w["gap"], now - max(last, w["last"]))  # the silence since the last reply counts
        why = []
        if w["replies"] == 0:
            why.append("no telemetry at all")
        if gap >= FRESH_S:
            why.append(f"telemetry stale for {gap:.2f} s")
        if w["malformed"]:
            why.append(f"{w['malformed']} telemetry replies without a position")
        if w["deviation"] >= HOLD_BAND_M:
            why.append(f"{w['deviation']:.2f} m off the height")
        if w["speed"] >= HOLD_SPEED_MS:
            why.append(f"vertical speed {w['speed']:.2f} m/s")
        return why


class Reader(threading.Thread):
    """One VideoCapture on the stream for the whole run: the latest frame, and every frame's size."""

    def __init__(self, url: str) -> None:
        super().__init__(daemon=True)
        os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp")
        self.cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if not self.cap.isOpened():
            raise RuntimeError(f"OpenCV could not open {url}")
        self.lock = threading.Lock()
        self.frames: list = []  # (time, frame) while collecting
        self.collecting = False
        self.sizes: list[tuple[int, int]] = []
        self.stop = False
        self.errors = 0

    def run(self) -> None:
        while not self.stop:
            ok, frame = self.cap.read()
            if not ok:
                self.errors += 1
                time.sleep(0.05)
                continue
            size = (frame.shape[1], frame.shape[0])
            with self.lock:
                if not self.sizes or self.sizes[-1] != size:
                    self.sizes.append(size)
                if self.collecting:
                    self.frames.append((time.time(), frame))

    def collect(self, seconds: float) -> list:
        with self.lock:
            self.frames = []
            self.collecting = True
        time.sleep(seconds)
        with self.lock:
            self.collecting = False
            batch, self.frames = self.frames, []  # handed over, not kept: a 1080p batch is ~0.5 GB
            return batch


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", default="build/skysim")
    ap.add_argument("--args", default="--camera-fps 30 --camera-size 640x360 --camera-fov 78",
                    help="skysim camera arguments, one string")
    ap.add_argument("--heights", default="30,20,10,3")
    ap.add_argument("--seconds", type=float, default=3.0, help="stream read at each height")
    ap.add_argument("--label", default="")
    ap.add_argument("--json", default="")
    ap.add_argument("--save", default="", help="keep one decoded frame per height here")
    ap.add_argument("--offset", type=float, default=0.0, help="the dock this many metres east of the vehicle")
    args = ap.parse_args()
    if cv2 is None:
        print("stream_check: needs python3 OpenCV (cv2)", file=sys.stderr)
        return 2
    params = cv2.aruco.DetectorParameters_create() if hasattr(cv2.aruco, "DetectorParameters_create") \
        else cv2.aruco.DetectorParameters()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_250)
    if hasattr(cv2.aruco, "ArucoDetector"):
        det = cv2.aruco.ArucoDetector(dictionary, params)
        detect = lambda grey: det.detectMarkers(grey)[1]  # noqa: E731
    else:
        detect = lambda grey: cv2.aruco.detectMarkers(grey, dictionary, parameters=params)[1]  # noqa: E731

    cmd = [args.sim, "--vehicles", "0", "--base-instance", str(INSTANCE), "--time-mode", "interactive",
           "--api-port", str(API_PORT), "--rtsp-port", str(RTSP_PORT), "--rtsp-bind", "127.0.0.1",
           "--straggler-timeout", "10", *shlex.split(args.args)]
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    sitl = reader = None
    rows = []
    failures: list[str] = []
    try:
        deadline = time.time() + 20
        while True:
            try:
                api("/metrics")
                break
            except OSError:
                if time.time() > deadline:
                    raise
                time.sleep(0.1)
        api("/vehicles", {"instance": INSTANCE}, "POST")
        while not any(v["instance"] == INSTANCE for v in api("/vehicles")):
            time.sleep(0.05)
        if args.offset:
            api(f"/instances/{INSTANCE}/camera/dock", {"north_m": 0.0, "east_m": args.offset}, "POST")
        sitl = FakeSitl(9002 + 10 * INSTANCE)
        sitl.start()
        reader = Reader(f"rtsp://127.0.0.1:{RTSP_PORT}/instances/{INSTANCE}/main.264")
        reader.start()
        time.sleep(2.0)
        if not reader.sizes:
            raise RuntimeError("OpenCV decoded no frame from the stream")
        time.sleep(2.0)
        for h in [float(x) for x in args.heights.split(",") if x]:
            held = sitl.hold(h)
            if not held:
                # Not at the height asked for: nothing measured is labelled with it.
                print(f"stream_check: did not hold {h:g} m (at {sitl.height():.1f} m); not measured", flush=True)
                failures.append(f"hold {h:g} m")
                continue
            time.sleep(1.0)  # past the switch and its IDR
            cam = api(f"/instances/{INSTANCE}/camera")
            frames = reader.collect(args.seconds)
            # The hold must have lasted from hold() to here, the whole of the read: a vehicle that
            # drifted off the height, or telemetry that stopped (the last reply repeated is a
            # position nobody is measuring), makes the frames some other height's.
            left = sitl.end_window()
            if left:
                print(f"stream_check: did not stay at {h:g} m while the stream was read "
                      f"({'; '.join(left)}); not measured", flush=True)
                failures.append(f"hold {h:g} m lost while reading ({left[0]})")
                frames.clear()
                continue
            if not frames:
                failures.append(f"no decoded frames at {h:g} m")
            found = 0
            for _, frame in frames:
                ids = detect(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
                found += 1 if ids is not None and 228 in ids.ravel().tolist() else 0
            size = f"{frames[-1][1].shape[1]}x{frames[-1][1].shape[0]}" if frames else "-"
            count = len(frames)
            if args.save and frames:
                os.makedirs(args.save, exist_ok=True)
                name = f"{args.label or 'stream'}_{h:g}m" + (f"_east{args.offset:g}m" if args.offset else "")
                cv2.imwrite(os.path.join(args.save, name + ".png"), frames[-1][1])
            frames.clear()  # released before the next height's batch arrives
            row = {"label": args.label, "height_m": h, "offset_m": args.offset, "held": held,
                   "mode": cam["now"]["mode"],
                   "camera": f"{cam['now']['draw_width']}x{cam['now']['draw_height']}@{cam['now']['fps']:g}",
                   "decoded_size": size, "frames": count,
                   "fps": round(count / args.seconds, 1), "detected_228": found}
            rows.append(row)
            print(json.dumps(row), flush=True)
    finally:
        if reader is not None:
            reader.stop = True
            reader.join(timeout=5)
        if sitl is not None:
            sitl.done.set()
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            proc.kill()
    sizes = reader.sizes if reader else []
    print(f"stream_check: picture sizes OpenCV decoded, in order: {sizes}; read errors {reader.errors if reader else 0}")
    print(f"| {args.label} | height | camera now (mode, drawn at) | decoded | frames | 228 detected |")
    for r in rows:
        print(f"| | {r['height_m']:g} m | {r['mode']} {r['camera']} | {r['decoded_size']} | {r['frames']} | "
              f"{r['detected_228']} of {r['frames']} |")
    if args.json:
        with open(args.json, "w") as f:
            json.dump({"label": args.label, "args": args.args, "opencv": cv2.__version__, "sizes": sizes,
                       "rows": rows, "failures": failures}, f, indent=2)
    if failures:
        print(f"stream_check: not measured as asked: {', '.join(failures)}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
