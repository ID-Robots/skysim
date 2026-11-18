#!/usr/bin/env python3
"""What the pilot camera costs, by picture size, frame rate and fleet size (TASK-2661).

For every camera setting (WxH@fps) and every fleet size it starts skysim, answers each vehicle's
JSON lockstep with a stand-in SITL, and measures two phases:

  idle    nobody watching: the simulation without a camera feed
  stream  one RTSP (H.264, TCP interleaved) receiver per vehicle, as skyhub_core's camera input
          and aruco_landing read it

and reports per run: skysim's CPU (all threads, from /proc) in cores, the cores the camera adds
over idle, the frames per second each receiver got against the target, the mean render and
H.264 encode time per frame skysim reports (/metrics), CPU-ms per delivered frame, and the cores
the target rate would need (CPU-ms per frame x target fps x vehicles), which is the number to
size a task with when the box ran out of CPU before the target.

--state flying (default) climbs every vehicle (constant PWM), so every frame is a new picture;
--state parked leaves the motors off, so every vehicle rests where it spawned and its picture
does not change. --gimbal-pitch -90 (default) points every camera straight down, as during a
dock approach; the ground is the most expensive thing to draw, so this is the worst case.
--extra passes further skysim arguments (e.g. the landing profile).

--cpus pins skysim (taskset), e.g. 0-3 for a 4-vCPU budget; the receivers run on other CPUs
(--viewer-cpus) so they do not take skysim's.

Usage:
  python3 tools/bench/camera_cost.py --sim build/skysim --tiles build/bench_city/tiles \\
      --configs 640x360@30,1280x720@30,1920x1080@10,1920x1080@15,1920x1080@30 --vehicles 1,4,8 \\
      --cpus 0-3 --threads 4 --seconds 10 --label after --json out.json --markdown out.md
"""
from __future__ import annotations

import argparse
import importlib
import json
import os
import select
import shlex
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request
from typing import Any

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
RtspClient = importlib.import_module("rtsp_client").RtspClient  # beside this file, not a package

# Outside every port and instance the tests and camera_bench.py use (see their headers).
API_PORT = 18700
RTSP_PORT = 18701
BASE_INSTANCE = 120  # udp 10202+: 10000-10099 is often some other service's (WebRTC)
DT = 0.0025


def api(path: str, body: dict | None = None, method: str = "GET") -> Any:
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(f"http://127.0.0.1:{API_PORT}{path}", data=data, method=method)
    with urllib.request.urlopen(req, timeout=15) as resp:
        return json.loads(resp.read().decode() or "{}")


def cpu_s(pid: int) -> float:
    with open(f"/proc/{pid}/stat") as f:
        fields = f.read().rsplit(")", 1)[1].split()
    return (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")


def gimbal_pwm(pitch_deg: float) -> int:
    """The servo pulse that puts skysim's gimbal at `pitch_deg` (-90..20 over 1100..1900 us)."""
    return round(1100 + (min(max(pitch_deg, -90.0), 20.0) + 90.0) / 110.0 * 800.0)


class FakeSitl(threading.Thread):
    """Answers every vehicle's JSON lockstep: one servo frame out per state reply in."""

    def __init__(self, instances: list[int], pwm: list[int], gimbal_pitch: float | None) -> None:
        super().__init__(daemon=True)
        self.socks = []
        for i in instances:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("127.0.0.1", 9002 + 10 * i))
            self.socks.append(s)
        self.rate = round(1 / DT)
        self.pwm = []
        for motor in pwm:  # one motor PWM per vehicle
            frame = [motor] * 4 + [1000] * 12
            if gimbal_pitch is not None:
                frame[9] = gimbal_pwm(gimbal_pitch)  # servo 10: gimbal pitch
                frame[10] = 1500  # servo 11: gimbal yaw, along the nose
            self.pwm.append(frame)
        self.count = [0] * len(self.socks)
        self.stop = False
        self.error: str | None = None

    def send(self, k: int) -> None:
        self.count[k] += 1
        self.socks[k].send(struct.pack("<HHI16H", 18458, self.rate, self.count[k], *self.pwm[k]))

    def run(self) -> None:
        try:
            self.lockstep()
        except Exception as e:  # noqa: BLE001 - kept, so a run whose vehicles never flew is not reported
            self.error = f"{type(e).__name__}: {e}"

    def lockstep(self) -> None:
        last = [0.0] * len(self.socks)
        for k in range(len(self.socks)):
            self.send(k)
        while not self.stop:
            ready, _, _ = select.select(self.socks, [], [], 0.05)
            now = time.monotonic()
            for s in ready:
                k = self.socks.index(s)
                try:
                    s.recv(4096)
                except OSError:
                    continue
                last[k] = now
                self.send(k)
            for k in range(len(self.socks)):
                if now - last[k] > 0.2:  # a lost datagram: say it again
                    last[k] = now
                    self.send(k)


class Receiver(threading.Thread):
    """Plays one instance's RTSP stream and counts whole frames (RTP marker bits), keeping nothing."""

    def __init__(self, instance: int) -> None:
        super().__init__(daemon=True)
        self.url = f"rtsp://127.0.0.1:{RTSP_PORT}/instances/{instance}/main.264"
        self.stop = False
        self.error: str | None = None
        self.frames = 0
        self.bytes = 0

    def run(self) -> None:
        try:
            client = RtspClient(self.url)
            if client.describe().status != 200 or client.play().status != 200:
                raise RuntimeError(f"{self.url}: DESCRIBE/SETUP/PLAY refused")
            sock = client.sock
            buf = bytearray(client.buf)
            sock.settimeout(1.0)
            while not self.stop:
                try:
                    data = sock.recv(1 << 18)
                except TimeoutError:
                    continue
                if not data:
                    raise RuntimeError(f"{self.url}: the server closed the stream")
                buf += data
                pos = 0
                while len(buf) - pos >= 4 and buf[pos] == 0x24:  # '$' interleaved packet
                    length = struct.unpack_from(">H", buf, pos + 2)[0]
                    if len(buf) - pos < 4 + length:
                        break
                    if buf[pos + 1] == 0 and length >= 12:
                        self.bytes += length - 12
                        if buf[pos + 5] & 0x80:
                            self.frames += 1
                    pos += 4 + length
                if pos < len(buf) and buf[pos] != 0x24:
                    end = buf.find(b"\r\n\r\n", pos)  # an RTSP reply in between (none expected)
                    pos = len(buf) if end < 0 else end + 4
                del buf[:pos]
            client.close()
        except Exception as e:  # noqa: BLE001 - any failure invalidates the run
            self.error = f"{type(e).__name__}: {e}"


def parse_config(text: str) -> tuple[str, float]:
    size, _, fps = text.partition("@")
    w, _, h = size.partition("x")
    if not (w.isdigit() and h.isdigit() and fps):
        raise argparse.ArgumentTypeError(f"bad camera setting '{text}' (want WxH@fps)")
    return size, float(fps)


def run_one(args: argparse.Namespace, size: str, fps: float, vehicles: int) -> dict:
    for port in (API_PORT, RTSP_PORT):
        with socket.socket() as probe:
            if probe.connect_ex(("127.0.0.1", port)) == 0:
                raise RuntimeError(f"port {port} is already in use; stop whatever holds it")
    cmd = [args.sim, "--vehicles", str(vehicles), "--base-instance", str(BASE_INSTANCE), "--time-mode",
           "interactive", "--dt", str(DT), "--api-port", str(API_PORT), "--camera-fps", f"{fps:g}",
           "--camera-size", size, "--camera-fov", "81", "--rtsp-port", str(RTSP_PORT), "--rtsp-bind",
           "127.0.0.1", "--video-threads", str(args.video_threads)]
    if args.threads > 0:
        cmd += ["--camera-threads", str(args.threads)]
    if args.tiles:
        cmd += ["--tiles", args.tiles]
    cmd += shlex.split(args.extra)
    if args.cpus:
        cmd = ["taskset", "-c", args.cpus, *cmd]
    with tempfile.NamedTemporaryFile(prefix="camera_cost_", suffix=".log") as log:
        sim = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
        sitl: FakeSitl | None = None
        receivers: list[Receiver] = []
        try:
            bound = (f"control plane on http://127.0.0.1:{API_PORT}", f"on rtsp://127.0.0.1:{RTSP_PORT}/")
            deadline = time.monotonic() + 30
            while True:
                if sim.poll() is not None:
                    with open(log.name, errors="replace") as f:
                        raise RuntimeError(f"skysim exited during startup (rc {sim.returncode}): {f.read()[-800:]}")
                with open(log.name, errors="replace") as f:
                    text = f.read()
                if all(b in text for b in bound):
                    break
                if time.monotonic() > deadline:
                    with open(log.name, errors="replace") as f:
                        raise RuntimeError(f"skysim did not report binding its API and RTSP ports: {f.read()[-800:]}")
                time.sleep(0.1)
            while len(api("/vehicles")) < vehicles:  # spawned at startup, listed a tick later
                if time.monotonic() > deadline:
                    raise RuntimeError("the fleet did not appear in GET /vehicles")
                time.sleep(0.1)
            instances = sorted(v["instance"] for v in api("/vehicles"))
            # --state parked: every vehicle rests on its dock; --parked N: the last N do, the rest fly.
            parked = len(instances) if args.state == "parked" else min(args.parked, len(instances))
            motors = [args.pwm] * (len(instances) - parked) + [1000] * parked
            sitl = FakeSitl(instances, motors, args.gimbal_pitch)
            # --landing-vehicles K: the landing camera (in --extra) for the first K vehicles only.
            if args.landing_vehicles >= 0:
                for i in instances[args.landing_vehicles:]:
                    api(f"/instances/{i}/camera", {"landing": False}, "POST")
            sitl.start()
            time.sleep(args.settle)

            c0, t0 = cpu_s(sim.pid), time.monotonic()
            time.sleep(args.idle_seconds)
            idle_cores = (cpu_s(sim.pid) - c0) / (time.monotonic() - t0)

            receivers = [Receiver(i) for i in instances]
            for r in receivers:
                r.start()
            # Every receiver playing before the clock starts: one still in PLAY would count its
            # start-up as lost frames.
            deadline = time.monotonic() + 20
            while any(r.frames == 0 for r in receivers):
                failed = [r.error for r in receivers if r.error]
                if failed or time.monotonic() > deadline:
                    raise RuntimeError(f"receivers did not start: {failed or 'no frame within 20 s'}")
                time.sleep(0.05)
            time.sleep(args.warmup)
            m0 = api("/metrics")
            f0 = [r.frames for r in receivers]
            b0 = [r.bytes for r in receivers]
            c0, t0 = cpu_s(sim.pid), time.monotonic()
            time.sleep(args.seconds)
            c1, t1 = cpu_s(sim.pid), time.monotonic()
            f1 = [r.frames for r in receivers]
            b1 = [r.bytes for r in receivers]
            m1 = api("/metrics")
            failed = [r.error for r in receivers if r.error]
            if sitl.error:
                failed.append(f"stand-in SITL: {sitl.error}")
            if failed:
                raise RuntimeError(f"{len(failed)} receiver(s) failed: {failed}")
        finally:
            for r in receivers:
                r.stop = True
            for r in receivers:
                r.join(timeout=5)
            if sitl is not None:
                sitl.stop = True
            sim.send_signal(signal.SIGINT)
            try:
                sim.wait(timeout=20)
            except subprocess.TimeoutExpired:
                sim.kill()
                sim.wait()
    secs = t1 - t0
    uniform = (parked in (0, vehicles) and args.landing_vehicles < 0
               and not any(flag in args.extra for flag in ("--camera-landing", "--camera-rest-fps")))
    cores = (c1 - c0) / secs
    camera_cores = max(cores - idle_cores, 0.0)
    rates = [(b - a) / secs for a, b in zip(f0, f1)]
    delivered = sum(rates)
    cam0, cam1 = m0.get("camera") or {}, m1.get("camera") or {}
    drawn = cam1.get("frames_drawn", 0) - cam0.get("frames_drawn", 0)
    reused = cam1.get("frames_reused", 0) - cam0.get("frames_reused", 0)
    cpu_ms_per_frame = camera_cores * 1000.0 / delivered if delivered > 0 else None
    return {
        "label": args.label,
        "size": size,
        "fps": fps,
        "vehicles": vehicles,
        "state": "parked" if parked == vehicles else ("flying" if parked == 0 else
                                                      f"{vehicles - parked} flying, {parked} parked"),
        "landing_now": (m1.get("camera") or {}).get("landing"),
        "cores": round(cores, 2),
        "idle_cores": round(idle_cores, 2),
        "camera_cores": round(camera_cores, 2),
        "fps_each": round(sum(rates) / len(rates), 2) if rates else 0.0,
        "fps_min": round(min(rates), 2) if rates else 0.0,
        "drawn_per_s": round(drawn / secs, 2),
        "reused_per_s": round(reused / secs, 2),
        "render_ms": round(cam1.get("render_ms", 0.0), 1),
        "render_ms_max": round(cam1.get("render_ms_max", 0.0), 1),
        "h264_ms": round((m1.get("video") or {}).get("encode_ms", 0.0), 2),
        "cpu_ms_per_frame": round(cpu_ms_per_frame, 1) if cpu_ms_per_frame is not None else None,
        # Only where every vehicle was asked for `fps`: a fleet that mixes rates (parked at the rest
        # rate, landing at the landing rate) has no one target to scale to.
        "cores_for_target": round(cpu_ms_per_frame * fps * vehicles / 1000.0, 2)
        if cpu_ms_per_frame is not None and uniform
        else None,
        "tick_p99_us": m1.get("tick_p99_us"),
        "freezes": m1.get("freezes", 0) - m0.get("freezes", 0),
        "kbps_each": round(sum(b - a for a, b in zip(b0, b1)) * 8 / 1000 / max(secs, 1e-9)
                           / max(len(receivers), 1), 1),
    }


def markdown(rows: list[dict]) -> str:
    head = ("| label | camera | vehicles | state | fps each (min) | render ms/frame | H.264 ms/frame | "
            "camera cores | CPU-ms per frame | cores for target | tick p99 us |\n"
            "|---|---|---|---|---|---|---|---|---|---|---|\n")
    lines = []
    for r in rows:
        if "error" in r:
            lines.append(f"| {r['label']} | {r['size']}@{r['fps']:g} | {r['vehicles']} | {r.get('state', '')} | "
                         f"failed: {r['error']} | | | | | | |")
            continue
        lines.append(
            f"| {r['label']} | {r['size']}@{r['fps']:g} | {r['vehicles']} | {r['state']} | {r['fps_each']} "
            f"({r['fps_min']}) | {r['render_ms']} | {r['h264_ms']} | {r['camera_cores']} | "
            f"{r['cpu_ms_per_frame']} | {r['cores_for_target']} | {r['tick_p99_us']} |")
    return head + "\n".join(lines) + "\n"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", default="./build/skysim")
    ap.add_argument("--tiles", default="", help="a cooked world (tile_cooker output); none = ground only")
    ap.add_argument("--configs", default="640x360@30,1280x720@30,1920x1080@10,1920x1080@15,1920x1080@30")
    ap.add_argument("--vehicles", default="1,4,8")
    ap.add_argument("--state", choices=("flying", "parked"), default="flying")
    ap.add_argument("--parked", type=int, default=0, help="with --state flying: the last N vehicles stay parked")
    ap.add_argument("--landing-vehicles", type=int, default=-1,
                    help="the landing camera stays on for the first K vehicles only (-1: all)")
    ap.add_argument("--pwm", type=int, default=1600, help="motor PWM while flying (1000..2000 us)")
    ap.add_argument("--gimbal-pitch", type=float, default=-90.0, help="degrees; -999 = no gimbal (fixed mount)")
    ap.add_argument("--threads", type=int, default=0, help="--camera-threads (0 = skysim's default)")
    ap.add_argument("--video-threads", type=int, default=2)
    ap.add_argument("--cpus", default="", help="taskset CPU list for skysim, e.g. 0-3")
    ap.add_argument("--viewer-cpus", default="", help="CPU list for this script's receivers, e.g. 4,5")
    ap.add_argument("--settle", type=float, default=2.0)
    ap.add_argument("--idle-seconds", type=float, default=3.0)
    ap.add_argument("--warmup", type=float, default=3.0)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--extra", default="", help="more skysim arguments, one string")
    ap.add_argument("--label", default="")
    ap.add_argument("--json", default="")
    ap.add_argument("--markdown", default="")
    args = ap.parse_args()
    if not 1000 <= args.pwm <= 2000:
        ap.error("--pwm must be 1000..2000")
    if args.gimbal_pitch <= -999:
        args.gimbal_pitch = None
    if args.viewer_cpus:
        cpus: set[int] = set()
        for part in args.viewer_cpus.split(","):
            lo, _, hi = part.partition("-")
            cpus.update(range(int(lo), int(hi or lo) + 1))
        os.sched_setaffinity(0, cpus)
    configs = [parse_config(c) for c in args.configs.split(",") if c]
    fleet = [int(v) for v in args.vehicles.split(",") if v]
    rows: list[dict] = []
    for size, fps in configs:
        for n in fleet:
            try:
                row = run_one(args, size, fps, n)
            except Exception as e:  # noqa: BLE001 - one failed run is reported, the rest still run
                row = {"label": args.label, "size": size, "fps": fps, "vehicles": n, "state": args.state,
                       "error": str(e)}
            print(json.dumps(row), flush=True)
            rows.append(row)
    if args.json:
        with open(args.json, "w") as f:
            json.dump({"sim": args.sim, "tiles": args.tiles, "cpus": args.cpus, "threads": args.threads,
                       "gimbal_pitch": args.gimbal_pitch, "seconds": args.seconds, "rows": rows}, f, indent=2)
    table = markdown(rows)
    if args.markdown:
        with open(args.markdown, "w") as f:
            f.write(table)
    print(table)
    return 1 if any("error" in r for r in rows) else 0


if __name__ == "__main__":
    sys.exit(main())
