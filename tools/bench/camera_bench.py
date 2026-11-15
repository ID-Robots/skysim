#!/usr/bin/env python3
"""Camera feed cost and latency: MJPEG (camera.mjpg) against H.264 over RTSP (TASK-2561).

Starts skysim with N vehicles and the camera on, flies every vehicle with a stand-in SITL (the
JSON lockstep answered with a constant PWM, so each vehicle climbs and every frame is a new
picture to draw and encode), then runs three phases of --seconds each:

  idle   nobody watching: what the simulation costs without a camera feed
  mjpeg  one camera.mjpg viewer per vehicle
  h264   one RTSP (TCP interleaved) viewer per vehicle

and reports, per phase, skysim's CPU (all threads, from /proc), the extra CPU per stream over
idle, frames/s and kbit/s each viewer got, the per-frame encode time skysim reports, and frame
latency: the time from the picture being ready (before either encoder runs) to the viewer
having all of it — from X-Skysim-Capture-Unix-Usec for MJPEG, from RTCP sender reports for RTP.

--transcode adds the pipeline the SITL core ran before: an ffmpeg per vehicle decoding the
MJPEG and re-encoding it to H.264 (libx264, baseline, zerolatency, 1200 kbit/s), whose CPU is
the cost the direct H.264 stream removes. It is a stand-in for the core's own transcoder.

--cpus pins skysim (taskset), e.g. 0,1 for a 2-vCPU Fargate task.

Usage: python3 tools/bench/camera_bench.py --sim build/skysim [--vehicles 4] [--size 256x144]
       [--fps 10] [--seconds 15] [--cpus 0,1] [--transcode] [--json out.json]
"""
from __future__ import annotations

import argparse
import contextlib
import http.client
import importlib
import json
import os
import select
import shutil
import signal
import socket
import statistics
import struct
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
RtspClient = importlib.import_module("rtsp_client").RtspClient  # beside this file, not a package

API_PORT = 18680
RTSP_PORT = 18681
BASE_INSTANCE = 70


def api(path: str, body: bytes | None = None, method: str = "GET"):
    req = urllib.request.Request(f"http://127.0.0.1:{API_PORT}{path}", data=body, method=method)
    with urllib.request.urlopen(req, timeout=15) as resp:
        return json.loads(resp.read().decode() or "{}")


def cpu_s(pid: int) -> float:
    with open(f"/proc/{pid}/stat") as f:
        fields = f.read().rsplit(")", 1)[1].split()
    return (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")


class FakeSitl(threading.Thread):
    """Answers every vehicle's JSON lockstep: one servo frame out per state reply in."""

    def __init__(self, instances: list[int], rate_hz: int, pwm: int) -> None:
        super().__init__(daemon=True)
        self.socks = []
        for i in instances:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("127.0.0.1", 9002 + 10 * i))
            self.socks.append(s)
        self.rate = rate_hz
        self.pwm = [pwm] * 4 + [1000] * 12
        self.count = [0] * len(self.socks)
        self.stop = False
        self.replies = 0

    def send(self, k: int) -> None:
        self.count[k] += 1
        self.socks[k].send(struct.pack("<HHI16H", 18458, self.rate, self.count[k], *self.pwm))

    def run(self) -> None:
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
                self.replies += 1
                last[k] = now
                self.send(k)
            for k in range(len(self.socks)):
                if now - last[k] > 0.2:  # a lost datagram: say it again
                    last[k] = now
                    self.send(k)


class Viewer(threading.Thread):
    """A viewer thread whose failure is kept, so a phase it died in is not reported as measured."""

    def __init__(self) -> None:
        super().__init__(daemon=True)
        self.stop = False
        self.error: str | None = None
        self.frames: list[tuple[float, float, int]] = []  # (received, captured, bytes)

    def run(self) -> None:
        try:
            self.view()
        except Exception as e:  # noqa: BLE001 - any failure invalidates the phase
            self.error = f"{type(e).__name__}: {e}"

    def view(self) -> None:
        raise NotImplementedError


class MjpegViewer(Viewer):
    def __init__(self, instance: int) -> None:
        super().__init__()
        self.instance = instance

    def view(self) -> None:
        conn = http.client.HTTPConnection("127.0.0.1", API_PORT, timeout=10)
        conn.request("GET", f"/instances/{self.instance}/camera.mjpg")
        resp = conn.getresponse()
        # Through the response, not resp.fp: the stream is chunked, and the response strips the
        # chunk framing a raw read would leave inside the parts.
        while not self.stop:
            headers = {}
            line = resp.readline()
            if not line:
                break
            if not line.startswith(b"--skysimframe"):
                continue
            while True:
                line = resp.readline().strip()
                if not line:
                    break
                k, _, v = line.decode().partition(":")
                headers[k.strip().lower()] = v.strip()
            body = resp.read(int(headers.get("content-length", "0")))
            now = time.time()
            captured = int(headers.get("x-skysim-capture-unix-usec", "0")) / 1e6
            self.frames.append((now, captured, len(body)))
        conn.close()


class RtspViewer(Viewer):
    def __init__(self, instance: int) -> None:
        super().__init__()
        self.url = f"rtsp://127.0.0.1:{RTSP_PORT}/instances/{instance}/main.264"

    def view(self) -> None:
        client = RtspClient(self.url)
        try:
            if client.describe().status != 200 or client.play().status != 200:
                raise RuntimeError(f"{self.url}: DESCRIBE/SETUP/PLAY refused")
            seen = 0
            while not self.stop:
                client.read_for(0.25)
                for f in client.frames[seen:]:
                    captured = client.capture_time(f)
                    self.frames.append((f.received, captured or 0.0, f.size))
                seen = len(client.frames)
        finally:
            client.close()


def summarise(name: str, viewers: list, t0: float, t1: float, cpu: float, idle_cpu: float, n: int,
              encode_ms: float) -> dict:
    lat, fps, kbps = [], [], []
    for v in viewers:
        window = [f for f in v.frames if t0 <= f[0] <= t1]
        fps.append(len(window) / (t1 - t0))
        kbps.append(sum(f[2] for f in window) * 8 / 1000 / (t1 - t0))
        lat.extend((f[0] - f[1]) * 1e3 for f in window if f[1] > 0)
    lat.sort()
    row = {
        "phase": name,
        "cpu_pct": round(cpu, 1),
        "per_stream_pct": round((cpu - idle_cpu) / n, 1) if viewers else None,
        "fps": round(statistics.mean(fps), 2) if fps else None,
        "kbps": round(statistics.mean(kbps), 1) if kbps else None,
        "encode_ms": round(encode_ms, 2),
        "latency_ms_p50": round(lat[len(lat) // 2], 1) if lat else None,
        "latency_ms_p95": round(lat[int(len(lat) * 0.95)], 1) if lat else None,
        "latency_ms_max": round(lat[-1], 1) if lat else None,
    }
    return row


def measure_transcode(sim: subprocess.Popen, instances: list[int], seconds: float) -> dict:
    """The pipeline the SITL core ran: an ffmpeg per vehicle, MJPEG in, H.264 out."""
    with contextlib.ExitStack() as stack:
        procs: list[subprocess.Popen] = []
        logs = []
        for i in instances:
            logs.append(stack.enter_context(tempfile.TemporaryFile()))
            procs.append(stack.enter_context(subprocess.Popen(
                ["ffmpeg", "-loglevel", "error", "-seekable", "0", "-f", "mpjpeg", "-i",
                 f"http://127.0.0.1:{API_PORT}/instances/{i}/camera.mjpg", "-c:v", "libx264", "-preset",
                 "veryfast", "-tune", "zerolatency", "-profile:v", "baseline", "-b:v", "1200k", "-threads", "1",
                 "-f", "null", "-"], stdout=subprocess.DEVNULL, stderr=logs[-1])))

        def stop_all() -> None:  # runs before each Popen's own exit, which waits for it
            for p in procs:
                if p.poll() is None:
                    p.send_signal(signal.SIGINT)
            for p in procs:
                try:
                    p.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    p.kill()

        stack.callback(stop_all)

        def check_transcoders(when: str) -> None:
            dead = [(k, p.returncode) for k, p in enumerate(procs) if p.poll() is not None]
            for k, rc in dead:
                logs[k].seek(0)
                print(f"camera_bench: transcoder {k} exited {rc} {when}: "
                      f"{logs[k].read().decode(errors='replace').strip()}", file=sys.stderr)
            if dead:
                raise RuntimeError(f"{len(dead)} transcoder(s) exited {when}")

        time.sleep(3.0)
        check_transcoders("during warm-up")
        f0 = sum(cpu_s(p.pid) for p in procs)
        c0, t0 = cpu_s(sim.pid), time.time()
        time.sleep(seconds)
        check_transcoders("while measured")
        f1 = sum(cpu_s(p.pid) for p in procs)
        c1, t1 = cpu_s(sim.pid), time.time()
    return {"phase": "mjpeg+core-transcode", "cpu_pct": round(100.0 * (c1 - c0) / (t1 - t0), 1),
            "transcode_cpu_pct": round(100.0 * (f1 - f0) / (t1 - t0), 1),
            "transcode_per_stream_pct": round(100.0 * (f1 - f0) / (t1 - t0) / len(instances), 1)}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", default="./build/skysim")
    ap.add_argument("--vehicles", type=int, default=4)
    ap.add_argument("--size", default="256x144")
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--cpus", default="")
    ap.add_argument("--pwm", type=int, default=1600)
    ap.add_argument("--transcode", action="store_true")
    ap.add_argument("--json", default="")
    args = ap.parse_args()

    dt = 0.0025
    # The ports must be ours: an answer from a simulator that is already running would be measured
    # as this one's.
    for port in (API_PORT, RTSP_PORT):
        with socket.socket() as probe:
            if probe.connect_ex(("127.0.0.1", port)) == 0:
                print(f"camera_bench: port {port} is already in use; stop whatever holds it", file=sys.stderr)
                return 2
    cmd = [args.sim, "--vehicles", str(args.vehicles), "--base-instance", str(BASE_INSTANCE), "--time-mode",
           "interactive", "--dt", str(dt), "--api-port", str(API_PORT), "--camera-fps", str(args.fps),
           "--camera-size", args.size, "--rtsp-port", str(RTSP_PORT), "--rtsp-bind", "127.0.0.1"]
    if args.cpus:
        cmd = ["taskset", "-c", args.cpus, *cmd]
    sim = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    results: list[dict] = []
    try:
        for _ in range(100):
            if sim.poll() is not None:
                raise RuntimeError(f"skysim exited during startup (rc {sim.returncode})")
            try:
                api("/metrics")
                break
            except OSError:
                time.sleep(0.1)
        vehicles = api("/vehicles")
        instances = [v["instance"] for v in vehicles]
        sitl = FakeSitl(instances, round(1 / dt), args.pwm)
        sitl.start()
        time.sleep(3.0)  # airborne, climbing

        def measure(name: str, viewers: list, idle_cpu: float) -> dict:
            for v in viewers:
                v.start()
            time.sleep(3.0)  # warm-up: first frames, encoders built
            m0 = api("/metrics")
            c0, t0 = cpu_s(sim.pid), time.time()
            time.sleep(args.seconds)
            c1, t1 = cpu_s(sim.pid), time.time()
            m1 = api("/metrics")
            for v in viewers:
                v.stop = True
            for v in viewers:
                v.join(timeout=5)
            failed = [v.error or "viewer did not stop" for v in viewers if v.error or v.is_alive()]
            if failed:
                raise RuntimeError(f"{name}: {len(failed)} viewer(s) failed: {failed}")
            cpu = 100.0 * (c1 - c0) / (t1 - t0)
            encode_ms = (m1["video"]["encode_ms"] if name == "h264" else m1["camera"]["encode_ms"]) if viewers else 0
            row = summarise(name, viewers, t0, t1, cpu, idle_cpu, len(instances), encode_ms)
            row["render_ms"] = m1["camera"]["render_ms"]
            row["tick_p99_us"] = m1["tick_p99_us"]
            row["freezes"] = m1["freezes"] - m0["freezes"]
            row["sim_rate"] = round((m1["sim_time_s"] - m0["sim_time_s"]) / (t1 - t0), 3)
            print(json.dumps(row), flush=True)
            time.sleep(4.0)  # the renderer's demand window lapses before the next phase
            return row

        idle = measure("idle", [], 0.0)
        results.append(idle)
        results.append(measure("mjpeg", [MjpegViewer(i) for i in instances], idle["cpu_pct"]))
        results.append(measure("h264", [RtspViewer(i) for i in instances], idle["cpu_pct"]))
        if args.transcode and shutil.which("ffmpeg"):
            row = measure_transcode(sim, instances, args.seconds)
            print(json.dumps(row), flush=True)
            results.append(row)
        sitl.stop = True
    finally:
        sim.send_signal(signal.SIGINT)
        try:
            sim.wait(timeout=20)
        except subprocess.TimeoutExpired:
            sim.kill()
    if args.json:
        with open(args.json, "w") as f:
            json.dump({"vehicles": args.vehicles, "size": args.size, "fps": args.fps, "cpus": args.cpus,
                       "seconds": args.seconds, "results": results}, f, indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
