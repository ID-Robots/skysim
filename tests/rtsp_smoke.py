#!/usr/bin/env python3
"""The camera as H.264 over RTSP, end to end (TASK-2561).

Starts skysim with two vehicles and the camera on, pulls each vehicle's stream with ffprobe — one
over TCP, one over UDP — and checks what a browser or a Janus H.264 room will be handed: codec
h264, profile Constrained Baseline, the camera's size and frame rate, a keyframe every second.
Then looks inside one stream (SPS/PPS on every IDR, profile-level-id 42e01f in the SDP), changes
its encoder over the control API, checks the camera goes idle again when nobody plays, that the
MJPEG route still works, and that --camera-mjpeg 0 / --rtsp-port 0 / a taken port behave.

Exits 77 (skipped) when ffprobe is not installed.

Usage: python3 tests/rtsp_smoke.py <path-to-skysim>
"""
from __future__ import annotations

import itertools
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request

TOOLS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools", "bench")

SIM = sys.argv[1] if len(sys.argv) > 1 else "./build/skysim"
API_PORT = 18662
RTSP_PORT = 18663
OFF_API_PORT = 18664
CLASH_API_PORT = 18665
BASE_INSTANCE = 60
FAILURES: list[str] = []


def check(cond: bool, what: str) -> None:
    print(f"rtsp_smoke: {'ok  ' if cond else 'FAIL'} {what}")
    if not cond:
        FAILURES.append(what)


def api(port: int, method: str, path: str, body: bytes | None = None):
    req = urllib.request.Request(f"http://127.0.0.1:{port}{path}", data=body, method=method)
    with urllib.request.urlopen(req, timeout=15) as resp:
        return json.loads(resp.read().decode() or "{}")


def api_status(port: int, path: str) -> int:
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=15) as resp:
            return resp.status
    except urllib.error.HTTPError as e:
        return e.code


def wait_up(port: int, proc: subprocess.Popen, seconds: float = 15.0) -> dict | None:
    deadline = time.time() + seconds
    while time.time() < deadline and proc.poll() is None:
        try:
            return api(port, "GET", "/metrics")
        except (urllib.error.URLError, ConnectionError):
            time.sleep(0.1)
    return None


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
        return proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        proc.kill()
        return -9


def process_cpu_s(pid: int) -> float:
    with open(f"/proc/{pid}/stat") as f:
        fields = f.read().rsplit(")", 1)[1].split()
    return (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")


def ffprobe(url: str, transport: str) -> dict:
    cmd = ["ffprobe", "-v", "error", "-rtsp_transport", transport, "-i", url, "-select_streams", "v:0",
           "-show_entries", ("stream=codec_name,profile,width,height,r_frame_rate,pix_fmt,has_b_frames:"
                             "packet=pts_time,flags"), "-read_intervals", "%+6", "-of", "json"]
    out = subprocess.run(cmd, capture_output=True, text=True, timeout=60, check=False)
    if out.returncode != 0:
        print(out.stderr)
    return json.loads(out.stdout or "{}")


def case_ffprobe(vehicle: dict, transport: str) -> None:
    url = f"rtsp://127.0.0.1:{RTSP_PORT}/instances/{vehicle['instance']}/main.264"
    info = ffprobe(url, transport)
    streams = info.get("streams", [])
    check(len(streams) == 1, f"{transport}: ffprobe found one video stream at {url}")
    if not streams:
        return
    s = streams[0]
    print(f"rtsp_smoke: {transport} stream: {s}")
    check(s.get("codec_name") == "h264", f"{transport}: codec h264")
    check(s.get("profile") == "Constrained Baseline", f"{transport}: profile Constrained Baseline ({s.get('profile')})")
    check(s.get("width") == 256 and s.get("height") == 144, f"{transport}: 256x144")
    # r_frame_rate is ffmpeg's guess from the first timestamps (an instrumented build gave 30000/1001
    # once); the signalled rate is the SPS VUI's, checked in test_video. Here: the rate frames arrive at.
    print(f"rtsp_smoke: {transport}: ffprobe guesses r_frame_rate {s.get('r_frame_rate')}")
    check(s.get("has_b_frames", 0) == 0, f"{transport}: no B-frames")
    check(s.get("pix_fmt") == "yuv420p", f"{transport}: yuv420p")
    packets = [p for p in info.get("packets", []) if p.get("pts_time") not in (None, "N/A")]
    times = [float(p["pts_time"]) for p in packets]
    keys = [i for i, p in enumerate(packets) if "K" in p.get("flags", "")]
    check(len(packets) >= 30, f"{transport}: {len(packets)} frames in ~6 s")
    if len(times) >= 2:
        fps = (len(times) - 1) / (times[-1] - times[0])
        check(8.0 <= fps <= 11.0, f"{transport}: frames arrive at {fps:.2f} fps")
    check(len(keys) >= 3, f"{transport}: {len(keys)} keyframes")
    if len(keys) >= 2:
        gaps_s = [times[b] - times[a] for a, b in itertools.pairwise(keys)]
        gaps_n = [b - a for a, b in itertools.pairwise(keys)]
        print(f"rtsp_smoke: {transport} keyframe gaps {gaps_n} frames, {[round(g, 3) for g in gaps_s]} s")
        # The interval is wall-clock (keyframe_interval_s), so it is checked in seconds: a runner
        # that draws fewer than 10 fps has fewer frames between keyframes, not a wrong GOP.
        check(all(0.85 <= g <= 1.25 for g in gaps_s), f"{transport}: a keyframe every second")


def case_inside(vehicle: dict) -> None:
    """SDP and bitstream as the core will see them, and the encoder API changing a live stream."""
    sys.path.insert(0, TOOLS)
    from rtsp_client import RtspClient

    instance = vehicle["instance"]
    url = f"rtsp://127.0.0.1:{RTSP_PORT}/instances/{instance}/main.264"
    applied = api(API_PORT, "POST", f"/instances/{instance}/camera/encoder",
                  json.dumps({"bitrate_kbps": 600, "keyframe_interval_s": 0.5}).encode())
    check(applied["bitrate_kbps"] == 600 and applied["keyframe_interval_s"] == 0.5 and applied["fps"] == 10,
          f"POST camera/encoder applied {applied}")
    clamped = api(API_PORT, "POST", f"/instances/{instance}/camera/encoder", json.dumps({"fps": 60}).encode())
    check(clamped["fps"] == 10 and clamped["max_fps"] == 10, "fps clamped to the camera rate")

    client = RtspClient(url)
    try:
        r = client.describe()
        check(r.status == 200, "DESCRIBE 200")
        check("profile-level-id=42e01f" in client.sdp, "SDP profile-level-id=42e01f")
        check("packetization-mode=1" in client.sdp and "sprop-parameter-sets=" in client.sdp,
              "SDP packetization-mode=1 with sprop-parameter-sets")
        r = client.play()
        check(r.status == 200, "SETUP + PLAY over TCP 200")
        check(client.read_frames(30, timeout=10), f"30 frames read ({len(client.frames)})")
        frames = client.frames
        idrs = [i for i, f in enumerate(frames) if f.idr]
        check(bool(frames) and frames[0].idr, "the first frame a receiver gets is an IDR")
        check(all({7, 8} <= frames[i].types() for i in idrs), "SPS and PPS on every IDR")
        check(all(not ({7, 8} & f.types()) for f in frames if not f.idr), "no SPS/PPS outside IDRs")
        gaps = [((frames[b].rtp_ts - frames[a].rtp_ts) % (1 << 32)) / 90000.0 for a, b in itertools.pairwise(idrs)]
        check(bool(gaps) and all(0.4 <= g <= 0.7 for g in gaps),
              f"keyframe_interval_s 0.5: an IDR every 0.5 s {[round(g, 3) for g in gaps]}")
        sps = next(n for n in frames[0].nals if n[0] & 0x1F == 7)
        check(sps[1:4].hex() == "42e01f", f"in-band SPS says {sps[1:4].hex()}")
        check(api(API_PORT, "GET", f"/instances/{instance}/camera/encoder")["clients"] == 1, "encoder shows 1 client")
        metrics = api(API_PORT, "GET", "/metrics")["video"]
        check(metrics["clients"] == 1 and metrics["streams"] == 1 and metrics["frames_encoded"] > 0,
              f"/metrics video while playing {metrics}")
        # A forced IDR arrives within a frame or two, ahead of the 0.5 s schedule.
        api(API_PORT, "POST", f"/instances/{instance}/camera/encoder",
            json.dumps({"keyframe_interval_s": 10}).encode())
        client.read_for(1.5)
        before = len(client.frames)
        reply = api(API_PORT, "POST", f"/instances/{instance}/camera/keyframe", b"")
        check(reply == {"keyframe": True, "clients": 1}, f"POST camera/keyframe {reply}")
        client.read_frames(before + 3, timeout=5)
        check(any(f.idr for f in client.frames[before:before + 3]), "forced keyframe within 3 frames")
        check(client.report is not None, "RTCP sender report received")
        latencies = [f.received - client.capture_time(f) for f in client.frames[-10:] if client.report]
        if latencies:
            print(f"rtsp_smoke: capture -> received {min(latencies) * 1e3:.1f}..{max(latencies) * 1e3:.1f} ms")
            check(all(-0.05 <= x < 0.5 for x in latencies), "frames arrive within 0.5 s of being drawn")
    finally:
        client.close()


def main() -> int:
    if shutil.which("ffprobe") is None:
        print("rtsp_smoke: ffprobe not installed, skipping")
        return 77
    tmp = tempfile.mkdtemp(prefix="rtsp_smoke_")
    log_path = os.path.join(tmp, "skysim.log")
    with open(log_path, "w") as log:
        proc = subprocess.Popen([SIM, "--vehicles", "2", "--base-instance", str(BASE_INSTANCE), "--time-mode",
                                 "interactive", "--dt", "0.0025", "--api-port", str(API_PORT), "--camera-fps", "10",
                                 "--camera-size", "256x144", "--rtsp-port", str(RTSP_PORT), "--rtsp-bind",
                                 "127.0.0.1"], stdout=log, stderr=subprocess.STDOUT)
    try:
        metrics = wait_up(API_PORT, proc)
        check(metrics is not None, "skysim up")
        if metrics is None:
            return 1
        check(metrics["video"] is not None and metrics["video"]["rtsp_port"] == RTSP_PORT,
              f"/metrics video block {metrics['video']}")
        vehicles = listed(API_PORT, 2)
        check(len(vehicles) == 2, "two vehicles")
        enc = api(API_PORT, "GET", f"/instances/{vehicles[0]['instance']}/camera/encoder")
        check(enc["bitrate_kbps"] == 1200 and enc["keyframe_interval_s"] == 1 and enc["fps"] == 10 and
              enc["width"] == 256 and enc["height"] == 144, f"default encoder settings {enc}")
        check(api_status(API_PORT, "/instances/59/camera/encoder") == 404, "no such instance -> 404")

        # A second skysim cannot take a port that is in use: it says so and exits 1.
        clash = subprocess.run([SIM, "--vehicles", "0", "--time-mode", "interactive", "--api-port",
                                str(CLASH_API_PORT), "--camera-fps", "10", "--rtsp-port", str(RTSP_PORT),
                                "--rtsp-bind", "127.0.0.1"], capture_output=True, text=True, timeout=30,
                               check=False)
        check(clash.returncode == 1 and "--rtsp-port 0 turns RTSP off" in clash.stderr,
              f"taken RTSP port is a startup error (rc {clash.returncode})")

        case_ffprobe(vehicles[0], "tcp")
        case_ffprobe(vehicles[1], "udp")
        case_inside(vehicles[1])

        # Nobody playing: the renderer's demand lapses (3 s) and the server goes quiet.
        time.sleep(4.0)
        metrics = api(API_PORT, "GET", "/metrics")
        check(metrics["video"]["clients"] == 0 and metrics["video"]["streams"] == 0, "no clients after teardown")
        check(metrics["camera"]["watched"] == 0, "renderer idle once nobody plays")
        cpu0, wall0 = process_cpu_s(proc.pid), time.monotonic()
        time.sleep(2.0)
        idle = 100.0 * (process_cpu_s(proc.pid) - cpu0) / (time.monotonic() - wall0)
        print(f"rtsp_smoke: idle with the RTSP server listening: {idle:.1f}% of a core")
        check(idle < 25.0, f"idle near zero ({idle:.1f}%)")

        # MJPEG keeps working beside it (on by default), with its timestamps.
        with urllib.request.urlopen(
                f"http://127.0.0.1:{API_PORT}/instances/{vehicles[0]['instance']}/camera.mjpg", timeout=15) as resp:
            got = b""
            deadline = time.time() + 10
            while got.count(b"--skysimframe") < 2 and time.time() < deadline:
                got += resp.read1(65536)
        check(got.count(b"--skysimframe") >= 2 and b"X-Skysim-Capture-Unix-Usec: " in got,
              "camera.mjpg still streams, parts stamped")
    finally:
        check(stop(proc) == 0, "clean shutdown")
    with open(log_path) as f:
        log_text = f.read()
    check("camera H.264 (constrained baseline) on rtsp://127.0.0.1:18663/instances/<n>/main.264" in log_text,
          "startup line names the RTSP URL")

    # RTSP and MJPEG both off.
    proc = subprocess.Popen([SIM, "--vehicles", "1", "--base-instance", str(BASE_INSTANCE + 5), "--time-mode",
                             "interactive", "--dt", "0.0025", "--api-port", str(OFF_API_PORT), "--camera-fps", "5",
                             "--rtsp-port", "0", "--camera-mjpeg", "0"], stdout=subprocess.DEVNULL,
                            stderr=subprocess.STDOUT)
    try:
        metrics = wait_up(OFF_API_PORT, proc)
        check(metrics is not None and metrics["video"] is None, "--rtsp-port 0: /metrics video null")
        vehicle = listed(OFF_API_PORT, 1)[0]
        check(api_status(OFF_API_PORT, f"/instances/{vehicle['instance']}/camera/encoder") == 404,
              "--rtsp-port 0: encoder route 404")
        check(api_status(OFF_API_PORT, f"/instances/{vehicle['instance']}/camera.mjpg") == 404,
              "--camera-mjpeg 0: camera.mjpg 404")
        check(api_status(OFF_API_PORT, f"/vehicles/{vehicle['id']}/camera.jpg") == 200,
              "--camera-mjpeg 0: camera.jpg still served")
    finally:
        check(stop(proc) == 0, "clean shutdown (off)")
    for args in (["--rtsp-port", "70000"], ["--camera-mjpeg", "2"], ["--video-threads", "x"]):
        rc = subprocess.run([SIM, *args], capture_output=True, timeout=30, check=False).returncode
        check(rc == 2, f"bad {args} -> exit 2")

    shutil.rmtree(tmp, ignore_errors=True)
    if FAILURES:
        print(f"rtsp_smoke: {len(FAILURES)} failure(s)")
        return 1
    print("rtsp_smoke: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
