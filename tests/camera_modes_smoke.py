#!/usr/bin/env python3
"""Per-vehicle camera modes and the landing mode, end to end (TASK-2661).

Starts skysim with pictures drawn at half size in the normal mode, the landing mode at full size,
and a rest rate, then checks:

  - POST /vehicles "camera" gives a vehicle its own picture size and mode; one out of bounds is a
    400 and spawns nothing;
  - GET / POST / DELETE /instances/<n>/camera (and /vehicles/<id>/camera) read and change a
    vehicle's modes, refuse what is out of bounds without changing anything, and put it back;
  - a vehicle flown up from its dock draws the landing mode while it is in the zone around the
    dock and its normal mode above it, and its RTSP stream keeps one picture size throughout (an
    IDR at each switch, for the new frame rate), the intrinsics route says what it is drawn at,
    and /metrics counts it;
  - a vehicle parked on its dock rests at the rest rate and never switches; nor does one whose
    landing mode is turned off.

Usage: python3 tests/camera_modes_smoke.py <path-to-skysim>
"""
from __future__ import annotations

import json
import os
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools", "bench"))
from rtsp_client import RtspClient  # noqa: E402 - beside the bench tools, not a package

SIM = sys.argv[1] if len(sys.argv) > 1 else "./build/skysim"
API_PORT = 18702
RTSP_PORT = 18703
BASE = 54  # instances 54, 55, 56: udp 9542, 9552, 9562
FLEET = (320, 180, 10.0)  # the fleet's picture size and frame rate
DRAW = (160, 90)  # the normal mode draws half that and scales it up
OWN = (256, 144, 8.0)  # vehicle 54's own picture size and rate
LANDING = (5.0, 2000)  # the landing mode's rate and bitrate; drawn at the picture size
REST_FPS = 2.0
LANDING_HEIGHT = 12.0
MARGIN = 5.0
FAILURES: list[str] = []


def check(cond: bool, what: str) -> None:
    print(f"camera_modes_smoke: {'ok  ' if cond else 'FAIL'} {what}", flush=True)
    if not cond:
        FAILURES.append(what)


def call(method: str, path: str, body: dict | str | None = None) -> tuple[int, dict]:
    data = None if body is None else (body if isinstance(body, str) else json.dumps(body)).encode()
    req = urllib.request.Request(f"http://127.0.0.1:{API_PORT}{path}", data=data, method=method)
    try:
        with urllib.request.urlopen(req, timeout=15) as resp:
            return resp.status, json.loads(resp.read().decode() or "{}")
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode() or "{}")


def listed(instance: int) -> dict | None:
    """POST /vehicles answers before the tick publishes the snapshot: poll the list."""
    deadline = time.time() + 10
    while time.time() < deadline:
        for v in call("GET", "/vehicles")[1]:
            if v["instance"] == instance:
                return v
        time.sleep(0.05)
    return None


def mode_of(cam: dict, key: str) -> tuple:
    m = cam[key]
    return (m["draw_width"], m["draw_height"], m["fps"])


def picture_of(cam: dict) -> tuple:
    return (cam["picture"]["width"], cam["picture"]["height"])


def rbsp(nal: bytes) -> bytes:
    out = bytearray()
    zeros = 0
    for b in nal[1:]:
        if zeros >= 2 and b == 3:
            zeros = 0
            continue
        out.append(b)
        zeros = zeros + 1 if b == 0 else 0
    return bytes(out)


def sps_size(nal: bytes) -> tuple[int, int]:
    """Picture size of a Baseline SPS (no chroma fields), after cropping."""
    bits = "".join(f"{b:08b}" for b in rbsp(nal)[3:])
    pos = 0

    def ue() -> int:
        nonlocal pos
        zeros = 0
        while bits[pos] == "0":
            zeros += 1
            pos += 1
        value = int(bits[pos:pos + zeros + 1], 2) - 1
        pos += zeros + 1
        return value

    def u(n: int) -> int:
        nonlocal pos
        value = int(bits[pos:pos + n], 2)
        pos += n
        return value

    ue()  # seq_parameter_set_id
    ue()  # log2_max_frame_num_minus4
    poc_type = ue()
    if poc_type == 0:
        ue()
    ue()  # max_num_ref_frames
    u(1)  # gaps_in_frame_num_value_allowed_flag
    width_mbs = ue() + 1
    height_units = ue() + 1
    frame_mbs_only = u(1)
    if not frame_mbs_only:
        u(1)
    u(1)  # direct_8x8_inference_flag
    crop = [0, 0, 0, 0]
    if u(1):
        crop = [ue(), ue(), ue(), ue()]
    return (width_mbs * 16 - 2 * (crop[0] + crop[1]),
            (2 - frame_mbs_only) * height_units * 16 - 2 * (crop[2] + crop[3]))


class FakeSitl(threading.Thread):
    """Answers each vehicle's JSON lockstep with a constant servo frame (gimbal straight down)."""

    def __init__(self, motors: dict[int, int]) -> None:
        super().__init__(daemon=True)
        self.motors = motors
        self.socks = {}
        for instance in motors:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("127.0.0.1", 9002 + 10 * instance))
            self.socks[instance] = s
        self.count = dict.fromkeys(motors, 0)
        self.stop = False

    def send(self, instance: int) -> None:
        self.count[instance] += 1
        pwm = [self.motors[instance]] * 4 + [1000] * 12
        pwm[9] = 1100  # servo 10: gimbal pitch -90
        pwm[10] = 1500  # servo 11: gimbal yaw along the nose
        self.socks[instance].send(struct.pack("<HHI16H", 18458, 400, self.count[instance], *pwm))

    def run(self) -> None:
        last = dict.fromkeys(self.socks, 0.0)
        for instance in self.socks:
            self.send(instance)
        by_sock = {s: i for i, s in self.socks.items()}
        import select

        while not self.stop:
            ready, _, _ = select.select(list(by_sock), [], [], 0.05)
            now = time.monotonic()
            for s in ready:
                try:
                    s.recv(4096)
                except OSError:
                    continue
                last[by_sock[s]] = now
                self.send(by_sock[s])
            for instance, t in last.items():
                if now - t > 0.2:
                    last[instance] = now
                    self.send(instance)


class Receiver(threading.Thread):
    def __init__(self, instance: int) -> None:
        super().__init__(daemon=True)
        self.client = RtspClient(f"rtsp://127.0.0.1:{RTSP_PORT}/instances/{instance}/main.264")
        self.stop = False

    def run(self) -> None:
        while not self.stop:
            self.client.read_for(0.2)


def main() -> int:
    cmd = [SIM, "--vehicles", "0", "--base-instance", str(BASE), "--time-mode", "interactive", "--dt", "0.0025",
           "--api-port", str(API_PORT), "--rtsp-port", str(RTSP_PORT), "--rtsp-bind", "127.0.0.1",
           "--camera-fps", f"{FLEET[2]:g}", "--camera-size", f"{FLEET[0]}x{FLEET[1]}", "--camera-fov", "81",
           "--camera-threads", "2", "--camera-cache-mb", "64", "--camera-draw-size", f"{DRAW[0]}x{DRAW[1]}",
           "--camera-landing-fps", f"{LANDING[0]:g}", "--camera-landing-bitrate", str(LANDING[1]),
           "--camera-landing-radius", "25", "--camera-landing-height", f"{LANDING_HEIGHT:g}",
           "--camera-rest-fps", f"{REST_FPS:g}"]
    proc = subprocess.Popen(cmd)
    sitl = None
    receiver = None
    try:
        deadline = time.time() + 20
        while time.time() < deadline:
            try:
                call("GET", "/metrics")
                break
            except (urllib.error.URLError, ConnectionError):
                time.sleep(0.1)

        # --- spawn with a camera -------------------------------------------------------------
        status, body = call("POST", "/vehicles", {"instance": BASE + 1, "camera": {"width": 12}})
        check(status == 400 and "width" in body.get("error", ""), f"a camera out of bounds is a 400 ({body})")
        check(call("GET", f"/instances/{BASE + 1}/camera")[0] == 404, "... and spawns nothing")
        status, _ = call("POST", "/vehicles", {"instance": BASE, "camera": {"size": f"{OWN[0]}x{OWN[1]}",
                                                                             "fps": OWN[2]}})
        check(status == 200, "spawn with a camera of its own")
        check(call("POST", "/vehicles", {"instance": BASE + 1})[0] == 200, "spawn with the fleet's camera")
        check(call("POST", "/vehicles", {"instance": BASE + 2, "camera": {"landing": False}})[0] == 200,
              "spawn with the landing camera off")
        climbing = listed(BASE)
        parked = listed(BASE + 1)
        off = listed(BASE + 2)
        check(None not in (climbing, parked, off), "all three listed")

        status, cam = call("GET", f"/instances/{BASE}/camera")
        check(status == 200 and picture_of(cam) == OWN[:2] and cam["picture"]["own"],
              f"its own picture size {picture_of(cam)}")
        check(mode_of(cam, "normal") == (*DRAW, OWN[2]) and cam["normal"]["own"],
              f"its own rate, drawn at the fleet's draw size {mode_of(cam, 'normal')}")
        check(cam["landing"]["available"] and mode_of(cam, "landing") == (*OWN[:2], LANDING[0])
              and cam["landing"]["bitrate_kbps"] == LANDING[1] and not cam["landing"]["own"],
              "the fleet's landing mode, drawn at its picture size")
        check(cam["now"]["mode"] == "normal", "on its dock: its normal mode")
        check(call("GET", f"/vehicles/{climbing['id']}/camera")[1]["normal"] == cam["normal"],
              "the same by vehicle id")
        cam_off = call("GET", f"/instances/{BASE + 2}/camera")[1]
        check(not cam_off["landing"]["available"] and not cam_off["landing"]["enabled"],
              "landing:false at spawn turns it off for that vehicle")

        # --- change and put back -------------------------------------------------------------
        path = f"/instances/{BASE + 1}/camera"
        status, cam = call("POST", path, {"fps": 4})
        check(status == 200 and mode_of(cam, "normal") == (*DRAW, 4.0) and cam["normal"]["own"]
              and picture_of(cam) == FLEET[:2], f"POST fps changes the rate alone {mode_of(cam, 'normal')}")
        cam = call("POST", path, {"draw": f"{FLEET[0]}x{FLEET[1]}"})[1]
        check(mode_of(cam, "normal") == (*FLEET[:2], 4.0), "POST draw: drawn at full size")
        status, body = call("POST", path, {"draw_width": 99999})
        check(status == 400 and "draw_width" in body.get("error", ""), "a draw width out of bounds is a 400")
        status, body = call("POST", path, {"draw": "100x100"})
        check(status == 400 and "aspect" in body.get("error", ""), "a draw size of another aspect ratio is a 400")
        check(mode_of(call("GET", path)[1], "normal") == (*FLEET[:2], 4.0), "... and changes nothing")
        check(call("POST", path, "{not json")[0] == 400, "a body that is not JSON is a 400")
        check(call("POST", path, {"size": "big"})[0] == 400, "a size that is not WxH is a 400")
        cam = call("POST", path, {"landing": {"fps": 3}})[1]
        check(cam["landing"]["own"] and mode_of(cam, "landing") == (*FLEET[:2], 3.0),
              "its own landing mode, from the fleet's")
        cam = call("POST", path, {"landing": False})[1]
        check(not cam["landing"]["available"], "landing:false turns it off")
        status, cam = call("DELETE", path)
        check(status == 200 and not cam["normal"]["own"] and mode_of(cam, "normal") == (*DRAW, FLEET[2])
              and cam["landing"]["available"] and not cam["landing"]["own"], "DELETE: the fleet's settings again")
        check(call("GET", "/instances/999/camera")[0] == 404, "an unknown instance is a 404")

        # --- the landing camera in flight ----------------------------------------------------
        receiver = Receiver(BASE)
        check(receiver.client.describe().status == 200 and receiver.client.play().status == 200,
              "RTSP DESCRIBE/SETUP/PLAY")
        check(f"a=x-dimensions:{OWN[0]},{OWN[1]}" in receiver.client.sdp, "the SDP names its picture size")
        receiver.start()
        time.sleep(1.5)
        cam = call("GET", f"/instances/{BASE}/camera")[1]
        check(cam["now"]["resting"] and cam["now"]["fps"] == REST_FPS and cam["watched"],
              "watched on its dock, it rests: drawn at the rest rate")
        rest_frames = len(receiver.client.frames)
        time.sleep(2.0)
        rest_rate = (len(receiver.client.frames) - rest_frames) / 2.0
        check(1.0 <= rest_rate <= 3.5, f"... {rest_rate:.1f} frames/s over RTSP")
        sitl = FakeSitl({BASE: 1600, BASE + 1: 1000, BASE + 2: 1600})
        sitl.start()
        timeline = []
        saw_landing = saw_after = False
        landing_metrics = 0
        intrinsics_ok = False
        deadline = time.time() + 60
        while time.time() < deadline:
            cam = call("GET", f"/instances/{BASE}/camera")[1]
            height = -call("GET", f"/instances/{BASE}/camera/dock")[1]["vehicle_pose"]["offset_ned"][2]
            timeline.append((round(height, 1), cam["now"]["mode"]))
            if cam["landing"]["active"]:
                saw_landing = True
                landing_metrics = max(landing_metrics, call("GET", "/metrics")[1]["camera"]["landing"])
                intr = call("GET", f"/instances/{BASE}/camera/intrinsics")[1]
                intrinsics_ok = intrinsics_ok or (intr["width"] == OWN[0] and intr["drawn"]["mode"] == "landing"
                                                  and intr["drawn"]["width"] == OWN[0] and intr["fps"] == LANDING[0])
            elif saw_landing and height > LANDING_HEIGHT + MARGIN:
                saw_after = True
                break
            time.sleep(0.25)
        print(f"camera_modes_smoke: (height m, mode) every 0.25 s: {timeline}")
        check(saw_landing, "climbing from its dock it draws the landing mode")
        check(saw_after, f"above {LANDING_HEIGHT + MARGIN:g} m it draws its own mode again")
        check(landing_metrics >= 1, "/metrics camera.landing counts it")
        check(intrinsics_ok, "the intrinsics route: the same picture, drawn full size at the landing rate")
        time.sleep(2.0)  # frames of its own mode after the switch back
        receiver.stop = True
        receiver.join(timeout=5)
        sizes = []
        for f in receiver.client.frames:
            for nal in f.nals:
                if nal and nal[0] & 0x1F == 7:
                    size = sps_size(nal)
                    if not sizes or sizes[-1] != size:
                        sizes.append(size)
        print(f"camera_modes_smoke: picture sizes over the RTSP session, in order: {sizes}")
        check(sizes == [OWN[:2]], "one RTSP session, one picture size throughout (landing or not)")
        idr_at_switch = all(f.idr for k, f in enumerate(receiver.client.frames)
                            if any(n and n[0] & 0x1F == 7 for n in f.nals))
        check(idr_at_switch, "every SPS rides on an IDR")
        # Frame spacing over RTP: the landing rate (5 fps, 200 ms) in the zone, the vehicle's own
        # (8 fps, 125 ms) elsewhere.
        steps = [(b.rtp_ts - a.rtp_ts) / 90.0 for a, b in zip(receiver.client.frames, receiver.client.frames[1:])]
        check(any(180 < s < 220 for s in steps) and any(110 < s < 140 for s in steps),
              "frames 200 ms apart while landing and 125 ms apart in its normal mode")

        cam = call("GET", f"/instances/{BASE + 1}/camera")[1]
        check(not cam["landing"]["active"] and cam["now"]["mode"] == "normal" and cam["now"]["resting"],
              "the parked vehicle never switched, and rests")
        cam = call("GET", f"/instances/{BASE + 2}/camera")[1]
        check(not cam["landing"]["active"], "the vehicle with its landing camera off never switched")
        check(call("DELETE", f"/vehicles/{climbing['id']}")[0] == 200, "despawn")
        status, _ = call("POST", "/vehicles", {"instance": BASE})
        time.sleep(0.3)
        cam = call("GET", f"/instances/{BASE}/camera")[1]
        check(status == 200 and not cam["normal"]["own"] and not cam["picture"]["own"]
              and picture_of(cam) == FLEET[:2], "the next vehicle at the instance starts from the fleet's camera")
    finally:
        if sitl is not None:
            sitl.stop = True
        if receiver is not None:
            receiver.stop = True
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            proc.kill()
    if FAILURES:
        print(f"camera_modes_smoke: {len(FAILURES)} failure(s)")
        return 1
    print("camera_modes_smoke: all passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
