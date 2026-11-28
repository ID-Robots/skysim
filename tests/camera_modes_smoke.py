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


def fetch(path: str, limit: int | None = None) -> tuple[int, dict[str, str], bytes]:
    """A route's status, headers (lower-cased names) and body, the first `limit` bytes of it if given."""
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{API_PORT}{path}", timeout=15) as resp:
            body = resp.read(limit) if limit else resp.read()
            return resp.status, {k.lower(): v for k, v in resp.headers.items()}, body
    except urllib.error.HTTPError as e:
        return e.code, {k.lower(): v for k, v in e.headers.items()}, e.read()


def jpeg_size(jpeg: bytes) -> tuple[int, int]:
    """(width, height) from a JPEG's SOF marker; (0, 0) when there is none."""
    i = 2
    while i + 9 < len(jpeg) and jpeg[i] == 0xFF:
        if jpeg[i + 1] in (0xC0, 0xC1, 0xC2):
            return struct.unpack(">H", jpeg[i + 7:i + 9])[0], struct.unpack(">H", jpeg[i + 5:i + 7])[0]
        i += 2 + struct.unpack(">H", jpeg[i + 2:i + 4])[0]
    return 0, 0


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


def parse_sps(nal: bytes) -> tuple[tuple[int, int], float | None]:
    """(picture size after cropping, frame rate) of a Baseline SPS (no chroma fields). The rate is the
    VUI timing, time_scale / (2 * num_units_in_tick) as x264 writes it; None when there is none."""
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
    size = (width_mbs * 16 - 2 * (crop[0] + crop[1]), (2 - frame_mbs_only) * height_units * 16 - 2 * (crop[2] + crop[3]))
    fps = None
    try:
        if u(1):  # vui_parameters_present_flag
            if u(1) and u(8) == 255:  # aspect_ratio_info_present_flag, aspect_ratio_idc: Extended_SAR
                u(32)
            if u(1):  # overscan_info_present_flag
                u(1)
            if u(1):  # video_signal_type_present_flag
                u(4)
                if u(1):  # colour_description_present_flag
                    u(24)
            if u(1):  # chroma_loc_info_present_flag
                ue()
                ue()
            if u(1):  # timing_info_present_flag
                ticks, scale = u(32), u(32)
                fps = scale / (2.0 * ticks) if ticks else None
    except (ValueError, IndexError):  # a VUI that runs out of bits has no usable timing
        fps = None
    return size, fps


def sps_size(nal: bytes) -> tuple[int, int]:
    return parse_sps(nal)[0]


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
        status, body = call("POST", "/vehicles", {"instance": BASE + 1, "camera": {"size": "320x320"}})
        check(status == 400 and "aspect" in body.get("error", ""),
              f"a picture the fleet's draw size would be stretched to is a 400 ({body})")
        status, body = call("POST", "/vehicles", {"instance": BASE + 1, "camera": {"size": "4294969216x1080"}})
        check(status == 400, "a size too large for an int is a 400, not wrapped into range")
        check(call("POST", "/vehicles", {"instance": BASE + 1, "camera": {"fps": 0.5}})[0] == 400,
              "fps below 1 is a 400")
        check(call("GET", f"/instances/{BASE + 1}/camera")[0] == 404, "... none of them spawned anything")
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

        # --- a new picture size, and the frame and calibration that belong to it ------------------
        # Every JPEG names the vehicle and the picture revision it was drawn for (X-Skysim-Vehicle,
        # X-Skysim-Picture-Revision), and GET .../camera and the intrinsics name the current ones: a
        # size change is a new revision, and what is served after it is a frame of the new size with
        # the new revision, of the same vehicle.
        rev = call("GET", path)[1]["picture"]["revision"]
        vid = str(parked["id"])
        for size, step in (((480, 270), 1), (FLEET[:2], 2)):
            status, cam = (call("POST", path, {"size": f"{size[0]}x{size[1]}"}) if step == 1
                           else call("DELETE", path))
            check(status == 200 and cam["picture"]["revision"] == rev + step and picture_of(cam) == tuple(size),
                  f"{size[0]}x{size[1]}: the picture size change is revision {rev + step}")
            for _ in range(5):  # a cold frame can outlast camera.jpg's wait on a slow build: ask again
                status, headers, jpeg = fetch(f"/instances/{BASE + 1}/camera.jpg")
                if status != 404:
                    break
            intr = call("GET", f"/instances/{BASE + 1}/camera/intrinsics")[1]
            check(status == 200 and jpeg_size(jpeg) == tuple(size)
                  and headers.get("x-skysim-picture-revision") == str(rev + step)
                  and headers.get("x-skysim-vehicle") == vid and str(intr.get("vehicle")) == vid
                  and intr["picture_revision"] == rev + step and (intr["width"], intr["height"]) == tuple(size),
                  f"... camera.jpg is a {jpeg_size(jpeg)} frame of vehicle {headers.get('x-skysim-vehicle')} revision "
                  f"{headers.get('x-skysim-picture-revision')}, the intrinsics say {intr['width']}x{intr['height']} "
                  f"vehicle {intr.get('vehicle')} revision {intr['picture_revision']} (want vehicle {vid})")
            status, headers, part = fetch(f"/instances/{BASE + 1}/camera.mjpg", limit=1500)
            check(status == 200 and f"X-Skysim-Picture-Revision: {rev + step}".encode() in part
                  and f"X-Skysim-Vehicle: {vid}\r\n".encode() in part,
                  "... and so does the MJPEG stream's first part")
            status, headers, _ = fetch(f"/vehicles/{vid}/camera.jpg")
            check(status == 200 and headers.get("x-skysim-vehicle") == vid
                  and headers.get("x-skysim-picture-revision") == str(rev + step),
                  "... and camera.jpg by vehicle id")

        # A replacement at the same instance is a new vehicle: its revisions count again, so the
        # revision alone could pair its frames (another size) with a calibration of the old one,
        # read at the same number; the vehicle id cannot.
        old_vid = vid
        check(call("DELETE", f"/vehicles/{old_vid}")[0] == 200, "despawn the vehicle at the instance")
        deadline = time.monotonic() + 10
        while call("GET", path)[0] != 404 and time.monotonic() < deadline:
            time.sleep(0.05)
        check(call("POST", "/vehicles", {"instance": BASE + 1, "camera": {"size": "320x180"}})[0] == 200,
              "spawn a replacement at the same instance")
        parked = listed(BASE + 1)
        vid = str(parked["id"]) if parked else ""
        check(parked is not None and vid != old_vid, f"... a new vehicle id ({vid}, was {old_vid})")
        cam = call("GET", path)[1]
        for _ in range(5):
            status, headers, jpeg = fetch(f"/instances/{BASE + 1}/camera.jpg")
            if status != 404:
                break
        intr = call("GET", f"/instances/{BASE + 1}/camera/intrinsics")[1]
        check(status == 200 and jpeg_size(jpeg) == (320, 180) and str(cam.get("vehicle")) == vid
              and headers.get("x-skysim-vehicle") == vid and str(intr.get("vehicle")) == vid
              and headers.get("x-skysim-picture-revision") == str(intr["picture_revision"]),
              f"... its {jpeg_size(jpeg)} frame (vehicle {headers.get('x-skysim-vehicle')} revision "
              f"{headers.get('x-skysim-picture-revision')}) and calibration (vehicle {intr.get('vehicle')} revision "
              f"{intr['picture_revision']}) name the new vehicle and agree")
        # Back to the fleet's size, as the vehicle was before (and as what follows expects).
        check(call("DELETE", path)[0] == 200, "... and back to the fleet's camera")

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
        others_switched = []  # (instance, height) where the parked or landing-off vehicle switched
        off_top = 0.0  # the landing-off vehicle's greatest height
        switched_at: list[float] = []  # wall times of vehicle 54's mode changes (as polled)
        switch_polls: list[tuple[float, float]] = []  # ... and the poll before each, that saw the old mode
        draws: list[tuple[float, int, float]] = []  # per poll: wall time, vehicle 54's frames drawn, the last one's ms
        last_mode = None
        last_poll = time.time()
        deadline = time.time() + 60
        while time.time() < deadline:
            cam = call("GET", f"/instances/{BASE}/camera")[1]
            now = time.time()
            draws.append((now, cam["frames_drawn"], cam["render_ms"]))
            height = -call("GET", f"/instances/{BASE}/camera/dock")[1]["vehicle_pose"]["offset_ned"][2]
            timeline.append((round(height, 1), cam["now"]["mode"]))
            if last_mode is not None and cam["now"]["mode"] != last_mode:
                switched_at.append(now)
                switch_polls.append((last_poll, now))
            last_mode = cam["now"]["mode"]
            last_poll = now
            # The parked vehicle and the one with its landing mode off, watched all the way up.
            for other in (BASE + 1, BASE + 2):
                if call("GET", f"/instances/{other}/camera")[1]["landing"]["active"]:
                    others_switched.append(other)
            off_top = max(off_top, -call("GET", f"/instances/{BASE + 2}/camera/dock")[1]["vehicle_pose"]["offset_ned"][2])
            if cam["landing"]["active"]:
                saw_landing = True
                landing_metrics = max(landing_metrics, call("GET", "/metrics")[1]["camera"]["landing"])
                intr = call("GET", f"/instances/{BASE}/camera/intrinsics")[1]
                intrinsics_ok = intrinsics_ok or (intr["width"] == OWN[0] and intr["drawn"]["mode"] == "landing"
                                                  and intr["drawn"]["width"] == OWN[0] and intr["fps"] == LANDING[0])
            elif saw_landing and height > LANDING_HEIGHT + MARGIN:
                saw_after = True
                break
            until = time.time() + 0.25  # meanwhile vehicle 54's camera every 50 ms: each draw seen (below)
            while time.time() < until:
                time.sleep(0.05)
                polled = call("GET", f"/instances/{BASE}/camera")[1]
                draws.append((time.time(), polled["frames_drawn"], polled["render_ms"]))
        print(f"camera_modes_smoke: (height m, mode) every 0.25 s: {timeline}")
        check(saw_landing, "climbing from its dock it draws the landing mode")
        check(saw_after, f"above {LANDING_HEIGHT + MARGIN:g} m it draws its own mode again")
        check(landing_metrics >= 1, "/metrics camera.landing counts it")
        check(intrinsics_ok, "the intrinsics route: the same picture, drawn full size at the landing rate")
        check(not others_switched, f"neither the parked vehicle nor the landing-off one switched ({others_switched})")
        check(off_top > LANDING_HEIGHT, f"... the landing-off one climbed through the zone (to {off_top:.1f} m)")
        time.sleep(2.0)  # frames of its own mode after the switch back
        receiver.stop = True
        receiver.join(timeout=5)
        frames = receiver.client.frames
        sizes = []
        for f in frames:
            for nal in f.nals:
                if nal and nal[0] & 0x1F == 7:
                    size = sps_size(nal)
                    if not sizes or sizes[-1] != size:
                        sizes.append(size)
        print(f"camera_modes_smoke: picture sizes over the RTSP session, in order: {sizes}")
        check(sizes == [OWN[:2]], "one RTSP session, one picture size throughout (landing or not)")
        check(all(f.idr for f in frames if any(n and n[0] & 0x1F == 7 for n in f.nals)),
              "every SPS rides on an IDR")
        # Each switch (into landing, and back out) changes the frame rate, so it builds a new encoder:
        # the first access unit of the new mode is an IDR whose SPS times the new rate. The stream
        # also sends IDRs of its own accord, so one in a window around the switch proves nothing;
        # what does is the SPS. Going through every access unit in order, a change of the rate an SPS
        # times from the SPS before it is where a mode begins — a periodic IDR repeats the rate it is
        # in — so the changes must be exactly: into landing, then back to the normal rate, each on an
        # IDR, each between the poll that saw the old mode and the one that saw the new.
        check(len(switched_at) == 2, f"two switches seen ({len(switched_at)})")
        begins: list[tuple[object, float | None]] = []  # (access unit, the rate its SPS times)
        for f in frames:
            for nal in f.nals:
                if nal and nal[0] & 0x1F == 7:
                    rate = parse_sps(nal)[1]
                    if not begins or begins[-1][1] is None or rate is None or abs(rate - begins[-1][1]) > 1e-3:
                        begins.append((f, rate))
        print(f"camera_modes_smoke: the rate each change of SPS timing began: {[r for _, r in begins]}")
        expected = [OWN[2], LANDING[0], OWN[2]]
        check(len(begins) == 3 and all(r is not None and abs(r - e) < 1e-3 for (_, r), e in zip(begins, expected)),
              f"the SPS timed the normal rate, the landing rate, the normal rate again ({expected})")
        if len(begins) == 3:
            for k in range(2):
                first = begins[k + 1][0]
                opened, saw = switch_polls[k] if k < len(switch_polls) else (0.0, 0.0)
                check(first.idr and opened - 0.5 <= first.received <= saw + 1.5,
                      f"the first access unit of mode {k + 2} is an IDR with the new rate's SPS, "
                      f"{first.received - saw:+.2f} s from the poll that saw the switch")
        # And no access unit drawn in a mode is coded at another mode's rate. Where a mode begins is
        # not a frame that says so: an old encoder that went on coding new-mode frames until the
        # next periodic IDR rebuilt it would still show one change of rate on an IDR. So each access
        # unit is tied to the mode it was drawn in by when: its capture time (RTP timestamp against
        # the sender report) against the polls. The flip fell between the poll that last saw a mode
        # and the poll that saw the next, so before the one and after the other (with a little for the
        # two wall clocks) a frame's mode is known, and the rate its encoder was timing — the last
        # SPS's — must be that mode's. Every such access unit is looked at, none sampled.
        rate_in_force: list[float | None] = []
        timing = None
        for f in frames:
            for nal in f.nals:
                if nal and nal[0] & 0x1F == 7:
                    timing = parse_sps(nal)[1]
            rate_in_force.append(timing)
        clock_s = 0.15
        if len(switch_polls) == 2:
            known_in = [(-float("inf"), switch_polls[0][0] - clock_s),
                        (switch_polls[0][1] + clock_s, switch_polls[1][0] - clock_s),
                        (switch_polls[1][1] + clock_s, float("inf"))]
            for m, (lo, hi) in enumerate(known_in):
                known = [(f, r) for f, r in zip(frames, rate_in_force)
                         if (t := receiver.client.capture_time(f)) is not None and lo <= t <= hi]
                wrong = [r for _, r in known if r is None or abs(r - expected[m]) > 1e-3]
                check(len(known) > 0 and not wrong,
                      f"all {len(known)} access units drawn in mode {m + 1} are coded at {expected[m]:g} fps"
                      f" ({len(wrong)} are not: {sorted({r for r in wrong if r is not None})[:3]})")
        # Frame spacing over RTP, away from the switches: the landing rate (5 fps, 200 ms) while it
        # was landing, the vehicle's own (8 fps, 125 ms) after. RTP stamps when a frame was drawn, and
        # a frame is drawn when it is due or when the one before is done, whichever is later: frame i
        # comes at most max(period - D[i-1], 0) + D[i] after frame i-1, D being draw times. Where a
        # build draws the landing frames (full size, the dock marker close up) slower than their
        # period (the coverage build's -O0 on a CI runner: 237 ms apart), their median spacing is held
        # to that bound over the draws the polls of the camera route saw (frames_drawn, render_ms) —
        # exact where the poll before saw the draw before, else period + D[i], and each draw no poll
        # saw at the most allowed, twice the period, so the rest rate (500 ms) never passes for
        # landing — at its upper quartile, as the two are taken over different frames (alternating
        # draw times put their medians in different halves). Only where the polls saw every draw
        # (one at most between two) and each fits in the period is it the period itself: the
        # bounds' 10 % covers the draw times' jitter.
        def spacing(t0: float, t1: float) -> float | None:
            window = [f for f in frames if t0 <= f.received <= t1]
            steps = sorted((b.rtp_ts - a.rtp_ts) / 90.0 for a, b in zip(window, window[1:]))
            return steps[len(steps) // 2] if steps else None

        def due_spacing(t0: float, t1: float, period_ms: float) -> float:
            seen = [(n1 - n0, d1, max(period_ms - d0, 0.0) + d1 if n1 == n0 + 1 else period_ms + d1)
                    for (t, n0, d0), (_, n1, d1) in zip(draws, draws[1:]) if t0 <= t <= t1 and n1 != n0]
            if not seen:
                return 2.0 * period_ms  # no draw seen: no evidence either way
            if all(k == 1 and d <= period_ms for k, d, _ in seen):
                return period_ms
            unseen = sum(k - 1 for k, _, _ in seen)  # drawn between two polls and never seen: nothing known
            bounds = sorted([b for _, _, b in seen] + [2.0 * period_ms] * unseen)
            return min(2.0 * period_ms, max(period_ms, bounds[3 * len(bounds) // 4]))

        if len(switched_at) == 2:
            landing_step = spacing(switched_at[0] + 1.0, switched_at[1] - 0.5)
            landing_ms = due_spacing(switched_at[0] + 1.0, switched_at[1] - 0.5, 1000.0 / LANDING[0])
            normal_step = spacing(switched_at[1] + 1.0, time.time())
            check(landing_step is not None and 180 < landing_step < 220 / 200 * landing_ms,
                  f"frames {landing_step} ms apart while landing ({landing_ms:.0f})")
            check(normal_step is not None and 110 < normal_step < 140,
                  f"frames {normal_step} ms apart in its normal mode after (125)")

        cam = call("GET", f"/instances/{BASE + 1}/camera")[1]
        check(not cam["landing"]["active"] and cam["now"]["mode"] == "normal" and cam["now"]["resting"],
              "the parked vehicle rests")
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
