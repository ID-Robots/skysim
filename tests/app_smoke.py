#!/usr/bin/env python3
"""App-level smoke of the skysim binary (ctest 'app_smoke'): exercises main.cpp's replay,
canned, strict-physics, REST, radio field, strict-abort, and bad-args paths without any real SITL.

Usage: app_smoke.py <path-to-skysim>   (instances 91..95, 97 / ports 189xx: far from live use)
"""
from __future__ import annotations

import json
import os
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
import urllib.request

SIM = sys.argv[1]
FAILURES: list[str] = []


def check(cond: bool, what: str) -> None:
    if not cond:
        FAILURES.append(what)
        print(f"FAIL: {what}")


def servo(fc: int, pwm: int = 1500) -> bytes:
    return struct.pack("<HHI16H", 18458, 800, fc, *([pwm] * 16))


def stop(proc: subprocess.Popen) -> int:
    proc.send_signal(signal.SIGTERM)
    try:
        return proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        return proc.wait()


def api(port: int, method: str, path: str, body: bytes | None = None):
    req = urllib.request.Request(f"http://127.0.0.1:{port}{path}", data=body, method=method)
    with urllib.request.urlopen(req, timeout=15) as resp:
        return json.loads(resp.read().decode() or "{}")


def case_replay(tmp: str) -> None:
    rec = os.path.join(tmp, "servo.csv")
    out = os.path.join(tmp, "truth.csv")
    # 0.5 s of servo frames at the sim's default rate. Tied to the rate on purpose:
    # the assertion below is about how far the vehicle climbs in half a second, not in
    # some number of ticks, so the count has to track the default dt (800 Hz).
    ticks = 400
    with open(rec, "w") as f:
        for tick in range(ticks):
            f.write(f"{tick},0,800," + ",".join(["1700"] * 16) + "\n")
    rc = subprocess.run([SIM, "--vehicles", "1", "--replay-servo", rec, "--truth-log", out,
                         "--wind", "2,0,0", "--gust", "0.5,1", "--seed", "7"],
                        capture_output=True, text=True, timeout=60)
    check(rc.returncode == 0, f"replay exit ({rc.returncode}: {rc.stderr})")
    lines = open(out).readlines()
    check(len(lines) == ticks + 1, f"replay truth lines ({len(lines)})")  # header + ticks
    # 1700 pwm is well above hover: after 0.5 s the vehicle must have climbed clear.
    check(float(lines[-1].split(",")[4]) < -0.3, "replay vehicle climbed under 1700 pwm")

    rc = subprocess.run([SIM, "--replay-servo", "/nonexistent.csv"], capture_output=True, timeout=30)
    check(rc.returncode != 0, "replay missing file rejected")


def exchange(s: socket.socket, pkt: bytes, port: int, attempts: int = 5) -> str:
    """Send + await reply, resending on timeout (covers slow instrumented startup)."""
    for attempt in range(attempts):
        s.sendto(pkt, ("127.0.0.1", port))
        try:
            return s.recv(2048).decode()
        except socket.timeout:
            if attempt == attempts - 1:
                raise
    raise AssertionError("unreachable")


def case_canned() -> None:
    proc = subprocess.Popen([SIM, "--canned", "--vehicles", "1", "--base-instance", "91"])
    time.sleep(0.5)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(3)
    reply = exchange(s, servo(0), 9002 + 910)
    check(reply.startswith("{\"timestamp\":") and reply.endswith("}\n"), "canned reply shape")
    check("\"accel_body\":[0.000000,0.000000,-9.810000]" in reply, "canned at-rest accel")
    s.close()
    check(stop(proc) == 0, "canned clean shutdown")


def case_physics_udp() -> None:
    proc = subprocess.Popen([SIM, "--vehicles", "1", "--base-instance", "92", "--rangefinder"])
    time.sleep(0.5)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(3)
    last_ts = -1.0
    for fc in range(5):
        reply = exchange(s, servo(fc, 1000), 9002 + 920)
        ts = float(reply.split("\"timestamp\":")[1].split(",")[0])
        check(ts > last_ts, f"timestamp monotonic (fc={fc})")
        last_ts = ts
    check("\"rng_1\":" in reply, "rng_1 emitted")
    # duplicate frame: must be re-answered (handshake rule), timestamp unchanged
    s.sendto(servo(4, 1000), ("127.0.0.1", 9002 + 920))
    dup = s.recv(2048).decode()
    check(dup == reply, "duplicate re-answered with identical reply")
    s.close()
    check(stop(proc) == 0, "physics clean shutdown")


def case_rest() -> None:
    port = 18644
    proc = subprocess.Popen([SIM, "--vehicles", "0", "--base-instance", "93",
                             "--time-mode", "interactive", "--api-port", str(port),
                             "--spawn-binary", "/bin/true", "--spawn-defaults", "/dev/null"])
    time.sleep(1)
    try:
        v1 = api(port, "POST", "/vehicles", b"{}")
        check(v1["instance"] == 93, "spawn allocates base instance")
        v2 = api(port, "POST", "/vehicles", b"{\"launch_process\":true}")  # /bin/true exits: ok
        check(v2["instance"] == 94, "second spawn next instance")
        listing = api(port, "GET", "/vehicles")
        check(len(listing) == 2, "list shows both")
        m = api(port, "GET", "/metrics")
        check(m["vehicles"] == 2 and m["tick"] > 0, "metrics live")
        check(api(port, "DELETE", f"/vehicles/{v1['id']}")["ok"], "despawn ok")
        check(len(api(port, "GET", "/vehicles")) == 1, "list after despawn")
    finally:
        check(stop(proc) == 0, "rest clean shutdown")


def case_radio(tmp: str) -> None:
    """--radio-spec loads a field at startup; vehicles carry it; PUT/DELETE /radio replace it."""
    spec_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixtures", "radio_test_spec.json")
    port = 18647
    proc = subprocess.Popen([SIM, "--vehicles", "0", "--base-instance", "97",
                             "--time-mode", "interactive", "--api-port", str(port),
                             "--spawn-home", "47.0, 8.0, 0, 0", "--radio-spec", spec_path])
    time.sleep(1)
    try:
        check(api(port, "GET", "/radio")["radio"]["seed"] == 7, "radio spec loaded at startup")
        api(port, "POST", "/vehicles", b"{}")
        vehicle = api(port, "GET", "/vehicles")[0]
        cell = vehicle["radio"]["cell"]
        check(cell["source"] == "sim" and cell["operator"] == "SimTel A", "vehicle carries the primary modem")
        check(abs(vehicle["radio"]["lat"] - 47.0) < 1e-4, "vehicle radio sampled at its truth position")
        sample = api(port, "GET", "/radio/sample?lat=47&lon=8.002&alt_amsl=60")
        check(sample["cell"]["cell_id"] == "00101-A1" and sample["cell"]["registered"], "radio sample served")
        check(api(port, "DELETE", "/radio")["cleared"], "radio cleared")
        check("radio" not in api(port, "GET", "/vehicles")[0], "no radio without a spec")
    finally:
        check(stop(proc) == 0, "radio clean shutdown")

    # A home that only half parses must not place vehicles anywhere: no radio, and a warning.
    proc = subprocess.Popen([SIM, "--vehicles", "0", "--base-instance", "97",
                             "--time-mode", "interactive", "--api-port", str(port),
                             "--spawn-home", "47,8x,500,0", "--radio-spec", spec_path],
                            stderr=subprocess.PIPE, text=True)
    time.sleep(1)
    try:
        api(port, "POST", "/vehicles", b"{}")
        check("radio" not in api(port, "GET", "/vehicles")[0], "no radio without a usable home")
    finally:
        check(stop(proc) == 0, "bad-home clean shutdown")
    err = proc.stderr.read() if proc.stderr else ""
    check("--spawn-home 47,8x,500,0 is not lat,lon" in err, "bad home reported at startup")

    bad = os.path.join(tmp, "bad_radio.json")
    with open(bad, "w") as f:
        f.write('{"radio": {"seed": 1}}')
    for path in (bad, os.path.join(tmp, "missing_radio.json")):
        rc = subprocess.run([SIM, "--radio-spec", path], capture_output=True, text=True, timeout=30)
        check(rc.returncode == 2 and "--radio-spec" in rc.stderr, f"bad radio spec rejected: {path}")


def case_strict_abort() -> None:
    proc = subprocess.Popen([SIM, "--vehicles", "2", "--base-instance", "94",
                             "--strict-timeout", "1"],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    time.sleep(0.5)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(5)
    # Connect both vehicles once, then let 95 go silent while 94 keeps sending.
    s.sendto(servo(0), ("127.0.0.1", 9002 + 940))
    s.sendto(servo(0), ("127.0.0.1", 9002 + 950))
    t0 = time.time()
    fc = 1
    while proc.poll() is None and time.time() - t0 < 15:
        s.sendto(servo(fc), ("127.0.0.1", 9002 + 940))
        fc += 1
        time.sleep(0.02)
    s.close()
    out = proc.communicate()[0] or ""
    check(proc.returncode == 3, f"strict abort exit code ({proc.returncode})")
    check("STRICT ABORT" in out and "udp 9952" in out, "strict abort names the stalled vehicle")


def case_bad_args() -> None:
    for args in (["--wind", "garbage"], ["--gust", "x"], ["--time-mode", "warp"],
                 ["--vehicles", "-1"], ["--unknown-flag"], ["--dt"]):
        rc = subprocess.run([SIM, *args], capture_output=True, timeout=30)
        check(rc.returncode == 2, f"bad args rejected: {args}")
    rc = subprocess.run([SIM, "--tiles", "/nonexistent"], capture_output=True, timeout=60)
    check(rc.returncode == 1, "empty tiles dir aborts startup")


def main() -> int:
    with tempfile.TemporaryDirectory(prefix="skysim_smoke_") as tmp:
        case_replay(tmp)
        case_radio(tmp)
    case_canned()
    case_physics_udp()
    case_rest()
    case_strict_abort()
    case_bad_args()
    if FAILURES:
        print(f"app_smoke: {len(FAILURES)} failure(s)")
        return 1
    print("app_smoke: all checks OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
