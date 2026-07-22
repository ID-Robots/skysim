#!/usr/bin/env python3
"""Conformance harness: fly real ArduPilot SITL instances against skysim.

Per vehicle: heartbeat -> EKF origin -> prearm clean -> arm -> GUIDED takeoff 10 m ->
4-waypoint square -> RTL -> disarm. Fails on crash/failsafe/EKF-error STATUSTEXT.
Exit code 0 iff every vehicle passes. Used as the acceptance gate from M2 onward.

Usage:
  python3 tools/harness/conformance.py --binary $ARDUPILOT_ROOT/build/sitl/bin/arducopter \
      --sim ./build/skysim --vehicles 1 --base-instance 1 --home 42.1354,24.7453,164,0

--base-instance defaults to 1: on hosts where an unrelated SITL owns the -I0 ports.
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import tempfile
import threading
import time
from dataclasses import dataclass, field

from pymavlink import mavutil  # pip install pymavlink

TAKEOFF_ALT_M = 10.0
ALT_TOL_M = 0.5
WP_TOL_M = 2.0
SQUARE_NE = [(20.0, 0.0), (20.0, 20.0), (0.0, 20.0), (0.0, 0.0)]
POSITION_ONLY_MASK = 0b0000_1101_1111_1000  # use position, ignore vel/accel/yaw

FAILURE_RE = re.compile(r"crash|failsafe|ekf variance|thrust loss|dead reckoning", re.IGNORECASE)


@dataclass
class Vehicle:
    instance: int
    proc: subprocess.Popen
    mav: mavutil.mavfile | None = None
    statustexts: list[str] = field(default_factory=list)
    failures: list[str] = field(default_factory=list)
    _last_gcs_hb: float = 0.0

    @property
    def mavlink_addr(self) -> str:
        return f"tcp:127.0.0.1:{5760 + 10 * self.instance}"

    # Single message pump: keeps a GCS heartbeat on the wire and records STATUSTEXT.
    def poll(self, types: list[str], timeout: float):
        assert self.mav is not None
        now = time.time()
        if now - self._last_gcs_hb > 1.0:
            self.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self._last_gcs_hb = now
        msg = self.mav.recv_match(type=types + ["STATUSTEXT"], blocking=True, timeout=timeout)
        if msg is not None and msg.get_type() == "STATUSTEXT":
            text = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="replace")
            self.statustexts.append(text)
            print(f"  [I{self.instance}] {text}")
            if FAILURE_RE.search(text):
                self.failures.append(text)
            return None
        return msg

    def wait_until(self, types: list[str], pred, timeout: float, what: str):
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.poll(types, timeout=2)
            if msg is not None and pred(msg):
                return msg
        raise TimeoutError(f"[I{self.instance}] timeout waiting for {what}")


def stop(proc: subprocess.Popen, timeout: float = 10.0) -> None:
    """Terminate and WAIT. Back-to-back runs collide on SITL ports otherwise."""
    proc.terminate()
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


def launch_sitl(binary: str, instance: int, home: str, defaults: str) -> subprocess.Popen:
    run_dir = tempfile.mkdtemp(prefix=f"skysim_sitl_I{instance}_")
    cmd = [
        binary,
        "--model", "json:127.0.0.1",
        "-I", str(instance),
        "--home", home,
        "--defaults", defaults,
    ]
    log = open(os.path.join(run_dir, "sitl.log"), "w")  # kept for post-mortem debugging
    print(f"  [I{instance}] run dir: {run_dir}")
    return subprocess.Popen(cmd, cwd=run_dir, stdout=log, stderr=subprocess.STDOUT)


def wait_ready(v: Vehicle, timeout: float = 120.0) -> None:
    v.mav = mavutil.mavlink_connection(v.mavlink_addr, retries=30)
    v.mav.wait_heartbeat(timeout=timeout)
    v.mav.mav.request_data_stream_send(v.mav.target_system, v.mav.target_component,
                                       mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    v.wait_until(["GLOBAL_POSITION_INT"], lambda m: abs(m.lat) > 10_000_000, timeout,
                 "EKF origin (GLOBAL_POSITION_INT)")
    print(f"  [I{v.instance}] EKF origin set")
    prearm_bit = mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
    v.wait_until(["SYS_STATUS"],
                 lambda m: (m.onboard_control_sensors_enabled & prearm_bit) and
                           (m.onboard_control_sensors_health & prearm_bit),
                 timeout, "prearm checks clean")
    print(f"  [I{v.instance}] prearm clean")


def flight(v: Vehicle, east_offset: float = 0.0) -> None:
    m = v.mav
    assert m is not None
    m.set_mode("GUIDED")
    v.wait_until(["HEARTBEAT"],
                 lambda h: h.get_srcComponent() == 1 and h.custom_mode == 4,  # GUIDED
                 30, "GUIDED mode")

    for attempt in range(5):
        m.arducopter_arm()
        try:
            v.wait_until(["HEARTBEAT"],
                         lambda h: h.get_srcComponent() == 1 and
                                   (h.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                         10, "armed")
            break
        except TimeoutError:
            if attempt == 4:
                raise
    print(f"  [I{v.instance}] armed")

    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                            0, 0, 0, 0, 0, 0, TAKEOFF_ALT_M)
    v.wait_until(["GLOBAL_POSITION_INT"],
                 lambda p: abs(p.relative_alt / 1000.0 - TAKEOFF_ALT_M) < ALT_TOL_M,
                 90, f"takeoff to {TAKEOFF_ALT_M} m")
    print(f"  [I{v.instance}] takeoff {TAKEOFF_ALT_M} m OK")

    for i, (north, east_rel) in enumerate(SQUARE_NE):
        east = east_rel + east_offset  # per-vehicle airspace cell (shared physical world)
        m.mav.set_position_target_local_ned_send(
            0, m.target_system, m.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            POSITION_ONLY_MASK, north, east, -TAKEOFF_ALT_M, 0, 0, 0, 0, 0, 0, 0, 0)
        v.wait_until(["LOCAL_POSITION_NED"],
                     lambda p, n=north, e=east: ((p.x - n) ** 2 + (p.y - e) ** 2) ** 0.5 < WP_TOL_M
                                                and abs(-p.z - TAKEOFF_ALT_M) < 2.0,
                     90, f"waypoint {i + 1} ({north},{east})")
        print(f"  [I{v.instance}] wp{i + 1} ({north:.0f},{east:.0f}) reached")

    m.set_mode("RTL")
    v.wait_until(["HEARTBEAT"],
                 lambda h: h.get_srcComponent() == 1 and
                           not (h.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                 240, "RTL + land + disarm")
    print(f"  [I{v.instance}] landed + disarmed")

    if v.failures:
        raise RuntimeError(f"[I{v.instance}] failure STATUSTEXT during flight: {v.failures}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--binary",
                    default=os.path.expandvars("$ARDUPILOT_ROOT/build/sitl/bin/arducopter"))
    ap.add_argument("--sim", default="./build/skysim")
    ap.add_argument("--vehicles", type=int, default=1)
    ap.add_argument("--base-instance", type=int, default=1)
    ap.add_argument("--home", default="42.1354,24.7453,164,0")
    ap.add_argument("--tiles", default="")
    ap.add_argument("--dt", default="0.00125")  # 800 Hz, pairs with SIM_RATE_HZ=800 overlay
    ap.add_argument("--spacing", type=float, default=30.0)  # per-vehicle airspace cell, m
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    defaults = ",".join([
        os.path.expandvars("$ARDUPILOT_ROOT/Tools/autotest/default_params/copter.parm"),
        os.path.join(here, "params", "skysim.parm"),
    ])

    sim_cmd = [args.sim, "--vehicles", str(args.vehicles), "--base-instance",
               str(args.base_instance), "--time-mode", "strict", "--dt", args.dt,
               "--spacing", str(args.spacing), "--strict-timeout", "60"]
    if args.tiles:
        sim_cmd += ["--tiles", args.tiles]
    sim = subprocess.Popen(sim_cmd)
    time.sleep(0.5)

    vehicles = [Vehicle(args.base_instance + i,
                        launch_sitl(args.binary, args.base_instance + i, args.home, defaults))
                for i in range(args.vehicles)]
    try:
        # Readiness and flights run concurrently: strict lockstep couples all vehicles, so a
        # sequential flight loop would stall everyone else's clock at each barrier.
        errors: list[str] = []

        def fly(idx: int, v: Vehicle) -> None:
            try:
                wait_ready(v)
                flight(v, east_offset=args.spacing * idx)
            except Exception as e:  # noqa: BLE001 — report per-vehicle, fail the run below
                errors.append(f"I{v.instance}: {e}")

        threads = [threading.Thread(target=fly, args=(i, v), daemon=True)
                   for i, v in enumerate(vehicles)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        if errors:
            for e in errors:
                print(f"FAIL {e}")
            return 1
        print(f"PASS: {len(vehicles)} vehicle(s)")
        return 0
    finally:
        # Sim first: killing SITLs one by one would otherwise trip the strict straggler
        # abort while later SITLs are still sending (harmless, but noisy).
        stop(sim)
        for v in vehicles:
            stop(v.proc)


if __name__ == "__main__":
    sys.exit(main())
