#!/usr/bin/env python3
"""Conformance harness: fly real ArduPilot SITL instances against skysim.

Per vehicle: heartbeat -> EKF ready -> arm -> GUIDED takeoff 10 m -> square -> RTL -> disarm.
Exit code 0 iff every vehicle passes. Used as the acceptance gate from M2 onward.

Usage:
  python3 tools/harness/conformance.py --binary $ARDUPILOT_ROOT/build/sitl/bin/arducopter \
      --sim ./build/skysim --vehicles 1 --home 42.1354,24.7453,164,0
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from dataclasses import dataclass

from pymavlink import mavutil  # pip install pymavlink

TAKEOFF_ALT_M = 10.0
ALT_TOL_M = 0.5


@dataclass
class Vehicle:
    instance: int
    proc: subprocess.Popen
    mav: mavutil.mavfile | None = None

    @property
    def mavlink_addr(self) -> str:
        return f"tcp:127.0.0.1:{5760 + 10 * self.instance}"


def launch_sitl(binary: str, instance: int, home: str, defaults: str) -> subprocess.Popen:
    cmd = [
        binary,
        "--model", "json:127.0.0.1",
        "-I", str(instance),
        "--home", home,
        "--defaults", defaults,
    ]
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def wait_ready(v: Vehicle, timeout: float = 120.0) -> None:
    v.mav = mavutil.mavlink_connection(v.mavlink_addr, retries=20)
    v.mav.wait_heartbeat(timeout=timeout)
    # TODO(M2): wait for EKF origin / prearm-clean via STATUSTEXT + SYS_STATUS instead of sleeping.
    time.sleep(20)


def flight(v: Vehicle) -> None:
    m = v.mav
    assert m is not None
    m.set_mode("GUIDED")
    m.arducopter_arm()
    m.motors_armed_wait()
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                            0, 0, 0, 0, 0, 0, TAKEOFF_ALT_M)
    _wait_alt(m, TAKEOFF_ALT_M, ALT_TOL_M, timeout=60)
    # TODO(M2): 4-waypoint square via SET_POSITION_TARGET_LOCAL_NED, then RTL, then
    # motors_disarmed_wait; assert no failsafe/EKF STATUSTEXT seen during the flight.


def _wait_alt(m: mavutil.mavfile, target: float, tol: float, timeout: float) -> None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
        if msg and abs(msg.relative_alt / 1000.0 - target) < tol:
            return
    raise TimeoutError(f"never reached {target} m")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--binary", required=True)
    ap.add_argument("--sim", default="./build/skysim")
    ap.add_argument("--vehicles", type=int, default=1)
    ap.add_argument("--home", default="42.1354,24.7453,164,0")
    ap.add_argument("--defaults",
                    default=os.path.expandvars("$ARDUPILOT_ROOT/Tools/autotest/default_params/copter.parm"))
    args = ap.parse_args()

    sim = subprocess.Popen([args.sim, "--vehicles", str(args.vehicles), "--time-mode", "strict"])
    vehicles = [Vehicle(i, launch_sitl(args.binary, i, args.home, args.defaults))
                for i in range(args.vehicles)]
    try:
        for v in vehicles:
            wait_ready(v)
        for v in vehicles:  # TODO(M4): run concurrently (threads) once multi-vehicle lands
            flight(v)
        print(f"PASS: {len(vehicles)} vehicle(s)")
        return 0
    finally:
        for v in vehicles:
            v.proc.terminate()
        sim.terminate()


if __name__ == "__main__":
    sys.exit(main())
