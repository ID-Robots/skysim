#!/usr/bin/env python3
"""Live collision demo on the cooked demo map, against a real ArduPilot SITL.

Map (tools/cooker/pretile.py): buildings on a 100 m grid, cross streets through home.
Target building (i=1,j=1): NED north 80..120, east 80..120, roof at 20 m.

Flight A — rooftop landing: takeoff 40 m, fly over the building, LAND; assert the vehicle
comes to rest ON the roof (rel alt ~20 m) and disarms.
Flight B — wall impact: re-arm on the roof, hop off to the street west of the building at
10 m, then command a position INSIDE the building.

The no-tunneling assertion is made on skysim's GROUND TRUTH (--truth-log), not on
ArduPilot's EKF estimate: a real wall impact makes the EKF glitch and drift several
meters (observed: "EKF variance" / "GPS Glitch or Compass error"), so the estimate is
exactly the wrong signal to judge collision correctness with.

Usage:
  python3 tools/harness/collision_demo.py [--base-instance 1] [--tiles build/demo_map/tiles]
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time

from pymavlink import mavutil

from conformance import Vehicle, launch_sitl, wait_ready  # same-dir import

ROOF_Z = -20.0          # building (1,1) top, NED
BLDG_N = (80.0, 120.0)  # north extent
BLDG_E = (80.0, 120.0)  # east extent
POSITION_ONLY_MASK = 0b0000_1101_1111_1000


def goto(v: Vehicle, north: float, east: float, down: float) -> None:
    v.mav.mav.set_position_target_local_ned_send(
        0, v.mav.target_system, v.mav.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        POSITION_ONLY_MASK, north, east, down, 0, 0, 0, 0, 0, 0, 0, 0)


def wait_pos(v: Vehicle, north: float, east: float, down: float, tol: float, timeout: float,
             what: str) -> None:
    v.wait_until(["LOCAL_POSITION_NED"],
                 lambda p: ((p.x - north) ** 2 + (p.y - east) ** 2 + (p.z - down) ** 2) ** 0.5 < tol,
                 timeout, what)


def arm_guided(v: Vehicle) -> None:
    v.mav.set_mode("GUIDED")
    v.wait_until(["HEARTBEAT"],
                 lambda h: h.get_srcComponent() == 1 and h.custom_mode == 4, 30, "GUIDED")
    for attempt in range(6):
        v.mav.arducopter_arm()
        try:
            v.wait_until(["HEARTBEAT"],
                         lambda h: h.get_srcComponent() == 1 and
                                   (h.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                         10, "armed")
            return
        except TimeoutError:
            if attempt == 5:
                raise


def takeoff(v: Vehicle, rel_alt: float) -> None:
    v.mav.mav.command_long_send(v.mav.target_system, v.mav.target_component,
                                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, rel_alt)
    v.wait_until(["GLOBAL_POSITION_INT"],
                 lambda p: abs(p.relative_alt / 1000.0 - rel_alt) < 1.0, 90,
                 f"takeoff to {rel_alt} m")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--binary",
                    default=os.path.expandvars("$ARDUPILOT_ROOT/build/sitl/bin/arducopter"))
    ap.add_argument("--sim", default="./build/skysim")
    ap.add_argument("--tiles", default="build/demo_map/tiles")
    ap.add_argument("--base-instance", type=int, default=1)
    ap.add_argument("--home", default="42.1354,24.7453,164,0")
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    defaults = ",".join([
        os.path.expandvars("$ARDUPILOT_ROOT/Tools/autotest/default_params/copter.parm"),
        os.path.join(here, "params", "skysim.parm"),
    ])

    truth_csv = os.path.abspath(f"collision_demo_truth_I{args.base_instance}.csv")
    sim = subprocess.Popen([args.sim, "--vehicles", "1", "--base-instance",
                            str(args.base_instance), "--time-mode", "strict",
                            "--dt", "0.00125", "--tiles", args.tiles,
                            "--truth-log", truth_csv])
    time.sleep(0.5)
    v = Vehicle(args.base_instance,
                launch_sitl(args.binary, args.base_instance, args.home, defaults))
    roof_ok = wall_ok = False
    try:
        wait_ready(v)

        # ---- Flight A: land on the roof of building (1,1). ----
        arm_guided(v)
        takeoff(v, 40.0)
        goto(v, 100.0, 100.0, -40.0)
        wait_pos(v, 100.0, 100.0, -40.0, 3.0, 90, "over building")
        v.mav.set_mode("LAND")
        v.wait_until(["HEARTBEAT"],
                     lambda h: h.get_srcComponent() == 1 and
                               not (h.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                     180, "landed on roof + disarmed")
        p = v.wait_until(["LOCAL_POSITION_NED"], lambda m: True, 10, "position after roof landing")
        roof_ok = (BLDG_N[0] < p.x < BLDG_N[1] and BLDG_E[0] < p.y < BLDG_E[1] and
                   abs(-p.z - 20.0) < 1.5)
        print(f"ROOF LANDING: pos=({p.x:.1f}N, {p.y:.1f}E, {-p.z:.1f}m alt) -> "
              f"{'PASS' if roof_ok else 'FAIL'} (expect on-roof at ~20.1 m)")

        # ---- Flight B: hop to the street, then fly INTO the west wall (east=80). ----
        arm_guided(v)
        takeoff(v, 30.0)
        goto(v, 100.0, 30.0, -30.0)
        wait_pos(v, 100.0, 30.0, -30.0, 3.0, 90, "street west of building")
        goto(v, 100.0, 30.0, -10.0)
        wait_pos(v, 100.0, 30.0, -10.0, 2.0, 60, "descend to 10 m in street")

        goto(v, 100.0, 100.0, -10.0)  # inside the building — impact expected at east=80
        ekf_max_east = -1e9
        t_end = time.time() + 45
        while time.time() < t_end:
            msg = v.poll(["LOCAL_POSITION_NED"], timeout=2)
            if msg is not None:
                ekf_max_east = max(ekf_max_east, msg.y)
            if any(("crash" in s.lower()) or ("failsafe" in s.lower()) for s in v.statustexts):
                # impact consequences reached ArduPilot; let it land/settle a bit, then stop
                time.sleep(10)
                break
        print(f"  EKF max east during impact: {ekf_max_east:.2f} m (estimate only, may drift)")
    finally:
        v.proc.terminate()
        sim.terminate()
        try:
            sim.wait(timeout=10)
        except subprocess.TimeoutExpired:
            sim.kill()

    # Ground-truth no-tunneling check: in the impact band (below the roof, in the building's
    # north span), the TRUE east position must stop at the wall plane (80) minus the body
    # half-width, never beyond. Roof-landing samples sit above the roof and are excluded.
    truth_max_east = -1e9
    reached_wall = False
    with open(truth_csv) as f:
        next(f)  # header
        for line in f:
            _, _, n, e, d = line.strip().split(",")
            north, east, down = float(n), float(e), float(d)
            alt = -down
            if 85.0 < north < 115.0 and 2.0 < alt < 19.5:
                truth_max_east = max(truth_max_east, east)
                if east > 77.0:
                    reached_wall = True
    wall_ok = reached_wall and truth_max_east < 80.0
    print(f"WALL IMPACT (ground truth): max_east={truth_max_east:.3f} m — wall plane at "
          f"80.0, body half-width 0.18 -> {'PASS' if wall_ok else 'FAIL'} "
          f"(reached wall: {reached_wall}, no tunneling iff max_east < 80.0)")

    print(f"COLLISION DEMO: roof={'PASS' if roof_ok else 'FAIL'} "
          f"wall={'PASS' if wall_ok else 'FAIL'}")
    return 0 if (roof_ok and wall_ok) else 1


if __name__ == "__main__":
    sys.exit(main())
