#!/usr/bin/env python3
"""M5 acceptance: corridor flight between buildings with streamed tiles + wall rangefinder.

Map (pretile.py): building rows at north 80..120 and 180..220. The corridor at north=150 is
60 m wide. The quad traverses it eastward at 10 m alt with a right-facing rangefinder
(RNGFND2, fed by skysim rng_2): abeam a building it must read ~30 m (wall at north=120),
in the gaps it must go out of range (40 m). Tiles stream with --stream-max 2 (of 4), so the
walls the sensor sees are loaded/unloaded by proximity during the flight — the memory bound
is asserted via /metrics the whole way.

Usage: python3 tools/harness/m5_corridor.py [--base-instance 1] [--tiles build/demo_map/tiles]
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
import urllib.request

from pymavlink import mavutil

from collision_demo import arm_guided, goto, takeoff, wait_pos
from conformance import Vehicle, launch_sitl, stop, wait_ready

CORRIDOR_N = 150.0
WALL_N = 120.0  # north face of building row j=1
ALT = 10.0


def api(port: int, path: str):
    with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=10) as resp:
        return json.loads(resp.read().decode())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--binary",
                    default=os.path.expandvars("$ARDUPILOT_ROOT/build/sitl/bin/arducopter"))
    ap.add_argument("--sim", default="./build/skysim")
    ap.add_argument("--tiles", default="build/demo_map/tiles")
    ap.add_argument("--base-instance", type=int, default=1)
    ap.add_argument("--api-port", type=int, default=8645)
    ap.add_argument("--home", default="42.1354,24.7453,164,0")
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    defaults = ",".join([
        os.path.expandvars("$ARDUPILOT_ROOT/Tools/autotest/default_params/copter.parm"),
        os.path.join(here, "params", "skysim.parm"),
        os.path.join(here, "params", "rangefinder.parm"),
        os.path.join(here, "params", "rangefinder2.parm"),
    ])

    sim = subprocess.Popen([args.sim, "--vehicles", "1", "--base-instance",
                            str(args.base_instance), "--time-mode", "strict", "--dt", "0.00125",
                            "--tiles", args.tiles, "--stream-radius", "250", "--stream-max", "2",
                            "--rangefinders", "2", "--api-port", str(args.api_port)])
    time.sleep(0.5)
    v = Vehicle(args.base_instance,
                launch_sitl(args.binary, args.base_instance, args.home, defaults))
    wall_ok = gaps_ok = bound_ok = streamed = False
    try:
        wait_ready(v)
        arm_guided(v)
        takeoff(v, ALT)
        # Route along the streets: a direct line to the corridor entrance passes through the
        # 40 m building at (100,-100) — at 10 m altitude that is a wall, not a shortcut.
        goto(v, 0.0, -150.0, -ALT)
        wait_pos(v, 0.0, -150.0, -ALT, 3.0, 120, "street junction south of corridor")
        goto(v, CORRIDOR_N, -150.0, -ALT)
        wait_pos(v, CORRIDOR_N, -150.0, -ALT, 3.0, 120, "corridor west end")

        goto(v, CORRIDOR_N, 150.0, -ALT)  # traverse; sample the right-facing rangefinder
        samples: list[tuple[float, float]] = []  # (east, wall_distance)
        max_resident = 0
        t_end = time.time() + 120
        east = -150.0
        while time.time() < t_end and east < 145.0:
            msg = v.poll(["DISTANCE_SENSOR", "LOCAL_POSITION_NED"], timeout=2)
            if msg is None:
                continue
            if msg.get_type() == "LOCAL_POSITION_NED":
                east = msg.y
                m = api(args.api_port, "/metrics")
                max_resident = max(max_resident, m["resident_tiles"])
                if m["resident_tiles"] >= 1:
                    streamed = True
            elif msg.orientation == 2:  # right-facing
                samples.append((east, msg.current_distance / 100.0))

        # Abeam building columns (east within [-115,-85] or [85,115]): wall at ~30 m.
        abeam = [d for e, d in samples if abs(abs(e) - 100.0) < 15.0]
        gaps = [d for e, d in samples if abs(e) < 50.0]
        wall_ok = bool(abeam) and min(abeam) > 26.0 and min(abeam) < 34.0
        gaps_ok = bool(gaps) and max(gaps) > 38.0
        bound_ok = 0 < max_resident <= 2
        print(f"corridor samples={len(samples)} abeam_min={min(abeam) if abeam else -1:.1f} m "
              f"(expect ~30), gap_max={max(gaps) if gaps else -1:.1f} m (expect 40), "
              f"max_resident_tiles={max_resident} (bound 2)")

        v.mav.set_mode("LAND")
        v.wait_until(["HEARTBEAT"],
                     lambda h: h.get_srcComponent() == 1 and
                               not (h.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                     180, "landed")
    finally:
        stop(v.proc)
        stop(sim)

    ok = wall_ok and gaps_ok and bound_ok and streamed
    print(f"M5 CORRIDOR: wall_tracking={'PASS' if wall_ok else 'FAIL'} "
          f"gaps={'PASS' if gaps_ok else 'FAIL'} "
          f"memory_bound={'PASS' if bound_ok else 'FAIL'} streamed={streamed}")
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
