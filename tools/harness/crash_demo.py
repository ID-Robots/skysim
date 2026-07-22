#!/usr/bin/env python3
"""Two-SITL mid-air collision demo: fly them into each other, detect the crash three ways.

Mission: both quads take off 30 m apart at the SAME altitude, then are commanded to swap
positions — a head-on pass along the same line. Position noise can make a pass miss
(the bodies are 36 cm wide), so the swap repeats until skysim's contact events fire.

Detection (all three reported, first two asserted):
  1. skysim physics:  GET /vehicles midair_collisions > 0 on both vehicles
  2. ground truth:    --truth-log minimum separation ~ body size at the moment of impact
  3. ArduPilot:       crash/EKF-failsafe STATUSTEXT and/or disarm after the tumble

Usage: python3 tools/harness/crash_demo.py [--base-instance 1] [--api-port 8646]
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import threading
import time
import urllib.request

from pymavlink import mavutil

from collision_demo import arm_guided, goto, takeoff
from conformance import Vehicle, launch_sitl, stop, wait_ready

ALT = 15.0
SPACING = 30.0
MAX_PASSES = 6


def api(port: int, path: str):
    with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=10) as resp:
        return json.loads(resp.read().decode())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--binary",
                    default=os.path.expandvars("$ARDUPILOT_ROOT/build/sitl/bin/arducopter"))
    ap.add_argument("--sim", default="./build/skysim")
    ap.add_argument("--base-instance", type=int, default=1)
    ap.add_argument("--api-port", type=int, default=8646)
    ap.add_argument("--home", default="42.1354,24.7453,164,0")
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    defaults = ",".join([
        os.path.expandvars("$ARDUPILOT_ROOT/Tools/autotest/default_params/copter.parm"),
        os.path.join(here, "params", "skysim.parm"),
    ])
    truth_csv = os.path.abspath(f"crash_demo_truth_I{args.base_instance}.csv")

    sim = subprocess.Popen([args.sim, "--vehicles", "2", "--base-instance",
                            str(args.base_instance), "--time-mode", "strict", "--dt", "0.00125",
                            "--spacing", str(SPACING), "--strict-timeout", "0",
                            "--api-port", str(args.api_port), "--truth-log", truth_csv])
    time.sleep(0.5)
    vehicles = [Vehicle(args.base_instance + i,
                        launch_sitl(args.binary, args.base_instance + i, args.home, defaults))
                for i in range(2)]
    sim_detected = False
    ap_reactions: list[str] = []
    both_disarmed = False
    try:
        # Boot + arm + climb concurrently: strict lockstep couples the two clocks.
        def prep(idx: int, v: Vehicle) -> None:
            wait_ready(v)
            arm_guided(v)
            takeoff(v, ALT)

        threads = [threading.Thread(target=prep, args=(i, v), daemon=True)
                   for i, v in enumerate(vehicles)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        a, b = vehicles
        print(f"both airborne at {ALT} m, {SPACING} m apart — commanding head-on passes")

        # Repeatedly swap positions until skysim's contact counter fires.
        for pass_no in range(MAX_PASSES):
            # A sits at east 0, B at east 30 (its home offset makes LOCAL (0,0) = spawn).
            goto(a, 0.0, SPACING, -ALT)   # A: charge to B's spawn column
            goto(b, 0.0, -SPACING, -ALT)  # B: LOCAL frame is offset by +30 east => east 0
            deadline = time.time() + 20
            while time.time() < deadline and not sim_detected:
                for v in vehicles:
                    v.poll(["HEARTBEAT"], timeout=0.3)
                infos = api(args.api_port, "/vehicles")
                if all(i["midair_collisions"] > 0 for i in infos):
                    sim_detected = True
            if sim_detected:
                print(f"  impact on pass {pass_no + 1}: skysim midair_collisions = "
                      f"{[i['midair_collisions'] for i in infos]}")
                break
            # Missed: swap back and try again.
            goto(a, 0.0, 0.0, -ALT)
            goto(b, 0.0, 0.0, -ALT)
            time.sleep(8)
            print(f"  pass {pass_no + 1}: clean miss, repositioning")

        # Watch the aftermath: ArduPilot should react (crash check / EKF failsafe / disarm).
        t_end = time.time() + 30
        while time.time() < t_end:
            for v in vehicles:
                v.poll(["HEARTBEAT"], timeout=0.3)
            armed = []
            for v in vehicles:
                hb = v.mav.recv_match(type="HEARTBEAT", blocking=False)
                if hb and hb.get_srcComponent() == 1:
                    armed.append(bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
            for v in vehicles:
                for s in v.statustexts:
                    low = s.lower()
                    if ("crash" in low or "failsafe" in low or "thrust loss" in low) and \
                            f"I{v.instance}:{s}" not in ap_reactions:
                        ap_reactions.append(f"I{v.instance}:{s}")
            if not armed or not any(armed):
                both_disarmed = True
                break
    finally:
        stop(sim)
        for v in vehicles:
            stop(v.proc)

    # Ground-truth minimum separation between the two vehicles.
    min_sep, min_t = 1e9, 0.0
    rows: dict[float, dict[int, list[float]]] = {}
    with open(truth_csv) as f:
        next(f)
        for line in f:
            t, vid, n, e, d = line.strip().split(",")
            rows.setdefault(float(t), {})[int(vid)] = [float(n), float(e), float(d)]
    for t, by_v in rows.items():
        if len(by_v) == 2:
            p0, p1 = by_v[0], by_v[1]
            sep = sum((x - y) ** 2 for x, y in zip(p0, p1)) ** 0.5
            if sep < min_sep:
                min_sep, min_t = sep, t

    print(f"\nDETECTION SUMMARY")
    print(f"  1. skysim contact events:  {'DETECTED' if sim_detected else 'none'}")
    print(f"  2. truth min separation:   {min_sep:.2f} m at t={min_t:.2f} s "
          f"(bodies touch at ~0.36 m)")
    print(f"  3. ArduPilot reactions:    {ap_reactions if ap_reactions else 'none captured'}"
          f"{'; all disarmed' if both_disarmed else ''}")
    ok = sim_detected and min_sep < 0.6
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
