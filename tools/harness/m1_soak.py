#!/usr/bin/env python3
"""M1 acceptance soak: real arducopter --model json against skysim's canned physics.

Asserts (docs/MILESTONES.md M1):
  - clock slaved: no "time moved backwards" / timestamp warning spam in SITL output
  - EKF origin set (STATUSTEXT)
  - heartbeat stable for 60 s (no gap > 2 s)

Usage:
  python3 tools/harness/m1_soak.py --binary $ARDUPILOT_ROOT/build/sitl/bin/arducopter \
      [--sim ./build/skysim] [--instance 1] [--duration 60]

Note: instance defaults to 1 — on hosts where an unrelated SITL owns the -I0 ports.
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import time

from pymavlink import mavutil  # pip install pymavlink

WARN_RE = re.compile(
    r"time moved backwards|Did not contain all mandatory fields|"
    r"Failed to parse|Failed to find key|behind by \d",
    re.IGNORECASE,
)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--binary", default=os.path.expandvars("$ARDUPILOT_ROOT/build/sitl/bin/arducopter"))
    ap.add_argument("--sim", default="./build/skysim")
    ap.add_argument("--instance", type=int, default=1)
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--home", default="42.1354,24.7453,164,0")
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    defaults = ",".join([
        os.path.expandvars("$ARDUPILOT_ROOT/Tools/autotest/default_params/copter.parm"),
        os.path.join(here, "params", "skysim.parm"),
    ])

    sim = subprocess.Popen(
        [args.sim, "--vehicles", "1", "--base-instance", str(args.instance), "--time-mode", "strict"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    time.sleep(0.5)
    sitl = subprocess.Popen(
        [args.binary, "--model", "json:127.0.0.1", "-I", str(args.instance),
         "--home", args.home, "--defaults", defaults],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )

    ekf_origin = False
    warnings: list[str] = []
    heartbeats = 0
    max_gap = 0.0
    try:
        mav = mavutil.mavlink_connection(f"tcp:127.0.0.1:{5760 + 10 * args.instance}", retries=30)
        mav.wait_heartbeat(timeout=60)
        print("heartbeat: first received")
        mav.mav.request_data_stream_send(mav.target_system, mav.target_component,
                                         mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)

        t_end = time.time() + args.duration
        last_hb = time.time()
        last_gcs_hb = 0.0
        while time.time() < t_end:
            now = time.time()
            if now - last_gcs_hb > 1.0:  # AP wants a live GCS on the link
                mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                       mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                last_gcs_hb = now
            msg = mav.recv_match(type=["HEARTBEAT", "STATUSTEXT", "GLOBAL_POSITION_INT"],
                                 blocking=True, timeout=3)
            now = time.time()
            if msg is None:
                continue
            if msg.get_type() == "HEARTBEAT" and msg.get_srcComponent() == 1:
                heartbeats += 1
                max_gap = max(max_gap, now - last_hb)
                last_hb = now
            elif msg.get_type() == "STATUSTEXT":
                text = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="replace")
                print(f"  statustext: {text}")
                if "origin set" in text.lower():
                    ekf_origin = True
            elif msg.get_type() == "GLOBAL_POSITION_INT":
                # lat/lon stay 0 until the EKF has an origin; home is at ~42.1354N.
                if not ekf_origin and abs(msg.lat) > 10_000_000:
                    ekf_origin = True
                    print(f"  ekf origin via GLOBAL_POSITION_INT: lat={msg.lat} lon={msg.lon}")
    finally:
        sitl.terminate()
        sim.terminate()
        try:
            sitl_out = sitl.communicate(timeout=10)[0] or ""
        except subprocess.TimeoutExpired:
            sitl.kill()
            sitl_out = sitl.communicate()[0] or ""
        try:
            sim_out = sim.communicate(timeout=10)[0] or ""
        except subprocess.TimeoutExpired:
            sim.kill()
            sim_out = sim.communicate()[0] or ""

    for line in sitl_out.splitlines():
        if WARN_RE.search(line):
            warnings.append(line)

    print("\n--- skysim output ---")
    print(sim_out.strip())
    print("--- results ---")
    print(f"heartbeats={heartbeats} over {args.duration:.0f}s, max_gap={max_gap:.2f}s")
    print(f"ekf_origin_set={ekf_origin}")
    print(f"sitl_warnings={len(warnings)}")
    for w in warnings[:10]:
        print(f"  WARN: {w}")

    ok = heartbeats >= args.duration * 0.8 and max_gap < 2.0 and ekf_origin and not warnings
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
