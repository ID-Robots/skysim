#!/usr/bin/env python3
"""M3 determinism acceptance: strict mode + fixed seed => identical trajectory hashes.

1. Fly a real SITL sortie (takeoff -> hover -> land) against skysim with wind + seeded
   gusts, recording every consumed servo frame (--record-servo) and the ground-truth
   trajectory (--truth-log).
2. Replay the recorded servo stream through skysim twice (no network, same seed).
3. PASS iff replay A and replay B truth logs are bit-identical, AND both match the live
   run's truth log (catches hidden wall-clock dependence in the physics path).

Usage: python3 tools/harness/determinism.py [--base-instance 1] [--seed 42]
"""
from __future__ import annotations

import argparse
import hashlib
import os
import subprocess
import sys
import time

from pymavlink import mavutil

from conformance import Vehicle, launch_sitl, stop, wait_ready

WIND_ARGS = ["--wind", "4,1,0", "--gust", "1.5,2.0"]


def sha256(path: str) -> str:
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--binary",
                    default=os.path.expandvars("$ARDUPILOT_ROOT/build/sitl/bin/arducopter"))
    ap.add_argument("--sim", default="./build/skysim")
    ap.add_argument("--base-instance", type=int, default=1)
    ap.add_argument("--seed", default="42")
    ap.add_argument("--home", default="42.1354,24.7453,164,0")
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    defaults = ",".join([
        os.path.expandvars("$ARDUPILOT_ROOT/Tools/autotest/default_params/copter.parm"),
        os.path.join(here, "params", "skysim.parm"),
        os.path.join(here, "params", "rangefinder.parm"),
    ])
    rec = os.path.abspath("determinism_servo.csv")
    truth_live = os.path.abspath("determinism_truth_live.csv")

    sim_base = [args.sim, "--vehicles", "1", "--time-mode", "strict", "--dt", "0.00125",
                "--seed", args.seed, *WIND_ARGS]
    sim = subprocess.Popen(sim_base + ["--base-instance", str(args.base_instance),
                                       "--rangefinder",
                                       "--record-servo", rec, "--truth-log", truth_live])
    time.sleep(0.5)
    v = Vehicle(args.base_instance,
                launch_sitl(args.binary, args.base_instance, args.home, defaults))
    rng_ok = False
    try:
        wait_ready(v)
        m = v.mav
        m.set_mode("GUIDED")
        v.wait_until(["HEARTBEAT"],
                     lambda h: h.get_srcComponent() == 1 and h.custom_mode == 4, 30, "GUIDED")
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
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
        v.wait_until(["GLOBAL_POSITION_INT"],
                     lambda p: abs(p.relative_alt / 1000.0 - 10) < 1.0, 90, "takeoff 10 m")
        # Hover in gusts; sample the SITL rangefinder (fed by skysim rng_1) meanwhile.
        # Copter-4.7 streams DISTANCE_SENSOR (cm), not the legacy RANGEFINDER message.
        rng_samples = []
        t_end = time.time() + 5
        while time.time() < t_end:
            msg = v.poll(["DISTANCE_SENSOR"], timeout=1)
            if msg is not None:
                rng_samples.append(msg.current_distance / 100.0)
        if rng_samples:
            avg = sum(rng_samples) / len(rng_samples)
            print(f"rangefinder during 10 m hover: n={len(rng_samples)} avg={avg:.2f} m "
                  f"min={min(rng_samples):.2f} max={max(rng_samples):.2f}")
            rng_ok = 8.5 < avg < 11.5
        else:
            print("rangefinder: no RANGEFINDER messages received")
            rng_ok = False
        m.set_mode("LAND")
        v.wait_until(["HEARTBEAT"],
                     lambda h: h.get_srcComponent() == 1 and
                               not (h.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                     180, "landed + disarmed")
    finally:
        stop(v.proc)
        stop(sim)

    frames = sum(1 for _ in open(rec))
    print(f"recorded {frames} servo frames, live truth {sha256(truth_live)[:16]}...")

    hashes = []
    for run in ("a", "b"):
        out = os.path.abspath(f"determinism_truth_replay_{run}.csv")
        rc = subprocess.run(sim_base + ["--replay-servo", rec, "--truth-log", out],
                            capture_output=True, text=True)
        if rc.returncode != 0:
            print(f"replay {run} failed: {rc.stderr}")
            return 1
        hashes.append(sha256(out))
        print(f"replay {run}: {hashes[-1][:16]}...")

    live_hash = sha256(truth_live)
    replays_match = hashes[0] == hashes[1]
    live_matches = live_hash == hashes[0]
    print(f"replayA == replayB: {replays_match}")
    print(f"live == replay:     {live_matches}")
    print(f"rangefinder tracks hover altitude: {rng_ok}")
    ok = replays_match and live_matches and rng_ok
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
