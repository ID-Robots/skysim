#!/usr/bin/env python3
"""M4 acceptance: straggler policy under SIGSTOP.

Interactive mode: two real SITLs connect; SIGSTOP one -> the world keeps ticking, the victim
freezes and is flagged in GET /vehicles; SIGCONT -> it thaws and both keep heartbeating.
Strict mode: the same SIGSTOP makes skysim abort with a straggler report (exit code 3).

Usage: python3 tools/harness/straggler.py [--base-instance 1] [--api-port 8643]
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
import urllib.request

from conformance import Vehicle, launch_sitl, stop, wait_ready


def api(port: int, path: str):
    with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=10) as resp:
        return json.loads(resp.read().decode())


def defaults_list() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return ",".join([
        os.path.expandvars("$ARDUPILOT_ROOT/Tools/autotest/default_params/copter.parm"),
        os.path.join(here, "params", "skysim.parm"),
    ])


def interactive_case(args) -> bool:
    print("=== interactive: SIGSTOP -> freeze+flag, SIGCONT -> thaw ===")
    sim = subprocess.Popen([args.sim, "--vehicles", "2", "--base-instance", str(args.base_instance),
                            "--time-mode", "interactive", "--dt", "0.00125",
                            "--api-port", str(args.api_port), "--hold-ticks", "3",
                            "--grace", "120", "--spacing", "30"])
    time.sleep(0.5)
    vehicles = [Vehicle(args.base_instance + i,
                        launch_sitl(args.binary, args.base_instance + i, args.home, defaults_list()))
                for i in range(2)]
    ok = False
    try:
        for v in vehicles:
            wait_ready(v)
        victim = vehicles[1]

        tick_before = api(args.api_port, "/metrics")["tick"]
        victim.proc.send_signal(signal.SIGSTOP)
        time.sleep(3)
        m_stopped = api(args.api_port, "/metrics")
        infos = {v["instance"]: v for v in api(args.api_port, "/vehicles")}
        frozen_flagged = infos[victim.instance]["frozen"]
        other_fine = not infos[vehicles[0].instance]["frozen"]
        world_ticking = m_stopped["tick"] > tick_before + 1000  # ~>1.25 s of 800 Hz ticks
        print(f"  during SIGSTOP: world_ticking={world_ticking} "
              f"(tick {tick_before} -> {m_stopped['tick']}), victim_frozen={frozen_flagged}, "
              f"other_ok={other_fine}, freezes={m_stopped['freezes']}")

        victim.proc.send_signal(signal.SIGCONT)
        time.sleep(4)
        infos = {v["instance"]: v for v in api(args.api_port, "/vehicles")}
        thawed = not infos[victim.instance]["frozen"]
        victim.mav.wait_heartbeat(timeout=10)
        vehicles[0].mav.wait_heartbeat(timeout=10)
        print(f"  after SIGCONT: victim_thawed={thawed}, both heartbeating")
        ok = world_ticking and frozen_flagged and other_fine and thawed
    finally:
        for v in vehicles:
            stop(v.proc)
        stop(sim)
    print(f"  interactive: {'PASS' if ok else 'FAIL'}")
    return ok


def strict_case(args) -> bool:
    print("=== strict: SIGSTOP -> abort with straggler report ===")
    sim = subprocess.Popen([args.sim, "--vehicles", "2", "--base-instance", str(args.base_instance),
                            "--time-mode", "strict", "--dt", "0.00125",
                            "--strict-timeout", "5", "--spacing", "30"],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    time.sleep(0.5)
    vehicles = [Vehicle(args.base_instance + i,
                        launch_sitl(args.binary, args.base_instance + i, args.home, defaults_list()))
                for i in range(2)]
    ok = False
    try:
        for v in vehicles:
            wait_ready(v)
        vehicles[1].proc.send_signal(signal.SIGSTOP)
        try:
            rc = sim.wait(timeout=30)
        except subprocess.TimeoutExpired:
            rc = None
        out = ""
        if rc is not None:
            out = sim.communicate()[0] or ""
        aborted = rc == 3 and "STRICT ABORT" in out
        report_names_victim = f"instance {vehicles[1].instance}" in out
        print(f"  exit_code={rc} abort_report={aborted} names_victim={report_names_victim}")
        for line in out.splitlines():
            if "STRICT" in line or "no frame" in line:
                print(f"    {line}")
        ok = aborted and report_names_victim
    finally:
        vehicles[1].proc.send_signal(signal.SIGCONT)
        for v in vehicles:
            stop(v.proc)
        if sim.poll() is None:
            stop(sim)
    print(f"  strict: {'PASS' if ok else 'FAIL'}")
    return ok


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--binary",
                    default=os.path.expandvars("$ARDUPILOT_ROOT/build/sitl/bin/arducopter"))
    ap.add_argument("--sim", default="./build/skysim")
    ap.add_argument("--base-instance", type=int, default=1)
    ap.add_argument("--api-port", type=int, default=8643)
    ap.add_argument("--home", default="42.1354,24.7453,164,0")
    args = ap.parse_args()

    ok_interactive = interactive_case(args)
    time.sleep(2)  # let SITL TCP ports clear before the strict case reuses them
    ok_strict = strict_case(args)
    print(f"STRAGGLER: interactive={'PASS' if ok_interactive else 'FAIL'} "
          f"strict={'PASS' if ok_strict else 'FAIL'}")
    return 0 if (ok_interactive and ok_strict) else 1


if __name__ == "__main__":
    sys.exit(main())
