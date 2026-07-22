#!/usr/bin/env python3
"""M4 acceptance: port/instance allocator leak-free across 100 spawn/despawn cycles.

Runs skysim with the control plane (no SITL processes — allocator + endpoint lifecycle is
what's under test), then loops POST /vehicles + DELETE /vehicles/{id}. PASS iff every spawn
succeeds, instances are recycled (max instance stays bounded), and the fleet ends empty.

Usage: python3 tools/harness/churn.py [--cycles 100] [--api-port 8642] [--base-instance 30]
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
import urllib.request


def api(port: int, method: str, path: str, body: bytes | None = None):
    req = urllib.request.Request(f"http://127.0.0.1:{port}{path}", data=body, method=method)
    with urllib.request.urlopen(req, timeout=15) as resp:
        return json.loads(resp.read().decode() or "{}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", default="./build/skysim")
    ap.add_argument("--cycles", type=int, default=100)
    ap.add_argument("--api-port", type=int, default=8642)
    ap.add_argument("--base-instance", type=int, default=30)
    args = ap.parse_args()

    sim = subprocess.Popen([args.sim, "--vehicles", "0", "--base-instance", str(args.base_instance),
                            "--time-mode", "interactive", "--api-port", str(args.api_port)])
    time.sleep(1)
    max_instance = -1
    keep_id = None
    try:
        # One long-lived vehicle so churn happens *while the world keeps running*.
        keep_id = api(args.api_port, "POST", "/vehicles", b"{}")["id"]
        for cycle in range(args.cycles):
            spawned = api(args.api_port, "POST", "/vehicles", b"{}")
            max_instance = max(max_instance, spawned["instance"])
            api(args.api_port, "DELETE", f"/vehicles/{spawned['id']}")
            if (cycle + 1) % 25 == 0:
                m = api(args.api_port, "GET", "/metrics")
                print(f"cycle {cycle + 1}: last instance {spawned['instance']}, "
                      f"tick {m['tick']}, vehicles {m['vehicles']}")
        api(args.api_port, "DELETE", f"/vehicles/{keep_id}")
        vehicles = api(args.api_port, "GET", "/vehicles")
        metrics = api(args.api_port, "GET", "/metrics")
    finally:
        sim.terminate()
        try:
            sim.wait(timeout=10)
        except subprocess.TimeoutExpired:
            sim.kill()

    # keep-alive holds base_instance; churn should reuse base+1 forever.
    leak_free = max_instance == args.base_instance + 1
    empty = len(vehicles) == 0
    ticking = metrics["tick"] > 0
    print(f"cycles={args.cycles} max_instance={max_instance} (expected {args.base_instance + 1}) "
          f"fleet_empty={empty} world_ticked={ticking} tick={metrics['tick']}")
    ok = leak_free and empty and ticking
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
