"""
can_health_check.py
Read-only CAN reliability test: repeatedly polls axis0.pos_estimate for
each given node over a fixed duration, and reports how many reads
failed outright (timeout / no response) vs succeeded, per node.
Does NOT arm or move anything.

Usage:
    python3 can_health_check.py 0 1          # test nodes 0 and 1 for 30s
    python3 can_health_check.py 0 1 60       # last arg = duration in seconds
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # so this can find src/ when run from tools/
import sys
import time
import can
from src.configure import load_endpoints, read_config

def main():
    args = sys.argv[1:]
    duration = 30.0
    if args and args[-1].replace('.', '', 1).isdigit():
        duration = float(args[-1])
        args = args[:-1]
    node_ids = [int(a) for a in args] if args else [0, 1]

    bus = can.interface.Bus("can0", interface="socketcan")
    try:
        endpoints = load_endpoints()["endpoints"]
        pos_ep = endpoints["axis0.pos_estimate"]
        attempts = {nid: 0 for nid in node_ids}
        failures = {nid: 0 for nid in node_ids}

        print(f"Polling nodes {node_ids} for {duration:.0f}s (read-only, no arming/motion)...")
        start = time.time()
        while time.time() - start < duration:
            for nid in node_ids:
                attempts[nid] += 1
                pos = read_config(bus, nid, pos_ep["id"], pos_ep["type"])
                if pos is None:
                    failures[nid] += 1
            time.sleep(0.02)

        print("\n--- CAN read reliability ---")
        for nid in node_ids:
            a, f = attempts[nid], failures[nid]
            rate = (f / a * 100.0) if a else 0.0
            print(f"Node {nid}: {a} attempts, {f} failed ({rate:.1f}% failure rate)")
    finally:
        bus.shutdown()

if __name__ == "__main__":
    main()
