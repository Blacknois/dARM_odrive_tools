#!/usr/bin/env python3
"""
Walks through each safe_up_pos pose one at a time. Press Enter, then
you get a countdown to physically pose the arm (both hands free to
support it since nodes are unarmed/no holding torque), and it captures
the live position automatically at the end of the countdown.
Read-only - does not arm or move anything.
"""
import time
import can
from src.configure import load_endpoints, read_config

STEPS = [
    ("Shoulder vertical", [1, 2]),
    ("Elbow vertical", [4]),
    ("Wrist bend vertical", [5, 6]),
]
COUNTDOWN = 6

def main():
    bus = can.interface.Bus("can0", interface="socketcan")
    endpoints = load_endpoints()['endpoints']
    pos_ep = endpoints['axis0.pos_estimate']

    results = {}
    for label, node_ids in STEPS:
        input(f"\nPress Enter, then you'll have {COUNTDOWN}s to pose '{label}' (nodes {node_ids})...")
        for i in range(COUNTDOWN, 0, -1):
            print(f"  capturing in {i}...  ", end="\r", flush=True)
            time.sleep(1)
        print("  capturing now...          ")
        for nid in node_ids:
            pos = read_config(bus, nid, pos_ep['id'], pos_ep['type'])
            results[nid] = pos
            print(f"  Node {nid}: {pos}")

    print("\n=== Summary (safe_up_pos candidates) ===")
    for nid, pos in results.items():
        print(f"Node {nid}: {pos}")

    bus.shutdown()

if __name__ == "__main__":
    main()
