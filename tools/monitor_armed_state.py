#!/usr/bin/env python3
"""
Watches axis0.is_armed on all discovered nodes at high frequency and
prints a line every time ANY node's armed state changes, with the
elapsed time since the script started. Meant to be started right before
triggering PS-hold (or any disarm event) to get a direct, timestamped
CAN-level record of exactly which nodes disarmed and how fast - not
just a visual/physical inference. Read-only, safe.

Usage: python3 monitor_armed_state.py [duration_seconds] [hz]
Example (watch for 15s at 10Hz): python3 monitor_armed_state.py 15 10
"""
import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # find src/ from tools/
import can
from src.can_utils import discover_node_ids
from src.configure import load_endpoints, read_config

def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
    hz = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    period = 1.0 / hz

    bus = can.interface.Bus("can0", interface="socketcan")
    endpoints = load_endpoints()['endpoints']
    armed_ep = endpoints['axis0.is_armed']
    node_ids = sorted(discover_node_ids(bus))
    print(f"Discovered: {node_ids}")

    last_state = {}
    for n in node_ids:
        v = read_config(bus, n, armed_ep['id'], armed_ep['type'])
        last_state[n] = v
    print(f"Starting armed state: {last_state}")
    print(f"Watching for {duration:.0f}s at {hz:g}Hz. Trigger PS-hold now.\n")

    start = time.time()
    try:
        while True:
            now = time.time()
            elapsed = now - start
            if elapsed > duration:
                break
            for n in node_ids:
                v = read_config(bus, n, armed_ep['id'], armed_ep['type'])
                if v is not None and v != last_state[n]:
                    print(f"t={elapsed:6.2f}s  node {n}: {last_state[n]} -> {v}")
                    last_state[n] = v
            sleep_left = period - (time.time() - now)
            if sleep_left > 0:
                time.sleep(sleep_left)
    except KeyboardInterrupt:
        print("\nStopped early by user.")
    finally:
        bus.shutdown()

    print(f"\nFinal armed state: {last_state}")

if __name__ == "__main__":
    main()
