#!/usr/bin/env python3
"""
Live-polls axis0.motor.foc.Iq_measured for one or more nodes at a fixed
rate and prints timestamped values (elapsed seconds, Iq in A per node,
plus the sum of all polled nodes). Read-only, safe regardless of
arm/power state - does not write anything to the ODrive.

At the end, prints per-node average/min/max plus the average of the
summed (combined) current, so a differential pair (e.g. nodes 1,2)
can be logged together in one pass instead of two separate holds.

Usage: python3 monitor_iq_live.py <node_id[,node_id...]> [duration_seconds] [hz]
Example (joint 1, both shoulder motors, 60s hold, 3Hz):
    python3 monitor_iq_live.py 1,2 60 3
"""
import sys, os, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # find src/ from tools/
import can
from src.configure import load_endpoints, read_config

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 monitor_iq_live.py <node_id[,node_id...]> [duration_seconds] [hz]")
        sys.exit(1)

    node_ids = [int(x) for x in sys.argv[1].split(",")]
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
    hz = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
    period = 1.0 / hz

    bus = can.interface.Bus("can0", interface="socketcan")
    endpoints = load_endpoints()['endpoints']
    iq_ep = endpoints['axis0.motor.foc.Iq_measured']

    print(f"Polling nodes {node_ids} Iq_measured for {duration:.0f}s at {hz:g}Hz. Ctrl+C to stop early.\n")
    header = "".join(f"{'node'+str(n)+' (A)':>14}" for n in node_ids)
    print(f"{'t(s)':>7}{header}{'sum (A)':>14}")

    history = {n: [] for n in node_ids}

    start = time.time()
    try:
        while True:
            now = time.time()
            elapsed = now - start
            if elapsed > duration:
                break
            row_vals = []
            total = 0.0
            any_fail = False
            for n in node_ids:
                iq = read_config(bus, n, iq_ep['id'], iq_ep['type'])
                if iq is None:
                    any_fail = True
                    row_vals.append("READ-FAIL")
                else:
                    history[n].append(iq)
                    total += iq
                    row_vals.append(f"{iq:.3f}")
            row = "".join(f"{v:>14}" for v in row_vals)
            sum_str = f"{total:.3f}" if not any_fail else "n/a"
            print(f"{elapsed:7.1f}{row}{sum_str:>14}")
            sleep_left = period - (time.time() - now)
            if sleep_left > 0:
                time.sleep(sleep_left)
    except KeyboardInterrupt:
        print("\nStopped early by user.")
    finally:
        bus.shutdown()

    print("\n--- summary ---")
    for n in node_ids:
        vals = history[n]
        if vals:
            print(f"node {n}: avg={sum(vals)/len(vals):.3f}A  min={min(vals):.3f}A  max={max(vals):.3f}A  n={len(vals)}")
        else:
            print(f"node {n}: no successful reads")
    if all(history[n] for n in node_ids) and len(node_ids) > 1:
        n_samples = min(len(history[n]) for n in node_ids)
        combined = [sum(history[n][i] for n in node_ids) for i in range(n_samples)]
        print(f"combined (sum of {node_ids}): avg={sum(combined)/len(combined):.3f}A  min={min(combined):.3f}A  max={max(combined):.3f}A")

if __name__ == "__main__":
    main()
