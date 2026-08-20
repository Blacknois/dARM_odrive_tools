#!/usr/bin/env python3
"""
monitor_move_velocity.py [duration_seconds] [hz]

Polls pos_stream_server.py's /positions endpoint (the same safe HTTP
source every other read-only tool uses - never touches CAN directly)
at a fixed rate and logs raw node positions + computed per-node
velocity over time. Meant to run WHILE a real IK-mode move is
triggered, to see exactly where/how deceleration kicks in - not just
the end state.

Read-only, does not arm/move/trigger anything.

Usage:
    python3 monitor_move_velocity.py 25 10   # 25s, 10Hz
Start this just BEFORE holding L1+Triangle, so it captures the whole
move including the ramp-up and any decel tail.
"""
import sys
import time
import json
import urllib.request

POSITIONS_URL = 'http://localhost:8080/positions'
NODES = ['0', '1', '3', '4', '5', '6']


def get_positions():
    with urllib.request.urlopen(POSITIONS_URL, timeout=1.0) as resp:
        return json.loads(resp.read())


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0
    hz = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    interval = 1.0 / hz

    print(f"Polling for {duration}s at {hz}Hz. Ctrl+C to stop early.")
    print(f"{'t':>6} " + " ".join(f"{'n'+n:>9}" for n in NODES) + " " + " ".join(f"{'v'+n:>8}" for n in NODES))

    start = time.time()
    last = None
    last_t = None
    rows = []
    try:
        while time.time() - start < duration:
            t = time.time() - start
            try:
                data = get_positions()
            except Exception as e:
                print(f"{t:6.2f}  (poll failed: {e})")
                time.sleep(interval)
                continue

            pos = {n: data.get(n, 0.0) or 0.0 for n in NODES}
            vel = {}
            if last is not None:
                dt = t - last_t
                for n in NODES:
                    vel[n] = (pos[n] - last[n]) / dt if dt > 0 else 0.0
            else:
                vel = {n: 0.0 for n in NODES}

            row = {'t': round(t, 3), 'pos': pos, 'vel': vel}
            rows.append(row)
            print(f"{t:6.2f} " + " ".join(f"{pos[n]:9.4f}" for n in NODES) + " " + " ".join(f"{vel[n]:8.3f}" for n in NODES))

            last = pos
            last_t = t
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nStopped early.")

    out_file = f"~/dARM/odrive_tools/velocity_log_{int(time.time())}.json".replace('~', __import__('os').path.expanduser('~'))
    with open(out_file, 'w') as f:
        json.dump(rows, f, indent=2)
    print(f"\nSaved {len(rows)} samples to {out_file}")


if __name__ == '__main__':
    main()
