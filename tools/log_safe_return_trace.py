#!/usr/bin/env python3
"""
Logs live axis0.pos_estimate for ALL nodes (0-7) at a fixed rate while
a PS-tap staged safe-return sequence runs, so the actual timing of each
stage can be examined afterward instead of relying on eyeballing 5+
nodes moving simultaneously. Read-only - no motor commands sent by this
script; the actual sequence is triggered normally via the real
gamecontroller.py by DrJones.

Built 2026-08-10 to investigate a real near-miss: PS-tap reached
safe_up_pos, but the forearm appeared to swing toward the floor during
the fold-down-to-rest_pos stage, suggesting Stage 5 (un-spin
base/elbow-roll/wrist-rotate) may not have actually finished before
Stage 6 (fold everything to rest_pos) began - wait_for_position()'s
return value is discarded at the Stage 5 call site in
run_safe_return_sequence(), so a timeout there wouldn't block Stage 6
from starting.

Writes a JSON file with all samples at the end. Ctrl+C stops early and
still saves whatever was captured.

Usage: python3 log_safe_return_trace.py [duration_seconds] [hz] [out_file]
"""
import sys, os, time, json, subprocess
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import can
from src.configure import load_endpoints, read_config

RESET_SCRIPT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "reset_interfaces.sh")
NODES = [0, 1, 2, 3, 4, 5, 6, 7]

def is_bus_off():
    try:
        out = subprocess.run(["ip", "-details", "link", "show", "can0"],
                              capture_output=True, text=True, timeout=5).stdout
        return "BUS-OFF" in out
    except Exception:
        return False

def hard_reset_interface():
    print("  -> genuine BUS-OFF detected, running reset_interfaces.sh (needs a moment)...")
    subprocess.run(["bash", RESET_SCRIPT], capture_output=True, timeout=30)
    time.sleep(2)

def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    hz = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
    out_file = sys.argv[3] if len(sys.argv) > 3 else "safe_return_trace.json"
    period = 1.0 / hz

    def open_bus():
        return can.interface.Bus("can0", interface="socketcan")

    bus = open_bus()
    endpoints = load_endpoints()['endpoints']
    pos_ep = endpoints['axis0.pos_estimate']

    print(f"Logging nodes {NODES} pos_estimate for {duration:.0f}s at {hz:g}Hz -> {out_file}")
    print("Read-only - no motor commands sent by this script. Ctrl+C to stop early.\n")
    header = "  ".join(f"n{n:>6}" for n in NODES)
    print(f"{'t(s)':>7}  {header}")

    samples = []
    consecutive_fails = 0
    start = time.time()
    try:
        while True:
            now = time.time()
            elapsed = now - start
            if elapsed > duration:
                break
            row = {}
            ok = True
            try:
                for n in NODES:
                    row[n] = read_config(bus, n, pos_ep['id'], pos_ep['type'])
                    if row[n] is None:
                        ok = False
            except (can.CanError, OSError) as e:
                print(f"{elapsed:7.1f}  CAN ERROR ({e}) - reopening bus...")
                try:
                    bus.shutdown()
                except Exception:
                    pass
                time.sleep(1.0)
                bus = open_bus()
                ok = False
            if ok:
                sample = {"t": elapsed}
                sample.update({f"n{n}": row[n] for n in NODES})
                samples.append(sample)
                vals = "  ".join(f"{row[n]:7.3f}" for n in NODES)
                print(f"{elapsed:7.2f}  {vals}")
                consecutive_fails = 0
            else:
                print(f"{elapsed:7.2f}  READ-FAIL")
                consecutive_fails += 1
                if consecutive_fails >= 10:
                    print(f"{elapsed:7.2f}  {consecutive_fails} consecutive fails - checking interface...")
                    try:
                        bus.shutdown()
                    except Exception:
                        pass
                    if is_bus_off():
                        hard_reset_interface()
                    else:
                        time.sleep(1.0)
                    bus = open_bus()
                    consecutive_fails = 0
            sleep_left = period - (time.time() - now)
            if sleep_left > 0:
                time.sleep(sleep_left)
    except KeyboardInterrupt:
        print("\nStopped early by user.")
    finally:
        bus.shutdown()

    with open(out_file, "w") as f:
        json.dump({"samples": samples}, f, indent=2)
    print(f"\nSaved {len(samples)} samples to {out_file}")

if __name__ == "__main__":
    main()
