#!/usr/bin/env python3
"""
Logs live axis0.pos_estimate for nodes 5 and 6 together at a fixed rate
while the wrist is manually hand-swept through its real range of motion
(disarmed - no motor commands sent, read-only). Meant to capture real
(node5, node6) pairs across the full physical range so the actual safe
boundary shape can be analyzed afterward, instead of assuming
independent rectangular BEND/ROTATE clamps.

Writes a JSON file with all samples at the end (and prints live values
so Carla/Claude can narrate the sweep - "now at full bend left" etc).

Usage: python3 log_wrist_sweep.py [duration_seconds] [hz] [out_file]
Example: python3 log_wrist_sweep.py 90 5 wrist_sweep_1.json
"""
import sys, os, time, json, subprocess
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # find src/ from tools/
import can
from src.configure import load_endpoints, read_config

RESET_SCRIPT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "reset_interfaces.sh")

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
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
    hz = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
    out_file = sys.argv[3] if len(sys.argv) > 3 else "wrist_sweep.json"
    period = 1.0 / hz

    def open_bus():
        return can.interface.Bus("can0", interface="socketcan")

    bus = open_bus()
    endpoints = load_endpoints()['endpoints']
    pos_ep = endpoints['axis0.pos_estimate']

    print(f"Logging nodes [5,6] pos_estimate for {duration:.0f}s at {hz:g}Hz -> {out_file}")
    print("Read-only - no motor commands sent. Ctrl+C to stop early.\n")
    print(f"{'t(s)':>7}  {'node5':>8}  {'node6':>8}")

    samples = []
    consecutive_fails = 0
    start = time.time()
    try:
        while True:
            now = time.time()
            elapsed = now - start
            if elapsed > duration:
                break
            try:
                n5 = read_config(bus, 5, pos_ep['id'], pos_ep['type'])
                n6 = read_config(bus, 6, pos_ep['id'], pos_ep['type'])
            except (can.CanError, OSError) as e:
                print(f"{elapsed:7.1f}  CAN ERROR ({e}) - reopening bus...")
                try:
                    bus.shutdown()
                except Exception:
                    pass
                time.sleep(1.0)
                bus = open_bus()
                n5 = n6 = None
            if n5 is not None and n6 is not None:
                samples.append({"t": elapsed, "node5": n5, "node6": n6})
                print(f"{elapsed:7.1f}  {n5:8.3f}  {n6:8.3f}")
                consecutive_fails = 0
            else:
                print(f"{elapsed:7.1f}  READ-FAIL")
                consecutive_fails += 1
                if consecutive_fails >= 10:
                    # 10 fails in a row (at 10Hz, ~1s) means something's
                    # actually wrong, not just a blip. Reopening the
                    # Python bus object alone can't fix a genuine
                    # BUS-OFF interface state - that needs the real
                    # ip-link-level reset (same as reset_interfaces.sh),
                    # so check for that specifically and run it if so.
                    print(f"{elapsed:7.1f}  {consecutive_fails} consecutive fails - checking interface...")
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
    if samples:
        n5s = [s["node5"] for s in samples]
        n6s = [s["node6"] for s in samples]
        print(f"node5 range: [{min(n5s):.3f}, {max(n5s):.3f}]")
        print(f"node6 range: [{min(n6s):.3f}, {max(n6s):.3f}]")

if __name__ == "__main__":
    main()
