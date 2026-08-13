#!/usr/bin/env python3
"""
Logs live axis0.pos_estimate and axis0.motor.foc.Iq_measured for node7
(gripper) at a fixed rate, so a real squeeze-to-skip test has an actual
current trace instead of just "it happened, no number attached".
Read-only - no motor commands sent by this script.

Built 2026-08-11 to bound the gripper rack-and-pinion tooth-skip
current at both extremes (fully open, fully closed) - DrJones drives
the gripper normally via the controller, this just watches and logs.

Usage: python3 log_gripper_current.py [duration_seconds] [hz] [out_file]
"""
import sys, os, time, json, subprocess
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
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
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    hz = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    out_file = sys.argv[3] if len(sys.argv) > 3 else "gripper_current_log.json"
    period = 1.0 / hz

    def open_bus():
        return can.interface.Bus("can0", interface="socketcan")

    bus = open_bus()
    endpoints = load_endpoints()['endpoints']
    pos_ep = endpoints['axis0.pos_estimate']
    iq_ep = endpoints['axis0.motor.foc.Iq_measured']

    print(f"Logging node7 pos + Iq for {duration:.0f}s at {hz:g}Hz -> {out_file}")
    print("Read-only - no motor commands sent. Ctrl+C to stop early.\n")
    print(f"{'t(s)':>7}  {'pos':>8}  {'Iq(A)':>8}")

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
                pos = read_config(bus, 7, pos_ep['id'], pos_ep['type'])
                iq = read_config(bus, 7, iq_ep['id'], iq_ep['type'])
            except (can.CanError, OSError) as e:
                print(f"{elapsed:7.1f}  CAN ERROR ({e}) - reopening bus...")
                try:
                    bus.shutdown()
                except Exception:
                    pass
                time.sleep(1.0)
                bus = open_bus()
                pos = iq = None
            if pos is not None and iq is not None:
                samples.append({"t": elapsed, "pos": pos, "iq": iq})
                flag = "  <-- HIGH" if abs(iq) > 0.8 else ""
                print(f"{elapsed:7.2f}  {pos:8.4f}  {iq:8.3f}{flag}")
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
    if samples:
        iqs = [abs(s["iq"]) for s in samples]
        print(f"max |Iq| observed: {max(iqs):.3f} A")

if __name__ == "__main__":
    main()
