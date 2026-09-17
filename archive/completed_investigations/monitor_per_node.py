"""
Polls axis0.is_armed (a cheap, always-safe-to-read field) for every
node individually at ~1Hz, and prints a timestamped line only when a
node's success/fail status *changes* - so we can see exactly which
node(s) drop out during a fault, not just that "the bus" had a
problem. Read-only, safe. Runs until killed.
"""
import sys, os, time
from datetime import datetime
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import can
from src.configure import load_endpoints, read_config

NODE_IDS = [0, 1, 2, 3, 4, 5, 6, 7]

def open_bus():
    return can.interface.Bus("can0", interface="socketcan")

def main():
    bus = open_bus()
    endpoints = load_endpoints()['endpoints']
    armed_ep = endpoints['axis0.is_armed']
    last_ok = {n: None for n in NODE_IDS}

    print(f"[{datetime.now().strftime('%H:%M:%S')}] Starting per-node monitor for {NODE_IDS}")

    while True:
        for n in NODE_IDS:
            try:
                v = read_config(bus, n, armed_ep['id'], armed_ep['type'])
                ok = v is not None
            except (can.CanError, OSError) as e:
                ok = False
                try:
                    bus.shutdown()
                except Exception:
                    pass
                time.sleep(0.5)
                bus = open_bus()
            if ok != last_ok[n]:
                ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                print(f"[{ts}] node {n}: {'OK' if last_ok[n] is None else ('FAIL->OK' if ok else 'OK->FAIL')}")
                last_ok[n] = ok
        time.sleep(1.0)

if __name__ == "__main__":
    main()
