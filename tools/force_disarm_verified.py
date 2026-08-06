#!/usr/bin/env python3
"""
Immediate, verified disarm: sends IDLE to every node and confirms via
is_armed read-back, retrying any node that doesn't confirm within a
few attempts. Prints a clear final status for every node.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # so this can find src/ when run from tools/
import time
import can
from src.configure import load_endpoints, read_config
from src.control import set_idle_mode

NODE_IDS = [0, 1, 2, 3, 4, 5, 6, 7]
RETRIES = 5
SETTLE_TIMEOUT = 0.3

def main():
    bus = can.interface.Bus("can0", interface="socketcan")
    endpoints = load_endpoints()['endpoints']
    armed_ep = endpoints['axis0.is_armed']

    results = {}
    for nid in NODE_IDS:
        confirmed = False
        for attempt in range(RETRIES):
            set_idle_mode(bus, nid)
            start = time.time()
            while time.time() - start < SETTLE_TIMEOUT:
                armed = read_config(bus, nid, armed_ep['id'], armed_ep['type'])
                if armed is not None and not armed:
                    confirmed = True
                    break
                time.sleep(0.05)
            if confirmed:
                break
        results[nid] = confirmed
        status = "DISARMED (confirmed)" if confirmed else "STILL ARMED - retries exhausted"
        print(f"Node {nid}: {status}")

    print()
    if all(results.values()):
        print("All 8 nodes confirmed disarmed.")
    else:
        failed = [nid for nid, ok in results.items() if not ok]
        print(f"[WARNING] Nodes still armed after {RETRIES} retries each: {failed}")

    bus.shutdown()

if __name__ == "__main__":
    main()
