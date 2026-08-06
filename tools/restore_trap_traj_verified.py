#!/usr/bin/env python3
"""
One-time manual restore of trap_traj vel/accel/decel limits, using
verified (write-then-read-back-then-retry) writes. Does not arm or
move any node - config writes only, RAM only (not saved to flash),
matching move_to_neutral_slowly()'s original not-persisted design.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # so this can find src/ when run from tools/
import time
import can
from src.configure import load_endpoints, read_config, write_config

BASELINE = {
    0: (5.0, 8.0, 8.0),
    1: (5.0, 8.0, 8.0),
    2: (5.0, 8.0, 8.0),
    3: (5.0, 8.0, 8.0),
    4: (5.0, 8.0, 8.0),
    5: (30.0, 20.0, 20.0),
    6: (30.0, 20.0, 20.0),
    7: (50.0, 20.0, 20.0),
}
RETRIES = 5
SETTLE_TIMEOUT = 0.3
TOLERANCE = 1e-3

def write_verified(bus, node_id, ep, value, label):
    for attempt in range(RETRIES):
        write_config(bus, node_id, ep['id'], ep['type'], value)
        start = time.time()
        while time.time() - start < SETTLE_TIMEOUT:
            readback = read_config(bus, node_id, ep['id'], ep['type'])
            if readback is not None and abs(readback - value) <= TOLERANCE:
                return True
            time.sleep(0.05)
    print(f"[ERROR] Node {node_id} {label}: could not confirm write of {value} after {RETRIES} attempts.")
    return False

def main():
    bus = can.interface.Bus("can0", interface="socketcan")
    endpoints = load_endpoints()['endpoints']
    vel_ep   = endpoints['axis0.trap_traj.config.vel_limit']
    accel_ep = endpoints['axis0.trap_traj.config.accel_limit']
    decel_ep = endpoints['axis0.trap_traj.config.decel_limit']

    all_ok = True
    for nid, (vel, accel, decel) in BASELINE.items():
        ok_v = write_verified(bus, nid, vel_ep, vel, "trap_vel")
        ok_a = write_verified(bus, nid, accel_ep, accel, "trap_accel")
        ok_d = write_verified(bus, nid, decel_ep, decel, "trap_decel")
        status = "OK" if (ok_v and ok_a and ok_d) else "FAILED"
        print(f"Node {nid}: {status} (target vel={vel} accel={accel} decel={decel})")
        all_ok = all_ok and ok_v and ok_a and ok_d

    print("\nAll nodes confirmed restored." if all_ok else "\n[WARNING] Some nodes not confirmed - re-run or check manually.")
    bus.shutdown()

if __name__ == "__main__":
    main()
