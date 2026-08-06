#!/usr/bin/env python3
"""
Persist the restored trap_traj vel/accel/decel limits to flash on all 8
nodes via save_config(), following the same pattern already proven for
the broadcast-rate save (save_broadcast_changes.py). Nodes stay
disarmed throughout - config write only, no arming or motion. Each
node will briefly reboot to apply+persist the save, which is normal
ODrive behavior. After reboot, values are re-read and compared against
baseline; any node that doesn't match gets one retry.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # so this can find src/ when run from tools/
import time
import can
from src.configure import load_endpoints, read_config, save_config

NODE_IDS = [0, 1, 2, 3, 4, 5, 6, 7]
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
TOLERANCE = 1e-3
REBOOT_WAIT = 3

def check_all(bus, endpoints):
    vel_ep   = endpoints['axis0.trap_traj.config.vel_limit']
    accel_ep = endpoints['axis0.trap_traj.config.accel_limit']
    decel_ep = endpoints['axis0.trap_traj.config.decel_limit']
    results = {}
    for nid in NODE_IDS:
        vel   = read_config(bus, nid, vel_ep['id'], vel_ep['type'])
        accel = read_config(bus, nid, accel_ep['id'], accel_ep['type'])
        decel = read_config(bus, nid, decel_ep['id'], decel_ep['type'])
        target = BASELINE[nid]
        ok = (vel is not None and abs(vel - target[0]) <= TOLERANCE and
              accel is not None and abs(accel - target[1]) <= TOLERANCE and
              decel is not None and abs(decel - target[2]) <= TOLERANCE)
        results[nid] = (ok, vel, accel, decel)
    return results

def do_save(bus, node_ids, save_ep):
    for nid in node_ids:
        save_config(bus, nid, save_ep)
        time.sleep(0.3)

def main():
    bus = can.interface.Bus("can0", interface="socketcan")
    endpoints = load_endpoints()['endpoints']
    save_ep = endpoints['save_configuration']['id']

    print("=== Confirming current (RAM) trap_traj values before saving ===")
    pre = check_all(bus, endpoints)
    for nid, (ok, vel, accel, decel) in pre.items():
        print(f"  Node {nid}: vel={vel} accel={accel} decel={decel} {'OK' if ok else 'MISMATCH (unexpected pre-save state)'}")

    print("\n=== Saving configuration on all 8 nodes ===")
    do_save(bus, NODE_IDS, save_ep)

    print(f"\nWaiting {REBOOT_WAIT}s for nodes to reboot after save...")
    time.sleep(REBOOT_WAIT)

    print("\n=== Re-reading trap_traj values after save+reboot ===")
    post = check_all(bus, endpoints)
    failed = []
    for nid, (ok, vel, accel, decel) in post.items():
        status = "OK" if ok else "MISMATCH"
        print(f"  Node {nid}: vel={vel} accel={accel} decel={decel} -> {status}")
        if not ok:
            failed.append(nid)

    if failed:
        print(f"\n[WARNING] Nodes not confirmed persisted: {failed} - retrying once...")
        time.sleep(1)
        do_save(bus, failed, save_ep)
        print(f"Waiting {REBOOT_WAIT}s for retry reboot...")
        time.sleep(REBOOT_WAIT)
        retry = check_all(bus, endpoints)
        still_failed = [nid for nid in failed if not retry[nid][0]]
        for nid in failed:
            ok, vel, accel, decel = retry[nid]
            print(f"  Node {nid} retry: vel={vel} accel={accel} decel={decel} -> {'OK' if ok else 'STILL MISMATCH'}")
        if still_failed:
            print(f"\n[ERROR] Nodes still not confirmed after retry: {still_failed} - do not power off until resolved, check manually.")
        else:
            print("\nAll nodes confirmed persisted after retry.")
    else:
        print("\nAll nodes confirmed persisted to flash on first attempt.")

    bus.shutdown()

if __name__ == "__main__":
    main()
