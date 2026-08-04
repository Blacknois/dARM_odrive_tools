#!/usr/bin/env python3
"""
Change can.config.baud_rate on all 8 nodes, for A/B testing whether the
persistent CAN bus errors (task #28) are tied to running at 1Mbps.

Uses the same safe write+validate+save pattern as setup_odrive()/
set_odrive_parameter() (src/configure.py), but ONLY touches this one
config path - it does NOT reapply the full motor/tuning profile, so it
can't clobber later per-node calibration (e.g. the tightened clamps on
node 3 and nodes 5/6).

Nodes stay disarmed throughout - config write only, no motion. Each
node will briefly reboot to persist the change (normal ODrive behavior,
same as save_broadcast_changes.py).

IMPORTANT: after this script finishes, the Pi's own can0 interface is
still on the OLD bitrate and must be manually switched to match:
    sudo ip link set can0 down
    sudo ip link set can0 type can bitrate <NEW_BAUD>
    sudo ip link set can0 up
before you can talk to the nodes again.

Usage:
    python3 set_can_baudrate.py <new_baud>
Example (test):   python3 set_can_baudrate.py 500000
Example (revert): python3 set_can_baudrate.py 1000000
"""
import sys
import time
import can
from src.configure import load_endpoints, set_odrive_parameter, save_config

NODE_IDS = [0, 1, 2, 3, 4, 5, 6, 7]
PATH = "can.config.baud_rate"

def main():
    if len(sys.argv) != 2:
        print(f"Usage: python3 {sys.argv[0]} <new_baud>")
        sys.exit(1)
    new_baud = int(sys.argv[1])

    bus = can.interface.Bus("can0", interface="socketcan")
    endpoints = load_endpoints()

    print(f"=== Writing {PATH} = {new_baud} to nodes {NODE_IDS} ===")
    ok_nodes = []
    for nid in NODE_IDS:
        ok = set_odrive_parameter(bus, nid, PATH, new_baud, endpoints)
        if ok:
            ok_nodes.append(nid)
        else:
            print(f"[ERROR] Node {nid} failed to accept new baud rate - NOT saving this node.")

    if not ok_nodes:
        print("No nodes accepted the change. Aborting - nothing saved.")
        bus.shutdown()
        return

    print(f"\n=== Saving configuration on nodes {ok_nodes} (each will briefly reboot) ===")
    save_ep = endpoints['endpoints']['save_configuration']['id']
    for nid in ok_nodes:
        save_config(bus, nid, save_ep)
        time.sleep(0.3)

    print("\nWaiting 3s for nodes to reboot...")
    time.sleep(3)

    bus.shutdown()

    print(f"""
Done. Nodes {ok_nodes} have been told to persist baud_rate={new_baud} and have rebooted.

NEXT STEP (required before you can talk to them again):
    sudo ip link set can0 down
    sudo ip link set can0 type can bitrate {new_baud}
    sudo ip link set can0 up

Then confirm with:
    python3 diag_can_health.py
""")

if __name__ == "__main__":
    main()
