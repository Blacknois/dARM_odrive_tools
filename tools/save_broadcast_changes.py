#!/usr/bin/env python3
"""
Persist tonight's disabled-broadcast-rate changes (encoder/iq/torques/
bus_voltage msg_rate_ms = 0) to flash on all 8 nodes via save_config().
Nodes stay disarmed throughout - this is a config write only, no
arming or motion. Each node will briefly reboot to apply+persist the
save, which is normal ODrive behavior.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # so this can find src/ when run from tools/
import time
import can
from src.configure import load_endpoints, read_config, save_config

NODE_IDS = [0, 1, 2, 3, 4, 5, 6, 7]

RATE_PATHS = [
    "axis0.config.can.encoder_msg_rate_ms",
    "axis0.config.can.iq_msg_rate_ms",
    "axis0.config.can.torques_msg_rate_ms",
    "axis0.config.can.bus_voltage_msg_rate_ms",
]

def main():
    bus = can.interface.Bus("can0", interface="socketcan")
    endpoints = load_endpoints()['endpoints']
    save_ep = endpoints['save_configuration']['id']

    print("=== Confirming current (RAM) rate values before saving ===")
    for path in RATE_PATHS:
        ep = endpoints[path]
        vals = [read_config(bus, nid, ep['id'], ep['type']) for nid in NODE_IDS]
        print(f"  {path}: {vals}")

    print("\n=== Saving configuration on all 8 nodes ===")
    for nid in NODE_IDS:
        save_config(bus, nid, save_ep)
        time.sleep(0.3)

    print("\nWaiting 3s for nodes to reboot after save...")
    time.sleep(3)

    print("\n=== Re-reading rate values after save+reboot (should still be 0) ===")
    for path in RATE_PATHS:
        ep = endpoints[path]
        vals = [read_config(bus, nid, ep['id'], ep['type']) for nid in NODE_IDS]
        print(f"  {path}: {vals}")

    bus.shutdown()

if __name__ == "__main__":
    main()
