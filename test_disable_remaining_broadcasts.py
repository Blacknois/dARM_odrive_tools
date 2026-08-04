#!/usr/bin/env python3
"""
Disable iq_msg_rate_ms, torques_msg_rate_ms, and bus_voltage_msg_rate_ms
on all 8 nodes (RAM only - NOT saved to flash yet). Then actively read
torque_estimate and vbus_voltage via read_config to confirm the
underlying values are still live. No arming, no motion.
"""
import time
import can
from src.configure import load_endpoints, read_config, write_config

NODE_IDS = [0, 1, 2, 3, 4, 5, 6, 7]

RATE_PATHS = [
    "axis0.config.can.iq_msg_rate_ms",
    "axis0.config.can.torques_msg_rate_ms",
    "axis0.config.can.bus_voltage_msg_rate_ms",
]

def main():
    bus = can.interface.Bus("can0", interface="socketcan")
    endpoints = load_endpoints()['endpoints']

    print("=== Disabling remaining unused broadcasts (RAM only, not saved) ===")
    for path in RATE_PATHS:
        ep = endpoints[path]
        before = [read_config(bus, nid, ep['id'], ep['type']) for nid in NODE_IDS]
        for nid in NODE_IDS:
            write_config(bus, nid, ep['id'], ep['type'], 0)
        after = [read_config(bus, nid, ep['id'], ep['type']) for nid in NODE_IDS]
        print(f"  {path}")
        print(f"    before: {before}")
        print(f"    after:  {after}")

    torque_ep = endpoints['axis0.motor.torque_estimate']
    volts_ep  = endpoints['vbus_voltage']

    print("\n=== Confirm torque_estimate still reads live (3 samples/node) ===")
    for nid in NODE_IDS:
        samples = []
        for _ in range(3):
            samples.append(read_config(bus, nid, torque_ep['id'], torque_ep['type']))
            time.sleep(0.15)
        print(f"  Node {nid}: {samples}")

    print("\n=== Confirm vbus_voltage still reads live (3 samples/node) ===")
    for nid in NODE_IDS:
        samples = []
        for _ in range(3):
            samples.append(read_config(bus, nid, volts_ep['id'], volts_ep['type']))
            time.sleep(0.15)
        print(f"  Node {nid}: {samples}")

    print("\nDone. Nothing saved to flash yet - power-cycling reverts all of")
    print("today's rate changes instantly if anything looks wrong.")
    bus.shutdown()

if __name__ == "__main__":
    main()
