#!/usr/bin/env python3
"""
Test: disable the encoder_msg_rate_ms periodic CAN broadcast on all 8
nodes (RAM only - NOT saved to flash, so a power-cycle instantly reverts
this), then actively read axis0.pos_estimate via read_config (the same
path get_metrics() uses) to confirm the underlying value is still live
and readable. No arming, no motion.
"""
import time
import can
from src.configure import load_endpoints, read_config, write_config

NODE_IDS = [0, 1, 2, 3, 4, 5, 6, 7]

def main():
    bus = can.interface.Bus("can0", interface="socketcan")
    endpoints = load_endpoints()['endpoints']

    rate_ep = endpoints['axis0.config.can.encoder_msg_rate_ms']
    pos_ep  = endpoints['axis0.pos_estimate']

    print("=== Step 1: current encoder_msg_rate_ms (before) ===")
    for nid in NODE_IDS:
        val = read_config(bus, nid, rate_ep['id'], rate_ep['type'])
        print(f"  Node {nid}: {val}")

    print("\n=== Step 2: disabling broadcast (writing 0, RAM only, not saved) ===")
    for nid in NODE_IDS:
        write_config(bus, nid, rate_ep['id'], rate_ep['type'], 0)

    print("\n=== Step 3: read back encoder_msg_rate_ms (confirm it's now 0) ===")
    for nid in NODE_IDS:
        val = read_config(bus, nid, rate_ep['id'], rate_ep['type'])
        print(f"  Node {nid}: {val}")

    print("\n=== Step 4: confirm axis0.pos_estimate still reads live via active polling ===")
    print("(3 samples per node, 0.2s apart - just needs to keep returning a valid float)")
    for nid in NODE_IDS:
        samples = []
        for _ in range(3):
            samples.append(read_config(bus, nid, pos_ep['id'], pos_ep['type']))
            time.sleep(0.2)
        print(f"  Node {nid}: {samples}")

    print("\nDone. encoder_msg_rate_ms was NOT saved to flash - power-cycling")
    print("reverts this instantly if you want to undo it.")
    bus.shutdown()

if __name__ == "__main__":
    main()
