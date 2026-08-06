#!/usr/bin/env python3
"""Read-only diagnostic: write can.config.baud_rate on node 0 only (no
save_config, so nothing persists or reboots), then wait and retry reads
to see if the node goes silent briefly and recovers, or never responds."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # so this can find src/ when run from tools/
import time
import can
from src.configure import load_endpoints, read_config, write_config

NODE_ID = 0

def main():
    bus = can.interface.Bus("can0", interface="socketcan")
    endpoints = load_endpoints()['endpoints']
    ep = endpoints['can.config.baud_rate']

    print("Before write:", read_config(bus, NODE_ID, ep['id'], ep['type']))

    write_config(bus, NODE_ID, ep['id'], ep['type'], 500000)
    print("Write sent. Polling for a response over the next 3s...")

    for i in range(10):
        time.sleep(0.3)
        val = read_config(bus, NODE_ID, ep['id'], ep['type'])
        print(f"  t+{(i+1)*0.3:.1f}s: {val}")

    bus.shutdown()

if __name__ == "__main__":
    main()
