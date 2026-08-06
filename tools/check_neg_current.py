#!/usr/bin/env python3
"""
Reads config.dc_max_negative_current (and brake_resistor0.enable for
context) for every discovered node. Read-only, safe regardless of
power/arm status.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # so this can find src/ when run from tools/
import can
from src.can_utils import discover_node_ids
from src.configure import load_endpoints, read_config

def main():
    bus = can.interface.Bus("can0", interface="socketcan")
    endpoints = load_endpoints()['endpoints']
    node_ids = sorted(discover_node_ids(bus))
    print(f"Discovered nodes: {node_ids}\n")

    neg_ep   = endpoints.get('config.dc_max_negative_current')
    brake_ep = endpoints.get('config.brake_resistor0.enable')

    if neg_ep is None:
        print("No 'config.dc_max_negative_current' endpoint found in flat_endpoints.json")
        bus.shutdown()
        return

    for nid in node_ids:
        neg = read_config(bus, nid, neg_ep['id'], neg_ep['type'])
        brake = None
        if brake_ep is not None:
            brake = read_config(bus, nid, brake_ep['id'], brake_ep['type'])
        print(f"Node {nid}: dc_max_negative_current={neg}  brake_resistor0.enable={brake}")

    bus.shutdown()

if __name__ == "__main__":
    main()
