#!/usr/bin/env python3
"""
Reads axis0.commutation_mapper.config.offset and offset_valid for every
discovered node, to check whether offset_valid correlates with which
nodes armed successfully (0,1,3) vs which didn't (2,4,5,6,7). Read-only.
"""
import can
from src.can_utils import discover_node_ids
from src.configure import load_endpoints, read_config

def main():
    bus = can.interface.Bus("can0", interface="socketcan")
    endpoints = load_endpoints()['endpoints']
    node_ids = sorted(discover_node_ids(bus))
    print(f"Discovered nodes: {node_ids}\n")

    offset_ep = endpoints['axis0.commutation_mapper.config.offset']
    valid_ep  = endpoints['axis0.commutation_mapper.config.offset_valid']

    for nid in node_ids:
        offset = read_config(bus, nid, offset_ep['id'], offset_ep['type'])
        valid  = read_config(bus, nid, valid_ep['id'], valid_ep['type'])
        print(f"Node {nid}: offset={offset}  offset_valid={valid}")

    bus.shutdown()

if __name__ == "__main__":
    main()
