#!/usr/bin/env python3
"""
Reads active_errors and disarm_reason for every discovered node, to find
out which nodes faulted (and with what code) during the last console.py
session. Read-only.
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

    err_ep    = endpoints['axis0.active_errors']
    disarm_ep = endpoints['axis0.disarm_reason']

    for nid in node_ids:
        err    = read_config(bus, nid, err_ep['id'], err_ep['type'])
        disarm = read_config(bus, nid, disarm_ep['id'], disarm_ep['type'])
        flag = "  <-- non-zero!" if (err or disarm) else ""
        print(f"Node {nid}: active_errors={err}  disarm_reason={disarm}{flag}")

    bus.shutdown()

if __name__ == "__main__":
    main()
