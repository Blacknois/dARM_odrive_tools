#!/usr/bin/env python3
"""
Read-only. Reads axis0.config.watchdog_timeout and axis0.config.enable_watchdog
for every node, just to see the current baseline before changing anything on
node 7 specifically. Does not arm or move anything.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # so this can find src/ when run from tools/
import can
from src.configure import load_endpoints, read_config
from src.can_utils import discover_node_ids

bus = can.interface.Bus("can0", interface="socketcan")
try:
    node_ids = sorted(discover_node_ids(bus))
    endpoints = load_endpoints()
    to_ep = endpoints["endpoints"]["axis0.config.watchdog_timeout"]
    en_ep = endpoints["endpoints"]["axis0.config.enable_watchdog"]

    print(f"{'node':>4}  {'enable_watchdog':>16}  {'watchdog_timeout':>16}")
    for nid in node_ids:
        enabled = read_config(bus, nid, en_ep['id'], en_ep['type'])
        timeout = read_config(bus, nid, to_ep['id'], to_ep['type'])
        print(f"{nid:>4}  {str(enabled):>16}  {str(timeout):>16}")
finally:
    bus.shutdown()
