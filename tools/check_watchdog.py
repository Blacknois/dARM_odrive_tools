#!/usr/bin/env python3
"""
Reads axis0.config.enable_watchdog and axis0.config.watchdog_timeout for
every discovered node, to compare nodes that armed successfully against
ones that didn't. Read-only - safe regardless of power/arm status.
"""
import can
from src.can_utils import discover_node_ids
from src.configure import load_endpoints, read_config

def main():
    bus = can.interface.Bus("can0", interface="socketcan")
    endpoints = load_endpoints()
    node_ids = sorted(discover_node_ids(bus))
    print(f"Discovered nodes: {node_ids}\n")

    wd_enable_ep  = endpoints['endpoints'].get('axis0.config.enable_watchdog')
    wd_timeout_ep = endpoints['endpoints'].get('axis0.config.watchdog_timeout')

    if wd_enable_ep is None:
        print("No 'axis0.config.enable_watchdog' endpoint found in flat_endpoints.json")
        bus.shutdown()
        return

    for nid in node_ids:
        enabled = read_config(bus, nid, wd_enable_ep['id'], wd_enable_ep['type'])
        timeout = None
        if wd_timeout_ep is not None:
            timeout = read_config(bus, nid, wd_timeout_ep['id'], wd_timeout_ep['type'])
        print(f"Node {nid}: enable_watchdog={enabled}  watchdog_timeout={timeout}")

    bus.shutdown()

if __name__ == "__main__":
    main()
