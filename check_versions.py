#!/usr/bin/env python3
"""
Queries every discovered node's exact firmware/hardware version over CAN
(Get_Version, command id 0x00) and prints it, so we can compare the nodes
that armed successfully (0,1,3) against the ones that didn't (2,4,5,6,7).
Read-only - does not touch axis state, safe regardless of power/arm status.
"""
import struct
import can
from src.can_utils import discover_node_ids, send_can_message, receive_can_message

READ = 0x00

def get_version(bus, node_id):
    send_can_message(bus, node_id, READ, '')
    response = receive_can_message(bus, node_id << 5 | READ)
    if response is None:
        return None
    _, hw_product_line, hw_version, hw_variant, fw_major, fw_minor, fw_revision, fw_unreleased = struct.unpack('<BBBBBBBB', response.data)
    return {
        'hw': f"{hw_product_line}.{hw_version}.{hw_variant}",
        'fw': f"{fw_major}.{fw_minor}.{fw_revision}",
        'fw_unreleased': fw_unreleased,
    }

def main():
    bus = can.interface.Bus("can0", interface="socketcan")
    node_ids = sorted(discover_node_ids(bus))
    print(f"Discovered nodes: {node_ids}\n")

    for nid in node_ids:
        info = get_version(bus, nid)
        if info is None:
            print(f"Node {nid}: no response")
        else:
            print(f"Node {nid}: hw={info['hw']}  fw={info['fw']}  fw_unreleased={info['fw_unreleased']}")

    bus.shutdown()

if __name__ == "__main__":
    main()
