#!/usr/bin/env python3
"""
apply_gripper_pos_gain.py

Second real lever on node 7's (gripper) fault-on-grip problem, 2026-08-28.
The vel_integrator_limit fix (see apply_gripper_integrator_limit.py) bounds
the integral term's contribution but isn't sufficient alone - live testing
confirmed the PROPORTIONAL term alone can still reach the fault threshold,
since node 7 is commanded to a fixed near-limit position (~-0.85) regardless
of whether an object is already blocking further closure. Node 7's
pos_gain=50.0 is still the raw, never-tuned "5208" profile default from
initial commissioning - same category of gap as current_soft_max/hard_max
and vel_integrator_limit before this session's fixes.

RAM ONLY by default - does NOT save to flash. Meant to be run, then
live-tested (grip something rigid/soft, watch Iq_measured), and only saved
to flash afterward with --save once confirmed good. Targets node 7 only.
"""
import sys
import os
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import can
from src.can_utils import discover_node_ids
from src.configure import load_endpoints, read_config, set_odrive_parameter, save_config

NODE_ID = 7
DEFAULT_POS_GAIN = 25.0  # 50% of the current 50.0 default - see module docstring


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gain", type=float, default=DEFAULT_POS_GAIN,
                         help=f"pos_gain (default {DEFAULT_POS_GAIN}, current live value is 50.0)")
    parser.add_argument("--save", action="store_true",
                         help="Also save to flash (only after live-testing this RAM-only first)")
    args = parser.parse_args()

    bus = can.interface.Bus("can0", interface="socketcan")
    endpoints = load_endpoints()

    node_ids = discover_node_ids(bus)
    if NODE_ID not in node_ids:
        print(f"[ERROR] Node {NODE_ID} not found on the bus (discovered: {node_ids}).")
        bus.shutdown()
        sys.exit(1)

    ep = endpoints['endpoints']['axis0.controller.config.pos_gain']
    current_val = read_config(bus, NODE_ID, ep['id'], ep['type'])
    print(f"[INFO] Current pos_gain: {current_val}")
    print(f"[INFO] Setting pos_gain to {args.gain} ({'RAM + flash' if args.save else 'RAM only'})...")

    ok = set_odrive_parameter(bus, NODE_ID, "axis0.controller.config.pos_gain", args.gain, endpoints)
    if not ok:
        print("[ERROR] Write failed - see above.")
        bus.shutdown()
        sys.exit(1)

    if args.save:
        save_endpoint_id = endpoints['endpoints']['save_configuration']['id']
        save_config(bus, NODE_ID, save_endpoint_id)
        print("[INFO] Saved to flash.")
    else:
        print("[INFO] RAM only - this reverts on next reboot/node reset (including a fault-forced "
              "reset). Live-test now, then re-run with --save once confirmed good.")

    bus.shutdown()


if __name__ == "__main__":
    main()
