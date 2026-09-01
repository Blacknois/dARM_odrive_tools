#!/usr/bin/env python3
"""
apply_gripper_integrator_limit.py

Fixes the real root cause found 2026-08-28 for node 7 (gripper) faulting on
ANY rigid grip: axis0.controller.config.vel_integrator_limit was unset
(=inf) - no anti-windup cap, so the position controller's integral term
accumulates commanded current without bound as long as any residual
position error persists (which it always will, gripping something that
won't yield further), until it hits current_hard_max and faults. This is
a different mechanism from the transient stall-spike investigated earlier
the same day - see darm_feature_backlog.md's gripper section for both.

Sets a finite integrator limit (Nm, same units as vel_integrator_torque/
torque_constant) instead of switching control architecture at all - the
existing position control keeps working exactly as it does today, just
capped instead of unbounded.

RAM ONLY by default - does NOT save to flash. Meant to be run, then
live-tested (grip something rigid, watch vel_integrator_torque/Iq_measured
plateau instead of climbing), and only saved to flash afterward with
--save once that's confirmed. Targets node 7 only.
"""
import sys
import os
import argparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import can
from src.can_utils import discover_node_ids
from src.configure import load_endpoints, read_config, set_odrive_parameter, save_config

NODE_ID = 7
DEFAULT_LIMIT_NM = 0.12  # conservative starting cap - see module docstring for the reasoning


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--limit", type=float, default=DEFAULT_LIMIT_NM,
                         help=f"vel_integrator_limit in Nm (default {DEFAULT_LIMIT_NM})")
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

    current_val = read_config(
        bus, NODE_ID,
        endpoints['endpoints']['axis0.controller.config.vel_integrator_limit']['id'],
        endpoints['endpoints']['axis0.controller.config.vel_integrator_limit']['type'],
    )
    print(f"[INFO] Current vel_integrator_limit: {current_val}")
    print(f"[INFO] Setting vel_integrator_limit to {args.limit} Nm ({'RAM + flash' if args.save else 'RAM only'})...")

    ok = set_odrive_parameter(
        bus, NODE_ID, "axis0.controller.config.vel_integrator_limit", args.limit, endpoints
    )
    if not ok:
        print("[ERROR] Write failed - see above.")
        bus.shutdown()
        sys.exit(1)

    if args.save:
        save_endpoint_id = endpoints['endpoints']['save_configuration']['id']
        save_config(bus, NODE_ID, save_endpoint_id)
        print("[INFO] Saved to flash.")
    else:
        print("[INFO] RAM only - this reverts on next reboot/node reset. "
              "Live-test now (grip something rigid, watch vel_integrator_torque and Iq_measured), "
              "then re-run with --save once confirmed good.")

    bus.shutdown()


if __name__ == "__main__":
    main()
