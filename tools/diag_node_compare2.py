#!/usr/bin/env python3
"""
Read-only diagnostic v2: uses real endpoint names for state/errors/current
limits. No writes, no arming, no motion.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # so this can find src/ when run from tools/
import can
from src.configure import load_endpoints, read_config

NODE_IDS = [0, 1, 2, 3, 4, 5, 6, 7]

FIELDS = [
    ("axis0.current_state",                 "state"),
    ("axis0.active_errors",                 "active_errors"),
    ("axis0.motor.effective_current_lim",   "eff_cur_lim"),
    ("axis0.config.motor.current_soft_max", "cur_soft_max"),
    ("axis0.config.motor.current_hard_max", "cur_hard_max"),
    ("axis0.trap_traj.config.vel_limit",    "trap_vel"),
    ("axis0.trap_traj.config.accel_limit",  "trap_accel"),
    ("axis0.trap_traj.config.decel_limit",  "trap_decel"),
    ("axis0.controller.config.vel_limit",   "ctrl_vel_lim"),
]

def main():
    bus = can.interface.Bus("can0", interface="socketcan")
    endpoints = load_endpoints()['endpoints']

    header = f"{'Node':<6}" + "".join(f"{label:<16}" for _, label in FIELDS)
    print(header)
    print("-" * len(header))

    for nid in NODE_IDS:
        row = f"{nid:<6}"
        for path, label in FIELDS:
            ep = endpoints.get(path)
            if ep is None:
                row += f"{'N/A':<16}"
                continue
            val = read_config(bus, nid, ep['id'], ep['type'])
            row += f"{str(val):<16}"
        print(row)

    print("\nDone. No commands were sent to any node.")
    bus.shutdown()

if __name__ == "__main__":
    main()
