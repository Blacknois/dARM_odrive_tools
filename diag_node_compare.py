#!/usr/bin/env python3
"""
Read-only diagnostic: compares each node's trap_traj vel/accel/decel limits,
current/torque limits, and error flags. No writes, no arming, no motion.
"""
import can
from src.configure import load_endpoints, read_config

NODE_IDS = [0, 1, 2, 3, 4, 5, 6, 7]

FIELDS = [
    ("axis0.trap_traj.config.vel_limit",    "trap_vel"),
    ("axis0.trap_traj.config.accel_limit",  "trap_accel"),
    ("axis0.trap_traj.config.decel_limit",  "trap_decel"),
    ("axis0.motor.config.current_lim",      "current_lim"),
    ("axis0.controller.config.vel_limit",   "ctrl_vel_lim"),
    ("axis0.error",                         "axis_error"),
    ("axis0.motor.error",                   "motor_error"),
    ("axis0.encoder.error",                 "encoder_error"),
    ("axis0.controller.error",              "ctrl_error"),
]

def main():
    bus = can.interface.Bus("can0", interface="socketcan")
    endpoints = load_endpoints()['endpoints']

    header = f"{'Node':<6}" + "".join(f"{label:<14}" for _, label in FIELDS)
    print(header)
    print("-" * len(header))

    for nid in NODE_IDS:
        row = f"{nid:<6}"
        for path, label in FIELDS:
            ep = endpoints.get(path)
            if ep is None:
                row += f"{'N/A':<14}"
                continue
            val = read_config(bus, nid, ep['id'], ep['type'])
            row += f"{str(val):<14}"
        print(row)

    print("\nDone. No commands were sent to any node.")
    bus.shutdown()

if __name__ == "__main__":
    main()
