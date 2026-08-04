#!/usr/bin/env python3
"""
Dry run only - never arms a node, never sends a single CAN command.

Tracks the min/max reached for every controlled value during the
session and prints ONE summary table when you press Ctrl+C, showing
whether each one actually reached its configured safety limit.
No scrolling wall of text to read while it's running.
"""
import time
import pygame
import can

import gamecontroller as gc

def main():
    bus = can.interface.Bus("can0", interface="socketcan")
    node_ids = list(gc.discover_node_ids(bus))
    endpoints = gc.load_endpoints()

    if not node_ids:
        print("[ERROR] No ODrives found on CAN bus.")
        return
    print(f"Discovered nodes: {node_ids}")

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No joystick found.")
        return
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    joint_positions = {0: 0.0, 3: 0.0, 4: 0.0, 7: 0.0}
    shoulder_val = 0.0
    bend_pos = 0.0
    rotate_pos = 0.0
    dt = 1.0 / gc.UPDATE_RATE

    # name -> [min_seen, max_seen, true_min, true_max]
    tracked = {
        "J0 (base, node0)":        [0.0, 0.0, gc.JOINT0_MIN, gc.JOINT0_MAX],
        "J1 (shoulder)":           [0.0, 0.0, gc.JOINT1_MIN, gc.JOINT1_MAX],
        "J2 (node3)":              [0.0, 0.0, gc.JOINT2_MIN, gc.JOINT2_MAX],
        "J3 (node4/elbow)":        [0.0, 0.0, gc.JOINT3_MIN, gc.JOINT3_MAX],
        "wrist bend":              [0.0, 0.0, gc.BEND_MIN, gc.BEND_MAX],
        "wrist rotate":            [0.0, 0.0, gc.ROTATE_MIN, gc.ROTATE_MAX],
        "motorA (node5, raw)":     [0.0, 0.0, gc.MOTOR5_MIN, gc.MOTOR5_MAX],
        "motorB (node6, raw)":     [0.0, 0.0, gc.MOTOR6_MIN, gc.MOTOR6_MAX],
        "gripper":                 [0.0, 0.0, gc.TRIGGER_MIN, gc.TRIGGER_MAX],
    }

    def track(name, val):
        t = tracked[name]
        if val < t[0]: t[0] = val
        if val > t[1]: t[1] = val

    print("\nDRY RUN - nothing is armed, nothing is sent to the robot.")
    print("Hold L1 to 'move'. Push every axis/trigger to its extremes (both directions,")
    print("normal mode and R1-held wrist mode). Ctrl+C when done for a summary.\n")

    try:
        while True:
            pygame.event.pump()
            lb = joystick.get_button(gc.DEAD_MAN_BUTTON_INDEX)
            rb = joystick.get_button(gc.MODE_TOGGLE_BUTTON_INDEX)

            raw_bend   = gc.apply_dead_zone(joystick.get_axis(gc.AXIS_LEFT_Y))
            raw_rotate = gc.apply_dead_zone(joystick.get_axis(gc.AXIS_LEFT_X))
            rx = gc.apply_dead_zone(joystick.get_axis(gc.AXIS_RIGHT_X))
            ry = gc.apply_dead_zone(joystick.get_axis(gc.AXIS_RIGHT_Y))
            raw_lt = gc.apply_dead_zone(joystick.get_axis(gc.AXIS_LEFT_TRIGGER))
            raw_rt = gc.apply_dead_zone(joystick.get_axis(gc.AXIS_RIGHT_TRIGGER))

            if lb:
                wrist_mode = (rb == 1)

                if 3 in node_ids:
                    nv = joint_positions[3] + gc.taper_increment(joint_positions[3], rx * gc.VELOCITY_SCALING * dt, gc.JOINT2_MIN, gc.JOINT2_MAX, gc.DECEL_ZONE)
                    joint_positions[3] = max(gc.JOINT2_MIN, min(gc.JOINT2_MAX, nv))

                if 4 in node_ids:
                    nv = joint_positions[4] + gc.taper_increment(joint_positions[4], -ry * gc.VELOCITY_SCALING * dt, gc.JOINT3_MIN, gc.JOINT3_MAX, gc.DECEL_ZONE)
                    joint_positions[4] = max(gc.JOINT3_MIN, min(gc.JOINT3_MAX, nv))

                if not wrist_mode:
                    if 0 in node_ids:
                        nv = joint_positions[0] + gc.taper_increment(joint_positions[0], raw_rotate * gc.VELOCITY_SCALING * dt, gc.JOINT0_MIN, gc.JOINT0_MAX, gc.DECEL_ZONE)
                        joint_positions[0] = max(gc.JOINT0_MIN, min(gc.JOINT0_MAX, nv))

                    if (1 in node_ids) and (2 in node_ids):
                        nv = shoulder_val + gc.taper_increment(shoulder_val, raw_bend * gc.VELOCITY_SCALING * dt, gc.JOINT1_MIN, gc.JOINT1_MAX, gc.DECEL_ZONE)
                        shoulder_val = max(gc.JOINT1_MIN, min(gc.JOINT1_MAX, nv))
                else:
                    if (5 in node_ids) and (6 in node_ids):
                        nb = bend_pos + gc.taper_increment(bend_pos, raw_bend * gc.VELOCITY_SCALING * gc.FOREARM_VELOCITY_SCALING * dt, gc.BEND_MIN, gc.BEND_MAX, gc.DECEL_ZONE_WRIST)
                        nr = rotate_pos + gc.taper_increment(rotate_pos, raw_rotate * gc.VELOCITY_SCALING * gc.FOREARM_VELOCITY_SCALING * dt, gc.ROTATE_MIN, gc.ROTATE_MAX, gc.DECEL_ZONE_WRIST)
                        bend_pos = max(gc.BEND_MIN, min(gc.BEND_MAX, nb))
                        rotate_pos = max(gc.ROTATE_MIN, min(gc.ROTATE_MAX, nr))

                if 7 in node_ids:
                    np_ = joint_positions[7]
                    if raw_lt > 0:
                        np_ += gc.taper_increment(np_, -raw_lt * gc.VELOCITY_SCALING * gc.GRIPPER_SCALING * dt, gc.TRIGGER_MIN, gc.TRIGGER_MAX, gc.DECEL_ZONE_GRIPPER)
                    if raw_rt > 0:
                        np_ += gc.taper_increment(np_, raw_rt * gc.VELOCITY_SCALING * gc.GRIPPER_SCALING * dt, gc.TRIGGER_MIN, gc.TRIGGER_MAX, gc.DECEL_ZONE_GRIPPER)
                    joint_positions[7] = max(gc.TRIGGER_MIN, min(gc.TRIGGER_MAX, np_))

            motorA_c = max(gc.MOTOR5_MIN, min(gc.MOTOR5_MAX, rotate_pos + bend_pos))
            motorB_c = max(gc.MOTOR6_MIN, min(gc.MOTOR6_MAX, rotate_pos - bend_pos))

            track("J0 (base, node0)", joint_positions[0])
            track("J1 (shoulder)", shoulder_val)
            track("J2 (node3)", joint_positions[3])
            track("J3 (node4/elbow)", joint_positions[4])
            track("wrist bend", bend_pos)
            track("wrist rotate", rotate_pos)
            track("motorA (node5, raw)", motorA_c)
            track("motorB (node6, raw)", motorB_c)
            track("gripper", joint_positions[7])

            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()
        pygame.quit()

    print("\nStopped. No CAN commands were ever sent to the ODrives.\n")
    print(f"{'Axis':<22} {'Min seen':>10} {'True min':>10} {'Min OK':>7}   {'Max seen':>10} {'True max':>10} {'Max OK':>7}")
    EPS = 0.02
    for name, (mn, mx, true_min, true_max) in tracked.items():
        min_hit = "HIT" if mn <= true_min + EPS else "no"
        max_hit = "HIT" if mx >= true_max - EPS else "no"
        print(f"{name:<22} {mn:>10.3f} {true_min:>10.3f} {min_hit:>7}   {mx:>10.3f} {true_max:>10.3f} {max_hit:>7}")

if __name__ == "__main__":
    main()
