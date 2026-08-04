#!/usr/bin/env python3
"""
Dry run only. Reads the real controller and computes exactly what
gamecontroller.py's logic would send to each ODrive node (same axis
mapping, dead zone, tapering, and clamps) - but NEVER arms a node and
NEVER sends a single CAN command. Pure read + compute + print.
Values shown are accumulators starting at 0 from when this script
launched - not necessarily the robot's true absolute position.
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
        print("[ERROR] No ODrives found on CAN bus (CAN/logic power must still be on).")
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
    last_print = 0.0

    print("\nDRY RUN - nothing is armed, nothing is sent to the robot.")
    print("Hold LB to 'move' (same dead-man gate as the real script). Ctrl+C to stop.\n")

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

            clamped = []

            if lb:
                wrist_mode = (rb == 1)

                if 3 in node_ids:
                    nv = joint_positions[3] + gc.taper_increment(joint_positions[3], rx * gc.VELOCITY_SCALING * dt, gc.JOINT2_MIN, gc.JOINT2_MAX, gc.DECEL_ZONE)
                    if nv <= gc.JOINT2_MIN or nv >= gc.JOINT2_MAX: clamped.append("J2")
                    joint_positions[3] = max(gc.JOINT2_MIN, min(gc.JOINT2_MAX, nv))

                if 4 in node_ids:
                    nv = joint_positions[4] + gc.taper_increment(joint_positions[4], -ry * gc.VELOCITY_SCALING * dt, gc.JOINT3_MIN, gc.JOINT3_MAX, gc.DECEL_ZONE)
                    if nv <= gc.JOINT3_MIN or nv >= gc.JOINT3_MAX: clamped.append("J3")
                    joint_positions[4] = max(gc.JOINT3_MIN, min(gc.JOINT3_MAX, nv))

                if not wrist_mode:
                    if 0 in node_ids:
                        nv = joint_positions[0] + gc.taper_increment(joint_positions[0], raw_rotate * gc.VELOCITY_SCALING * dt, gc.JOINT0_MIN, gc.JOINT0_MAX, gc.DECEL_ZONE)
                        if nv <= gc.JOINT0_MIN or nv >= gc.JOINT0_MAX: clamped.append("J0")
                        joint_positions[0] = max(gc.JOINT0_MIN, min(gc.JOINT0_MAX, nv))

                    if (1 in node_ids) and (2 in node_ids):
                        nv = shoulder_val + gc.taper_increment(shoulder_val, raw_bend * gc.VELOCITY_SCALING * dt, gc.JOINT1_MIN, gc.JOINT1_MAX, gc.DECEL_ZONE)
                        if nv <= gc.JOINT1_MIN or nv >= gc.JOINT1_MAX: clamped.append("J1")
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
                    if np_ <= gc.TRIGGER_MIN or np_ >= gc.TRIGGER_MAX: clamped.append("Gripper")
                    joint_positions[7] = max(gc.TRIGGER_MIN, min(gc.TRIGGER_MAX, np_))

            motorA = rotate_pos + bend_pos
            motorB = rotate_pos - bend_pos
            motorA_c = max(gc.MOTOR5_MIN, min(gc.MOTOR5_MAX, motorA))
            motorB_c = max(gc.MOTOR6_MIN, min(gc.MOTOR6_MAX, motorB))
            if motorA != motorA_c or motorB != motorB_c: clamped.append("Motor5/6-raw")

            now = time.time()
            if now - last_print > 0.1:
                last_print = now
                flags = f"  CLAMPED: {','.join(clamped)}" if clamped else ""
                line = (f"J0={joint_positions[0]:+.3f} J1(shoulder)={shoulder_val:+.3f} "
                        f"J2={joint_positions[3]:+.3f} J3={joint_positions[4]:+.3f} "
                        f"bend={bend_pos:+.3f} rotate={rotate_pos:+.3f} "
                        f"motorA={motorA_c:+.3f} motorB={motorB_c:+.3f} "
                        f"gripper={joint_positions[7]:+.3f}{flags}")
                print("\r" + line + " "*10, end="", flush=True)

            time.sleep(dt)
    except KeyboardInterrupt:
        print("\n\nStopped. No CAN commands were ever sent to the ODrives.")
    finally:
        bus.shutdown()
        pygame.quit()

if __name__ == "__main__":
    main()
