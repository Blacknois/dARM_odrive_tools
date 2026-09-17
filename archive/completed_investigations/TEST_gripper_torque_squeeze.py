#!/usr/bin/env python3
"""
TEST_gripper_torque_squeeze.py

Isolated, single-node (7 / gripper only) prototype of a "ratioed squeeze"
grip control: L2/R2 map proportionally to a COMMANDED CURRENT (via torque
control mode) instead of a commanded POSITION - see 2026-08-28 session
notes for the full reasoning. Does NOT touch gamecontroller.py, node 7's
flash config, or any other node. Refuses to start if gamecontroller.py is
already running, so two processes can never fight over node 7 at once.

WHY THIS EXISTS: node 7's normal position-control grip (driving to a fixed
closed position against a piece/obstruction) lets commanded current climb
unboundedly as position error refuses to close, and has now faulted twice
this way (disarm_reason=2048 on an old auto-open move, disarm_reason=4096
on 2026-08-28's deliberate stall test). In torque control mode, current
IS the direct command, so it can never exceed what's actually being asked
for - the motor holds at the commanded level instead of chasing an
unreachable position.

WHAT THIS SCRIPT DOES:
  - Switches ONLY node 7 into torque control mode (RAM only, never saved
    to flash - restore_position_mode() runs on every exit path, so
    gamecontroller.py's normal position-mode operation is never affected
    by having run this).
  - R2 = close (ratio 0-1 -> 0 to TEST_MAX_CURRENT_A of closing current)
    L2 = open  (ratio 0-1 -> 0 to TEST_MAX_CURRENT_A of opening current)
    NOTE: this is the OPPOSITE of gamecontroller.py's own position-mode
    convention (L2=close/R2=open there) - swapped 2026-08-28 at Carla's
    request for this test specifically. Don't assume the two scripts
    agree on trigger mapping.
  - LB is a dead-man switch, same as gamecontroller.py - release it and
    commanded current drops to zero immediately, regardless of trigger
    position.
  - Live position is still read every frame (the encoder keeps reporting
    in ANY control mode) and used as a software safety clamp: if closing
    and position has reached TRIGGER_MIN (or opening and reached
    TRIGGER_MAX), commanded current in that direction is forced to zero.
    Torque mode doesn't know about position limits on its own - this
    replaces that missing protection with the same live-clamp pattern
    gamecontroller.py already uses for every joint.
  - Prints live status every frame: dead-man state, L2/R2 ratio,
    commanded current, live position, live Iq_measured (polled at a
    reduced 1-in-5 frame rate to limit extra CAN traffic).

IMPORTANT - VERIFY BEFORE TRUSTING:
  - The closing-direction sign (negative torque = closing) is inferred
    from gamecontroller.py's existing position-mode convention (L2 =>
    joint_positions[7] moves negative => toward TRIGGER_MIN) but has NOT
    been live-verified for TORQUE mode specifically - phase/winding
    order could make the actual physical direction come out backwards.
    FIRST test: hold LB, press R2 very lightly (low ratio) and confirm
    the fingers actually move toward closed, not open, before trusting
    any real grip test.
  - TEST_MAX_CURRENT_A defaults conservative (0.6A), well under both
    current_soft_max (1.0A) and 2026-08-28's fault current (~1.5A) -
    deliberately not starting near the limits on a brand-new, never
    live-tested control path. Raise only after confirming direction and
    basic behavior are correct.
"""
import sys
import os
import time
import subprocess

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import can
import pygame

from src.can_utils import discover_node_ids
from src.control import set_closed_loop_control, set_idle_mode, move_odrive_with_torque
from src.configure import load_endpoints, read_config, set_odrive_parameter

NODE_ID = 7
DEAD_MAN_BUTTON_INDEX = 4   # LB - same index as gamecontroller.py
AXIS_LEFT_TRIGGER = 2       # L2 - close
AXIS_RIGHT_TRIGGER = 5      # R2 - open
TRIGGER_DEAD_ZONE = 0.05

# Same real measured/configured values gamecontroller.py uses for node 7.
TRIGGER_MIN, TRIGGER_MAX = -0.9037, 0.032
POSITION_SAFETY_MARGIN = 0.02  # stop commanding further closing/opening current this close to the limit
TORQUE_CONSTANT = 0.345        # Nm/A, axis0.config.motor.torque_constant - see 2026-08-28 session
                                # notes on the discrepancy with the datasheet's implied ~0.18-0.25
                                # Nm/A; using the configured/measured value since that's what the
                                # ODrive itself uses internally to convert input_torque -> current.

TEST_MAX_CURRENT_A = 0.6  # conservative starting ceiling - see module docstring


def apply_trigger_dead_zone(value):
    """Same normalization as gamecontroller.py's apply_trigger_dead_zone() -
    triggers rest at -1.0 (released), +1.0 (fully pressed)."""
    normalized = (value + 1.0) / 2.0
    if normalized < TRIGGER_DEAD_ZONE:
        return 0.0
    return (normalized - TRIGGER_DEAD_ZONE) / (1.0 - TRIGGER_DEAD_ZONE)


def restore_position_mode(bus, endpoints):
    """Always called on exit (see finally block in main()) - switches node 7
    back to position control (RAM only, matches how it started - never saved
    to flash) so gamecontroller.py's normal operation is completely
    unaffected by having run this test."""
    try:
        set_odrive_parameter(bus, NODE_ID, "axis0.controller.config.control_mode", 3, endpoints)
        set_odrive_parameter(bus, NODE_ID, "axis0.controller.config.enable_torque_mode_vel_limit", True, endpoints)
        print("[INFO] Node 7 restored to position control mode (RAM only).")
    except Exception as e:
        print(f"[WARNING] Failed to restore node 7 to position mode: {e} - "
              f"power-cycling before the next gamecontroller.py session is the safe fallback.")


def main():
    check = subprocess.run(["pgrep", "-f", "gamecontroller.py"], capture_output=True)
    if check.returncode == 0:
        print("[REFUSE] gamecontroller.py is already running - refusing to start, "
              "two processes commanding node 7 at once is unsafe. Stop it first.")
        sys.exit(1)

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("[ERROR] No joystick found.")
        sys.exit(1)
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    bus = can.interface.Bus("can0", interface="socketcan")
    endpoints = load_endpoints()

    node_ids = discover_node_ids(bus)
    if NODE_ID not in node_ids:
        print(f"[ERROR] Node {NODE_ID} not found on the bus (discovered: {node_ids}).")
        bus.shutdown()
        sys.exit(1)

    pos_ep = endpoints['endpoints']['axis0.pos_estimate']
    iq_ep = endpoints['endpoints']['axis0.motor.foc.Iq_measured']

    print("[INFO] Switching node 7 to torque control mode (RAM only, not saved to flash)...")
    set_odrive_parameter(bus, NODE_ID, "axis0.controller.config.control_mode", 1, endpoints)
    set_odrive_parameter(bus, NODE_ID, "axis0.controller.config.enable_torque_mode_vel_limit", False, endpoints)

    print("[INFO] Arming node 7 (closed loop control)...")
    set_closed_loop_control(bus, NODE_ID)
    time.sleep(0.3)

    clock = pygame.time.Clock()
    frame = 0
    iq_measured = 0.0

    print("\n" + "=" * 70)
    print("Hold LB (dead-man) + R2 to close, LB + L2 to open.")
    print(f"Max test current: {TEST_MAX_CURRENT_A}A. Ctrl+C to stop.")
    print("VERIFY DIRECTION FIRST with a light R2 press before trusting this.")
    print("=" * 70 + "\n")

    try:
        while True:
            clock.tick(30)
            pygame.event.pump()

            dead_man = joystick.get_button(DEAD_MAN_BUTTON_INDEX) == 1
            raw_lt = apply_trigger_dead_zone(joystick.get_axis(AXIS_LEFT_TRIGGER))
            raw_rt = apply_trigger_dead_zone(joystick.get_axis(AXIS_RIGHT_TRIGGER))

            position = read_config(bus, NODE_ID, pos_ep['id'], pos_ep['type'])
            if position is None:
                position = 0.0  # unavailable this frame - falls through to zero torque below

            closing_blocked = position <= (TRIGGER_MIN + POSITION_SAFETY_MARGIN)
            opening_blocked = position >= (TRIGGER_MAX - POSITION_SAFETY_MARGIN)

            close_ratio = raw_rt if (dead_man and not closing_blocked) else 0.0
            open_ratio = raw_lt if (dead_man and not opening_blocked) else 0.0

            target_current = (open_ratio - close_ratio) * TEST_MAX_CURRENT_A  # open=+, close=-
            target_torque = target_current * TORQUE_CONSTANT
            move_odrive_with_torque(bus, NODE_ID, target_torque)

            frame += 1
            if frame % 5 == 0:
                iq_read = read_config(bus, NODE_ID, iq_ep['id'], iq_ep['type'])
                if iq_read is not None:
                    iq_measured = iq_read

            blocked_str = ("CLOSE-LIMIT " if closing_blocked else "") + ("OPEN-LIMIT " if opening_blocked else "")
            print(f"\rLB:{'HELD' if dead_man else '-   '}  "
                  f"L2:{raw_lt:.2f} R2:{raw_rt:.2f}  "
                  f"target_I:{target_current:+.2f}A  pos:{position:+.4f}  "
                  f"Iq_meas:{iq_measured:+.2f}A  {blocked_str}   ", end="", flush=True)

    except KeyboardInterrupt:
        print("\n[INFO] Stopping...")
    finally:
        move_odrive_with_torque(bus, NODE_ID, 0.0)
        time.sleep(0.1)
        set_idle_mode(bus, NODE_ID)
        restore_position_mode(bus, endpoints)
        bus.shutdown()
        pygame.quit()


if __name__ == "__main__":
    main()
