#!/usr/bin/env python3
"""
Standalone diagnostic for node 7 (gripper) only - never touches the
rest of the arm. Aims to tell apart an ODrive-side fault (comms,
active_errors, refusal to arm) from something downstream of it
(motor connector, slipped coupling, mechanical bind) or a bad encoder.

Drop this in the same odrive_tools folder as gamecontroller.py (it
reuses the same src/ modules) and run it there.

Usage:
    python3 check_gripper.py          read-only: discovery, is_armed,
                                       active_errors, disarm_reason, and
                                       an 8s live pos_estimate trace
                                       while you move the gripper by
                                       hand (node stays IDLE throughout,
                                       cannot move anything itself)

    python3 check_gripper.py --arm    additionally arms node 7 and
                                       sends ONE small bounded nudge to
                                       test the motor output stage.
                                       Only run this with the gripper
                                       clear of anything it could pinch
                                       and a hand on the power switch.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # so this can find src/ when run from tools/
import sys
import time
import can

from src.configure import load_endpoints, read_config
from src.can_utils import discover_node_ids
from src.control import move_odrive_to_position, set_closed_loop_control, set_idle_mode

NODE = 7
GRIPPER_STARTUP_OPEN = 0.4  # matches gamecontroller.py


def read(bus, endpoints, key):
    ep = endpoints['endpoints'][key]
    return read_config(bus, NODE, ep['id'], ep['type'])


def main():
    do_arm = '--arm' in sys.argv

    bus = can.interface.Bus("can0", bustype="socketcan")
    discovered = list(discover_node_ids(bus))
    endpoints = load_endpoints()

    print(f"Discovered node IDs on CAN bus: {sorted(discovered)}")
    if NODE not in discovered:
        print(f"[RESULT] Node {NODE} did NOT respond to discovery at all. That points at the "
              f"ODrive itself (or its CAN wiring/power/termination), not the gripper mechanism "
              f"downstream of it - a board that's fine but mechanically jammed would still show "
              f"up here.")
        bus.shutdown()
        return

    print(f"[OK] Node {NODE} responds on the CAN bus.\n")

    is_armed      = read(bus, endpoints, 'axis0.is_armed')
    active_errors = read(bus, endpoints, 'axis0.active_errors')
    disarm_reason = read(bus, endpoints, 'axis0.disarm_reason')
    pos           = read(bus, endpoints, 'axis0.pos_estimate')

    print(f"    is_armed:      {is_armed}")
    print(f"    active_errors: {active_errors}  (nonzero = ODrive reporting a fault on its own)")
    print(f"    disarm_reason: {disarm_reason}")
    print(f"    pos_estimate:  {pos}")

    if active_errors:
        print(f"\n[RESULT] active_errors={active_errors} with nothing commanded yet - this is "
              f"an ODrive-side fault (or a motor/encoder wiring fault it's detecting), not a "
              f"downstream mechanical issue. Look up {active_errors} against the ODrive error "
              f"code table before doing anything else.")
        bus.shutdown()
        return

    print("\n[STEP] Encoder live-tracking check. Node stays IDLE/unarmed here - this cannot move "
          "anything. Gently move the gripper open/closed BY HAND now.")
    print("       Watching pos_estimate for 8 seconds - it should change smoothly as you move it.")
    start = time.time()
    last = None
    saw_change = False
    while time.time() - start < 8.0:
        pos = read(bus, endpoints, 'axis0.pos_estimate')
        if pos is not None and last is not None and abs(pos - last) > 0.01:
            saw_change = True
        if pos is not None:
            print(f"    pos_estimate = {pos:.4f}")
            last = pos
        time.sleep(0.3)

    if saw_change:
        print("\n[RESULT] pos_estimate tracked your hand movement - the ODrive and its encoder "
              "feedback are working fine. If the gripper still doesn't respond to the triggers "
              "in gamecontroller.py, the fault is most likely downstream: motor windings/"
              "connector, a slipped coupling, or a mechanical bind - not the ODrive board.")
    else:
        print("\n[RESULT] pos_estimate did NOT change while you moved it by hand. Either the "
              "encoder isn't being read correctly (wiring/connector, or the encoder itself), or "
              "nothing actually moved. This points at the ODrive/encoder side rather than a "
              "purely mechanical issue - a healthy encoder shows this regardless of the motor.")

    if not do_arm:
        print("\nRun with --arm to additionally test closed-loop control with a small, bounded "
              "move (gripper clear of anything, hand on the power switch).")
        bus.shutdown()
        return

    print("\n[STEP] Attempting to arm node 7 (closed-loop control)...")
    armed = False
    for attempt in range(5):
        set_closed_loop_control(bus, NODE)
        t0 = time.time()
        while time.time() - t0 < 0.3:
            if read(bus, endpoints, 'axis0.is_armed'):
                armed = True
                break
            time.sleep(0.05)
        if armed:
            break

    if not armed:
        active_errors = read(bus, endpoints, 'axis0.active_errors')
        disarm_reason = read(bus, endpoints, 'axis0.disarm_reason')
        print(f"[RESULT] Node 7 refused to arm (active_errors={active_errors}, "
              f"disarm_reason={disarm_reason}) - ODrive-side fault, not mechanical.")
        bus.shutdown()
        return

    print("[OK] Node 7 armed. Sending one small bounded nudge...")
    current_pos = read(bus, endpoints, 'axis0.pos_estimate')
    if current_pos is None:
        current_pos = GRIPPER_STARTUP_OPEN
    target = current_pos + 0.1 if current_pos < 0.6 else current_pos - 0.1
    move_odrive_to_position(bus, NODE, target)
    time.sleep(2)
    new_pos = read(bus, endpoints, 'axis0.pos_estimate')
    active_errors = read(bus, endpoints, 'axis0.active_errors')
    print(f"    commanded target: {target:.4f}")
    print(f"    pos_estimate now: {new_pos}")
    print(f"    active_errors:    {active_errors}")

    if active_errors:
        print(f"\n[RESULT] Motion triggered active_errors={active_errors} - ODrive-side fault "
              f"(likely motor driver/current-related).")
    elif new_pos is not None and abs(new_pos - target) <= 0.05:
        print("\n[RESULT] Node 7 moved to and held the commanded target with no errors - the "
              "ODrive, its motor output stage, and the encoder are all functioning. If the "
              "gripper still misbehaves in gamecontroller.py, look at the trigger mapping/"
              "scaling logic there rather than the hardware.")
    else:
        print("\n[RESULT] Commanded a move but pos_estimate never confirmed reaching it, with no "
              "active_errors reported - consistent with a mechanical bind or something "
              "physically stopping the gripper even though the ODrive thinks the command "
              "succeeded.")

    print("\nDisarming node 7...")
    set_idle_mode(bus, NODE)
    time.sleep(0.3)
    bus.shutdown()


if __name__ == "__main__":
    main()
