#!/usr/bin/env python3
"""
Watches node 2's axis0.current_state in real time right after requesting
CLOSED_LOOP_CONTROL, to see whether the request is flatly rejected (state
never leaves IDLE) or briefly accepted and then silently reverted.

Syncs input_pos to the actual current position first (same as console.py)
so there is no risk of a startup jump if it does arm.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # so this can find src/ when run from tools/
import time
import can
from src.configure import load_endpoints, read_config
from src.control import move_odrive_to_position, set_closed_loop_control
from src.can_utils import send_can_message

NODE_ID = 2

STATE_NAMES = {
    0: "UNDEFINED", 1: "IDLE", 2: "STARTUP_SEQUENCE", 3: "FULL_CALIBRATION",
    4: "MOTOR_CALIBRATION", 6: "ENCODER_INDEX_SEARCH", 7: "ENCODER_OFFSET_CALIBRATION",
    8: "CLOSED_LOOP_CONTROL", 9: "LOCKIN_SPIN", 10: "ENCODER_DIR_FIND",
    11: "HOMING", 12: "ENCODER_HALL_POLARITY_CALIBRATION", 13: "ENCODER_HALL_PHASE_CALIBRATION",
}

def main():
    bus = can.interface.Bus("can0", interface="socketcan")
    endpoints = load_endpoints()

    state_ep   = endpoints['endpoints']['axis0.current_state']
    armed_ep   = endpoints['endpoints']['axis0.is_armed']
    disarm_ep  = endpoints['endpoints']['axis0.disarm_reason']
    errors_ep  = endpoints['endpoints']['axis0.active_errors']
    pos_ep     = endpoints['endpoints']['axis0.pos_estimate']

    pos = read_config(bus, NODE_ID, pos_ep['id'], pos_ep['type'])
    print(f"Node {NODE_ID} current position: {pos}")
    move_odrive_to_position(bus, NODE_ID, pos if pos is not None else 0.0)
    time.sleep(0.1)

    print(f"Requesting CLOSED_LOOP_CONTROL for node {NODE_ID}, sampling current_state every 20ms for 2s...")
    set_closed_loop_control(bus, NODE_ID)

    start = time.time()
    while time.time() - start < 2.0:
        state = read_config(bus, NODE_ID, state_ep['id'], state_ep['type'])
        armed = read_config(bus, NODE_ID, armed_ep['id'], armed_ep['type'])
        name  = STATE_NAMES.get(state, f"UNKNOWN({state})")
        print(f"t={time.time()-start:5.3f}s  current_state={state} ({name})  is_armed={armed}")
        time.sleep(0.02)

    disarm_reason = read_config(bus, NODE_ID, disarm_ep['id'], disarm_ep['type'])
    active_errors = read_config(bus, NODE_ID, errors_ep['id'], errors_ep['type'])
    print(f"\nFinal: disarm_reason={disarm_reason}, active_errors={active_errors}")

    bus.shutdown()

if __name__ == "__main__":
    main()
