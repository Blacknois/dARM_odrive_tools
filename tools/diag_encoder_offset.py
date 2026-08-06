#!/usr/bin/env python3
"""
Requests ENCODER_OFFSET_CALIBRATION (state 7) alone on node 2 and watches
current_state, offset, and offset_valid every 50ms, to see exactly what
happens during just this one sub-step (the full calibration sequence's
1-second polling may have been too coarse to catch it).
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # so this can find src/ when run from tools/
import time
import can
from src.configure import load_endpoints, read_config, save_config
from src.control import move_odrive_to_position
from src.can_utils import send_can_message

NODE_ID = 2
ENCODER_OFFSET_CALIBRATION = 7
IDLE = 1

STATE_NAMES = {
    0: "UNDEFINED", 1: "IDLE", 2: "STARTUP_SEQUENCE", 3: "FULL_CALIBRATION",
    4: "MOTOR_CALIBRATION", 6: "ENCODER_INDEX_SEARCH", 7: "ENCODER_OFFSET_CALIBRATION",
    8: "CLOSED_LOOP_CONTROL",
}

def main():
    bus = can.interface.Bus("can0", interface="socketcan")
    endpoints = load_endpoints()['endpoints']

    state_ep  = endpoints['axis0.current_state']
    offset_ep = endpoints['axis0.commutation_mapper.config.offset']
    valid_ep  = endpoints['axis0.commutation_mapper.config.offset_valid']
    pos_ep    = endpoints['axis0.pos_estimate']
    save_ep   = endpoints['save_configuration']

    pos = read_config(bus, NODE_ID, pos_ep['id'], pos_ep['type'])
    print(f"Node {NODE_ID} current position: {pos}")
    move_odrive_to_position(bus, NODE_ID, pos if pos is not None else 0.0)
    time.sleep(0.1)

    input(f"About to run ENCODER_OFFSET_CALIBRATION on node {NODE_ID}. Press Enter when clear to proceed...")

    print(f"Requesting ENCODER_OFFSET_CALIBRATION for node {NODE_ID}...")
    send_can_message(bus, NODE_ID, 0x07, '<I', ENCODER_OFFSET_CALIBRATION)

    start = time.time()
    last_state = None
    while time.time() - start < 6.0:
        state = read_config(bus, NODE_ID, state_ep['id'], state_ep['type'])
        if state != last_state:
            print(f"t={time.time()-start:5.3f}s  current_state={state} ({STATE_NAMES.get(state, 'UNKNOWN')})")
            last_state = state
        if state == IDLE and time.time() - start > 0.5:
            break
        time.sleep(0.02)

    offset = read_config(bus, NODE_ID, offset_ep['id'], offset_ep['type'])
    valid  = read_config(bus, NODE_ID, valid_ep['id'], valid_ep['type'])
    print(f"\nImmediately after: offset={offset}  offset_valid={valid}")

    if valid:
        send_can_message(bus, NODE_ID, 0x04, '<BHB', 1, save_ep['id'], 0)
        print("Saved configuration.")

    bus.shutdown()

if __name__ == "__main__":
    main()
