#!/usr/bin/env python3
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # so this can find src/ when run from tools/

import argparse
import time
import can
from src.configure import load_endpoints, read_config, save_config
from src.control import set_idle_mode
from src.can_utils import discover_node_ids, send_can_message

# ODrive states mapped to descriptions
ODRIVE_STATES = {
    1: "IDLE",
    2: "STARTUP_SEQUENCE",
    3: "FULL_CALIBRATION",
    4: "MOTOR_CALIBRATION",
    6: "ENCODER_INDEX_SEARCH",
    7: "ENCODER_OFFSET_CALIBRATION",
    8: "CLOSED_LOOP_CONTROL",
}

def calibrate_motor(bus, node_id, endpoints):
    """
    Runs full calibration for a single motor and waits for it to complete.
    """
    try:
        print(f"Starting calibration for node {node_id}...")
        send_can_message(bus, node_id, 0x07, '<I', 3)  # Set_Axis_State: FULL_CALIBRATION

        state_endpoint_id = endpoints["endpoints"]["axis0.current_state"]["id"]
        state_endpoint_type = endpoints["endpoints"]["axis0.current_state"]["type"]

        start_time = time.time()
        timeout = 30

        while time.time() - start_time < timeout:
            state = read_config(bus, node_id, state_endpoint_id, state_endpoint_type)
            state_description = ODRIVE_STATES.get(state, "UNKNOWN")

            if state == 1:
                print(f"Node {node_id} calibration completed successfully.")
                return True
            else:
                print(f"[INFO] Node {node_id} is in state {state_description} (State Code: {state}). Waiting...")

            time.sleep(1)

        print(f"[ERROR] Node {node_id} did not complete calibration within {timeout} seconds.")
        return False
    except Exception as e:
        print(f"[ERROR] Calibration error for node {node_id}: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description="Calibrate a single ODrive node on the CAN bus.")
    parser.add_argument("-id", type=int, required=True, help="Node id to calibrate (e.g. -id 7)")
    args = parser.parse_args()

    bus = None
    try:
        bus = can.interface.Bus("can0", interface="socketcan")
        print("Discovering ODrives on the CAN network...")
        node_ids = discover_node_ids(bus)
        print(f"Discovered {len(node_ids)} ODrive(s): {node_ids}\n")

        if args.id not in node_ids:
            print(f"[ERROR] Node {args.id} not found on the bus. Available nodes: {node_ids}")
            return

        endpoints = load_endpoints()

        print(f"Preparing to calibrate ONLY node {args.id}.")
        input("Ensure it is safe to proceed with calibration. Press Enter to continue...")

        set_idle_mode(bus, args.id)
        if calibrate_motor(bus, args.id, endpoints):
            save_endpoint_id = endpoints["endpoints"]["save_configuration"]["id"]
            save_config(bus, args.id, save_endpoint_id)
            print(f"Node {args.id} successfully calibrated and saved.")
        else:
            print(f"[ERROR] Calibration failed for node {args.id}.")

    except Exception as e:
        print(f"[ERROR] Calibration process encountered an error: {e}")
    finally:
        if bus is not None:
            bus.shutdown()

if __name__ == "__main__":
    main()
