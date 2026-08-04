# odrive_configurator.py
import json
import os
import struct
import time
from src.can_utils import send_can_message, receive_can_message, can_lock

# Constants for ODrive CAN operations
READ  = 0x00
RXSDO = 0x04
TXSDO = 0x05
WRITE = 0x01

# Data type formats for CAN messages
format_lookup = {
    'bool': '?', 'uint8': 'B', 'int8': 'b',
    'uint16': 'H', 'int16': 'h', 'uint32': 'I', 'int32': 'i',
    'uint64': 'Q', 'int64': 'q', 'float': 'f'
}

def load_configuration():
    # Load the configuration file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, '..', 'data', 'config.py')

    config = {}
    with open(config_path, 'r') as config_file:
        exec(config_file.read(), globals(), config)

    return config['config']

def load_endpoints():
    # Load the endpoints file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    endpoints_path = os.path.join(script_dir, '..', 'data', 'flat_endpoints.json')

    with open(endpoints_path, 'r') as f:
        endpoints = json.load(f)

    return endpoints

def read_config(bus, node_id, endpoint_id, endpoint_type, timeout=0.03, retries=3):
    """
    Requests an endpoint value and waits for a matching-node response.
    Drains stale leftover replies before each attempt and retries a few
    times, since a busy node can occasionally have an old reply sitting
    in the queue that gets mistaken for the answer to a fresh request.
    """
    expected_arb_id = node_id << 5 | TXSDO
    fmt = '<BHB' + format_lookup[endpoint_type]
    needed = struct.calcsize(fmt)

    for attempt in range(retries):
        with can_lock:  # hold the bus for this whole attempt so another thread
                         # can't send/recv in between and steal our response
            for _ in range(50):
                if bus.recv(timeout=0) is None:
                    break

            send_can_message(bus, node_id, RXSDO, '<BHB', READ, endpoint_id, 0)

            start_time = time.time()
            while time.time() - start_time < timeout:
                msg = bus.recv(timeout=0)
                if msg is None:
                    continue
                if msg.arbitration_id != expected_arb_id:
                    continue
                if len(msg.data) < needed:
                    continue
                opcode, reply_endpoint_id, reserved, value = struct.unpack_from(fmt, msg.data)
                if reply_endpoint_id != endpoint_id:
                    continue  # stale reply meant for a different endpoint request
                              # on this same node - not our answer, keep waiting
                return value
    return None

def write_config(bus, node_id, endpoint_id, endpoint_type, value):
    # Send the CAN message
    message_format = '<BHB' + format_lookup[endpoint_type]
    send_can_message(bus, node_id, RXSDO, message_format, WRITE, endpoint_id, 0, value)

def validate_config(bus, node_id, endpoint_id, endpoint_type, expected_value, tolerance=1e-2):
    actual_value = read_config(bus, node_id, endpoint_id, endpoint_type)

    if actual_value is None:
        print(f"[ERROR] Node {node_id} - No response for endpoint {endpoint_id}")
        return False

    if isinstance(expected_value, float):
        return abs(actual_value - expected_value) <= tolerance
    else:
        return actual_value == expected_value

def save_config(bus, node_id, save_endpoint_id):
    # Send a command to save the current configuration on an ODrive node
    send_can_message(bus, node_id, RXSDO, '<BHB', WRITE, save_endpoint_id, 0)
    print(f"Configuration saved for node {node_id}")

def check_firmware_hardware_version(bus, node_id, fw_version_expected, hw_version_expected):
    # Check the firmware and hardware version of an ODrive node
    send_can_message(bus, node_id, READ, '')
    response = receive_can_message(bus, node_id << 5 | READ)
    if response:
        _, hw_product_line, hw_version, hw_variant, fw_major, fw_minor, fw_revision, fw_unreleased = struct.unpack('<BBBBBBBB', response.data)
        fw_version_actual = f"{fw_major}.{fw_minor}.{fw_revision}"
        hw_version_actual = f"{hw_product_line}.{hw_version}.{hw_variant}"

        if fw_version_actual != fw_version_expected or hw_version_actual != hw_version_expected:
            raise ValueError(f"Node {node_id} version mismatch. Firmware: {fw_version_actual} - Expected: {fw_version_expected}, Hardware: {hw_version_actual} - Expected: {hw_version_expected}")
    else:
        print(f"[ERROR] No response received when checking firmware and hardware version for node {node_id}.")

def clear_errors(bus, node_id, endpoints, clear=True):
    # List of endpoint paths based on ODrive documentation
    error_endpoints = [
        "axis0.active_errors",        # Active errors on the axis
        "axis0.disarm_reason",        # Reason for disarm
    ]

    # Clears or reads errors for the specified ODrive node.
    for error_endpoint in error_endpoints:
        if error_endpoint in endpoints['endpoints']:
            endpoint_id = endpoints['endpoints'][error_endpoint]['id']
            endpoint_type = endpoints['endpoints'][error_endpoint]['type']
            error_value = read_config(bus, node_id, endpoint_id, endpoint_type)

            if error_value:
                print(f"Node {node_id} - {error_endpoint} - Error: {error_value}")
                if clear and "active_errors" in error_endpoint:  # Only clear active errors
                    write_config(bus, node_id, endpoint_id, endpoint_type, 0)
                    print(f"Node {node_id} - {error_endpoint} - Error cleared.")
            else:
                print(f"Node {node_id} - {error_endpoint} - No error.")
        else:
            print(f"Endpoint {error_endpoint} not found in the provided endpoints.")
        print()

def set_odrive_parameter(bus, node_id, path, value, endpoints, tolerance=1e-2):
    """
    Sets a single parameter on an ODrive node with validation.
    """
    endpoint_id = endpoints['endpoints'][path]['id']
    endpoint_type = endpoints['endpoints'][path]['type']

    # State-request fields self-clear the instant the ODrive processes them,
    # so reading them back to "confirm" the write will always look like a
    # failure even when it worked. Fire the request and move on.
    if path.endswith('requested_state'):
        write_config(bus, node_id, endpoint_id, endpoint_type, value)
        print(f"[INFO] Node {node_id} - {path:50} - State change requested: {value}")
        return True

    current_value = read_config(bus, node_id, endpoint_id, endpoint_type)
    if current_value is None:
        print(f"[ERROR] Node {node_id} - {path:50} - Failed to read current value.")
        return False

    # Compare with tolerance for floats
    if isinstance(current_value, float):
        if abs(current_value - value) <= tolerance:
            print(f"[INFO] Node {node_id} - {path:50} - Already set: {current_value:.6f}")
            return True
    elif current_value == value:
        print(f"[INFO] Node {node_id} - {path:50} - Already set: {current_value}")
        return True

    # Write and validate
    write_config(bus, node_id, endpoint_id, endpoint_type, value)
    if not validate_config(bus, node_id, endpoint_id, endpoint_type, value, tolerance):
        print(f"[ERROR] Node {node_id} - {path:50} - Update failed: Expected {value}, Got {current_value}")
        return False

    print(f"[INFO] Node {node_id} - {path:50} - Updated: {value}")
    return True

def setup_odrive(bus, node_id, settings, endpoints):
    """
    Configures an entire ODrive node using provided settings.
    """
    try:
        for setting in settings:
            path = setting['path']
            value = setting['value']
            if not set_odrive_parameter(bus, node_id, path, value, endpoints):
                print(f"[ERROR] Failed to apply setting {path} to node {node_id}")
                return False

        # Save the configuration
        save_endpoint_id = endpoints['endpoints']['save_configuration']['id']
        save_config(bus, node_id, save_endpoint_id)
        return True
    except Exception as e:
        print(f"[ERROR] Unexpected error during ODrive setup: {e}")
        return False
