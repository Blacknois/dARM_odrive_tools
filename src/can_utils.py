import struct
import threading
import time
import can

can_lock = threading.RLock()  # serializes all bus.send/bus.recv across threads
                              # (main loop, watchdog, metrics/UI thread, and any
                              # automated sequence) so one thread's response can't
                              # get stolen by another thread's recv() call.

def extract_node_id(arbitration_id):
    """
    Extracts the node ID from a CAN arbitration ID.
    """
    return arbitration_id >> 5

def discover_node_ids(bus, discovery_duration=0.5):
    """
    Discover ODrive node IDs on the CAN network.
    """
    while bus.recv(timeout=0) is not None:
        pass
    end_time = time.time() + discovery_duration
    node_ids = set()

    while time.time() < end_time:
        try:
            msg = bus.recv(timeout=0)
            if msg:
                node_id = extract_node_id(msg.arbitration_id)
                node_ids.add(node_id)
        except can.CanError:
            pass

    return list(node_ids)

def send_can_message(bus, node_id, command_id, data_format, *data_args):
    """
    Sends a CAN message.
    """
    try:
        message = can.Message(
            arbitration_id=(node_id << 5 | command_id),
            data=struct.pack(data_format, *data_args),
            is_extended_id=False,
        )
        with can_lock:  # keep this send from landing in the middle of another thread's read
            bus.send(message, timeout=0.1)  # bounded - a stuck/bus-off TX queue must not hang the caller (this is on the same path as the emergency disarm)
        return True
    except can.CanError:
        return False

def receive_can_message(bus, expected_arbitration_id):
    """
    Receives a CAN message with a global timeout of 2 milliseconds.
    """
    start_time = time.time()
    while time.time() - start_time < 0.05:
        try:
            msg = bus.recv(timeout=0)  # Explicit timeout per receive attempt
            if msg and msg.arbitration_id == expected_arbitration_id:
                return msg
        except can.CanError:
            return None

    # Timeout reached, return None
    return None
