import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import can
from src.configure import load_endpoints, setup_odrive
from src.can_utils import discover_node_ids

# Conservative interim wrist current limits (2026-08-07) - the GB36
# gimbal motor's manufacturer page (T-Motor, direct source) does not
# publish a current rating at all, only torque (0.36 Nm) and internal
# resistance (16.4 ohm, matches real measured value exactly). Using the
# real measured torque_constant (0.276 Nm/A), that published torque
# figure only requires ~1.3A - well under the previous 3A soft_max.
# DrJones's call: cap at 1A now, revisit upward only if 1A proves
# insufficient for real use, rather than run at an unverified higher
# limit. Stock GB36 profile defaults were current_soft_max=3,
# current_hard_max=3.5, calibration_current=1.5.
SETTINGS = [
    {"path": "axis0.config.motor.current_soft_max",      "value": 1.0},
    {"path": "axis0.config.motor.current_hard_max",       "value": 1.2},
    {"path": "axis0.config.motor.calibration_current",    "value": 1.0},
    {"path": "axis0.config.calibration_lockin.current",   "value": 1.0},
]

TARGETS = [5, 6]

def main():
    bus = can.interface.Bus("can0", interface="socketcan")
    node_ids = set(discover_node_ids(bus))
    endpoints = load_endpoints()

    for node_id in TARGETS:
        if node_id not in node_ids:
            print(f"[SKIP] node {node_id} not currently on the bus")
            continue
        print(f"Applying wrist current limits to node {node_id}...")
        ok = setup_odrive(bus, node_id, SETTINGS, endpoints)
        print(f"  -> {'OK, saved to flash' if ok else 'FAILED'}")

    bus.shutdown()

if __name__ == "__main__":
    main()
