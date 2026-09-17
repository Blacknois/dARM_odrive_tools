import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import can
from src.configure import load_endpoints, setup_odrive
from src.can_utils import discover_node_ids

# Node 0 (base) current limits - 2026-08-11. Never got the same
# data-driven treatment nodes 1-4 got on 2026-08-05 (was still on
# generic 8308 defaults: current_soft_max=40, current_hard_max=41,
# calibration_current=18, calibration_lockin.current=18). DrJones's
# own judgment call (not a fresh max-leverage measurement like nodes
# 1/2/4 got): set deliberately higher than nodes 1-4's 15/17/10 since
# node0 has to rotate the whole arm's mass, not just a single link -
# sanity-checked against torque_constant=0.106 Nm/A (confirmed
# identical across all 8308 nodes) and real-world observation that the
# whole robot can be slid across the floor with one finger, meaning a
# stall against an obstruction would likely just slide the robot
# rather than build up dangerous torque.
SETTINGS = [
    {"path": "axis0.config.motor.current_soft_max",      "value": 20},
    {"path": "axis0.config.motor.current_hard_max",       "value": 25},
    {"path": "axis0.config.motor.calibration_current",    "value": 13},
    {"path": "axis0.config.calibration_lockin.current",   "value": 13},
]

TARGETS = [0]

def main():
    bus = can.interface.Bus("can0", interface="socketcan")
    node_ids = set(discover_node_ids(bus))
    endpoints = load_endpoints()

    for node_id in TARGETS:
        if node_id not in node_ids:
            print(f"[SKIP] node {node_id} not currently on the bus")
            continue
        print(f"Applying current limits to node {node_id}...")
        ok = setup_odrive(bus, node_id, SETTINGS, endpoints)
        print(f"  -> {'OK, saved to flash' if ok else 'FAILED'}")

    bus.shutdown()

if __name__ == "__main__":
    main()
