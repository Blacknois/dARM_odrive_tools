import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import can
from src.configure import load_endpoints, setup_odrive
from src.can_utils import discover_node_ids

# Data-driven current limits based on real measured Iq_measured during
# static max-leverage holds on 2026-08-05 (nodes 1,2,4 directly measured;
# node 3 shares node 4's joint/gearbox/leverage so uses the same values).
# Stock 8308 profile defaults were current_soft_max=40, current_hard_max=41,
# calibration_current=18, calibration_lockin.current=18 - all considerably
# higher than what real self-weight-only load requires.
SETTINGS = [
    {"path": "axis0.config.motor.current_soft_max",      "value": 15},
    {"path": "axis0.config.motor.current_hard_max",       "value": 17},
    {"path": "axis0.config.motor.calibration_current",    "value": 10},
    {"path": "axis0.config.calibration_lockin.current",   "value": 10},
]

TARGETS = [1, 2, 3, 4]

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
