import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import can
from src.configure import load_configuration, load_endpoints, setup_odrive
from src.can_utils import discover_node_ids

# Only the three nodes that got reflashed on Aug 2 and reverted to a
# generic default profile instead of their real tuned values. Every
# other node (0,1,3,5,6) already matches its correct profile and is
# deliberately NOT touched here.
TARGETS = {
    2: "8308",
    4: "8308",
    7: "5208",
}

def main():
    bus = can.interface.Bus("can0", interface="socketcan")
    node_ids = set(discover_node_ids(bus))
    config_data = load_configuration()
    endpoints = load_endpoints()

    for node_id, motor_type in TARGETS.items():
        if node_id not in node_ids:
            print(f"[SKIP] node {node_id} not currently on the bus")
            continue
        print(f"Applying '{motor_type}' profile to node {node_id}...")
        settings = config_data[motor_type]["settings"]
        ok = setup_odrive(bus, node_id, settings, endpoints)
        print(f"  -> {'OK, saved to flash' if ok else 'FAILED'}")

    bus.shutdown()

if __name__ == "__main__":
    main()
