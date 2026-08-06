import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # so this can find src/ when run from tools/
import can
from src.configure import load_endpoints, read_config

bus = can.interface.Bus("can0", interface="socketcan")
endpoints = load_endpoints()

for node_id in [0, 7]:
    print(f"--- Node {node_id} ---")
    for path in ["config.dc_bus_overvoltage_trip_level", "config.dc_bus_undervoltage_trip_level", "vbus_voltage"]:
        ep = endpoints["endpoints"][path]
        val = read_config(bus, node_id, ep["id"], ep["type"])
        print(f"{path}: id={ep['id']} type={ep['type']} -> {val}")

bus.shutdown()
