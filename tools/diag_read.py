import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # so this can find src/ when run from tools/
import can
from src.configure import load_endpoints, read_config

bus = can.interface.Bus("can0", interface="socketcan")
endpoints = load_endpoints()

for path in ["vbus_voltage", "config.dc_bus_overvoltage_trip_level"]:
    ep = endpoints["endpoints"][path]
    val = read_config(bus, 0, ep["id"], ep["type"])
    print(f"{path}: id={ep['id']} type={ep['type']} -> {val}")

bus.shutdown()
