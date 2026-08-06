import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # so this can find src/ when run from tools/
import can
import time
from src.configure import load_endpoints, read_config, write_config, save_config

bus = can.interface.Bus("can0", interface="socketcan")
endpoints = load_endpoints()

ep = endpoints["endpoints"]["config.dc_bus_overvoltage_trip_level"]
print("Before write:", read_config(bus, 7, ep["id"], ep["type"]))

write_config(bus, 7, ep["id"], ep["type"], 50.0)
print("Immediately after write:", read_config(bus, 7, ep["id"], ep["type"]))

save_ep = endpoints["endpoints"]["save_configuration"]["id"]
save_config(bus, 7, save_ep)
time.sleep(1)

print("After save, same session:", read_config(bus, 7, ep["id"], ep["type"]))

bus.shutdown()
