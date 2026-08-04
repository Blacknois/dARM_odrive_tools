import can
import gamecontroller as gc

bus = can.interface.Bus("can0", interface="socketcan")
node_ids = list(gc.discover_node_ids(bus))
endpoints = gc.load_endpoints()['endpoints']
ep = endpoints['axis0.is_armed']

for nid in node_ids:
    val = gc.read_config(bus, nid, ep['id'], ep['type'])
    print(f"Node {nid}: is_armed={val}")

bus.shutdown()
