import sys, os, json, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import can
from src.configure import load_configuration, load_endpoints, read_config
from src.can_utils import discover_node_ids

MOTOR_MAP = {0: "8308", 1: "8308", 2: "8308", 3: "8308", 4: "8308",
             5: "GB36", 6: "GB36", 7: "5208"}

def main():
    bus = can.interface.Bus("can0", interface="socketcan")
    node_ids = sorted(discover_node_ids(bus))
    config_data = load_configuration()
    endpoints = load_endpoints()["endpoints"]

    print(f"Discovered: {node_ids}")
    out = {"dumped_at": time.time(), "dumped_at_iso": time.strftime("%Y-%m-%dT%H:%M:%S"), "nodes": {}}

    for nid in node_ids:
        motor_type = MOTOR_MAP.get(nid)
        if motor_type is None:
            print(f"[SKIP] node {nid} not in MOTOR_MAP")
            continue
        settings = config_data[motor_type]["settings"]
        node_vals = {}
        for setting in settings:
            path = setting["path"]
            if path not in endpoints:
                continue
            # skip write-only/action endpoints like requested_state - not
            # meaningful to "read back" as persistent config
            if path in ("axis0.requested_state",):
                continue
            ep = endpoints[path]
            val = read_config(bus, nid, ep["id"], ep["type"])
            node_vals[path] = val
        out["nodes"][str(nid)] = {"motor_type": motor_type, "live_config": node_vals}
        print(f"node {nid} ({motor_type}): {len(node_vals)} fields read")

    bus.shutdown()

    out_path = sys.argv[1] if len(sys.argv) > 1 else "live_odrive_config_dump.json"
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\nSaved to {out_path}")

if __name__ == "__main__":
    main()
