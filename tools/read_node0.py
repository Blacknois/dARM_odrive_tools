import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # so this can find src/ when run from tools/
import can
from src.configure import load_endpoints, read_config

def main():
    bus = can.interface.Bus("can0", bustype="socketcan")
    data = load_endpoints()

    eps = data["endpoints"]
    print("Type of eps:", type(eps))

    node_id = 0
    printed = 0

    if isinstance(eps, dict):
        iterable = list(eps.values())
    else:
        iterable = eps

    print("Reading endpoints on node 0...")

    for ep in iterable:
        if printed >= 30:
            break
        printed += 1

        endpoint_id = ep["id"]
        endpoint_type = ep["type"]
        endpoint_name = ep.get("name", str(ep))

        try:
            val = read_config(bus, node_id, endpoint_id, endpoint_type)
            print(f"{endpoint_name}: {val}")
        except Exception as e:
            print(f"{endpoint_name}: ERROR ({e})")

    bus.shutdown()

if __name__ == "__main__":
    main()
