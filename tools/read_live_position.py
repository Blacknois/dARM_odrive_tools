#!/usr/bin/env python3
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # so this can find src/ when run from tools/
import argparse
import time
import can
from src.configure import load_endpoints, read_config

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-id", type=int, nargs='+', required=True, help="Node id(s) to watch")
    args = parser.parse_args()

    bus = can.interface.Bus("can0", interface="socketcan")
    endpoints = load_endpoints()['endpoints']
    pos_ep = endpoints['axis0.pos_estimate']

    mins = {nid: None for nid in args.id}
    maxs = {nid: None for nid in args.id}

    print("Reading position (radians), tracking min/max. Ctrl+C to stop.\n")
    try:
        while True:
            parts = []
            for nid in args.id:
                pos = read_config(bus, nid, pos_ep['id'], pos_ep['type'])
                if pos is not None:
                    if mins[nid] is None or pos < mins[nid]: mins[nid] = pos
                    if maxs[nid] is None or pos > maxs[nid]: maxs[nid] = pos
                    parts.append(f"Node {nid}: now={pos:+.4f}  min={mins[nid]:+.4f}  max={maxs[nid]:+.4f}")
                else:
                    parts.append(f"Node {nid}: ---")
            print("\r" + "   |   ".join(parts) + "     ", end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n\nFinal min/max seen this session:")
        for nid in args.id:
            print(f"  Node {nid}: min={mins[nid]:+.4f}  max={maxs[nid]:+.4f}")
    finally:
        bus.shutdown()

if __name__ == "__main__":
    main()
