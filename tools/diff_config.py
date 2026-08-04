#!/usr/bin/env python3
"""
Reads every axis0.*config* endpoint (plus a few key top-level config
endpoints) for two nodes and prints only the ones that differ, to find
whatever is actually different between a node that arms fine (0) and one
that refuses CLOSED_LOOP_CONTROL (2). Read-only.
"""
import can
from src.can_utils import discover_node_ids
from src.configure import load_endpoints, read_config

NODE_A = 0   # known-good (arms fine)
NODE_B = 2   # known-bad (refuses to arm)

def main():
    bus = can.interface.Bus("can0", interface="socketcan")
    endpoints = load_endpoints()['endpoints']

    node_ids = set(discover_node_ids(bus))
    if NODE_A not in node_ids or NODE_B not in node_ids:
        print(f"One of node {NODE_A}/{NODE_B} not detected. Found: {sorted(node_ids)}")
        bus.shutdown()
        return

    # Focus on config-ish paths under axis0, skip functions/arrays we can't easily read.
    paths = sorted(
        p for p in endpoints.keys()
        if p.startswith('axis0.') and 'config' in p
    )
    print(f"Comparing {len(paths)} axis0 config endpoints between node {NODE_A} and node {NODE_B}...\n")

    diffs = []
    for path in paths:
        ep = endpoints[path]
        try:
            val_a = read_config(bus, NODE_A, ep['id'], ep['type'])
            val_b = read_config(bus, NODE_B, ep['id'], ep['type'])
        except Exception as e:
            print(f"[SKIP] {path}: {e}")
            continue

        if val_a != val_b:
            diffs.append((path, val_a, val_b))

    print(f"\n=== Differences ({len(diffs)}) ===")
    for path, val_a, val_b in diffs:
        print(f"{path:55} node{NODE_A}={val_a!r:12} node{NODE_B}={val_b!r}")

    bus.shutdown()

if __name__ == "__main__":
    main()
