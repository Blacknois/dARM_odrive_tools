import sys
import time
import can
from src.configure import load_endpoints, read_config
from src.can_utils import discover_node_ids

# Reject a single reading if it jumps further than this from the last
# accepted reading for that node in one poll (0.1s). Real motion at any
# speed you'd do by hand won't cover this much distance that fast - a
# jump this big is a corrupted/garbled CAN read, not real movement.
MAX_PLAUSIBLE_DELTA = 2.0


def main():
    node_ids = [int(a) for a in sys.argv[1:]] if len(sys.argv) > 1 else None
    bus = can.interface.Bus("can0", interface="socketcan")
    try:
        if not node_ids:
            node_ids = sorted(discover_node_ids(bus))
        endpoints = load_endpoints()["endpoints"]
        pos_ep = endpoints["axis0.pos_estimate"]

        mins = {nid: None for nid in node_ids}
        maxs = {nid: None for nid in node_ids}
        last_good = {nid: None for nid in node_ids}
        glitch_count = {nid: 0 for nid in node_ids}

        print(f"Watching nodes {node_ids}. Read-only - does not touch arming or motion.")
        print("Move the arm slowly through the range you want to measure.")
        print("Press Ctrl+C when done for a min/max summary.\n")

        while True:
            parts = []
            for nid in node_ids:
                pos = read_config(bus, nid, pos_ep["id"], pos_ep["type"])

                if pos is not None and last_good[nid] is not None:
                    if abs(pos - last_good[nid]) > MAX_PLAUSIBLE_DELTA:
                        glitch_count[nid] += 1
                        pos = None  # ignore this sample, treat like a failed read

                if pos is not None:
                    last_good[nid] = pos
                    if mins[nid] is None or pos < mins[nid]:
                        mins[nid] = pos
                    if maxs[nid] is None or pos > maxs[nid]:
                        maxs[nid] = pos
                    parts.append(f"n{nid}={pos:6.2f} (min {mins[nid]:6.2f} max {maxs[nid]:6.2f})")
                else:
                    shown = last_good[nid] if last_good[nid] is not None else float("nan")
                    parts.append(f"n{nid}={shown:6.2f} [glitch ignored]")
            print("\r" + "   ".join(parts), end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\n--- Summary ---")
        for nid in node_ids:
            print(f"Node {nid}: min={mins[nid]}, max={maxs[nid]}  (ignored {glitch_count[nid]} bad reading(s))")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
