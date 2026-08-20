#!/usr/bin/env python3
"""Reanchor chess_catalogue.json after a node reset / possible board move.

Does NOT touch the live control path - reads live positions from
pos_stream_server.py (same read-only source calibrate_board.py uses),
and only ever writes a NEW catalogue file. gamecontroller.py keeps
reading the original chess_catalogue.json until Carla explicitly swaps
it in, after checking the reanchored result against a second square.

Single-point raw-joint-space offset: computed from ONE touched-off
reference square, then added identically to every catalogued square's
raw_nodes. This exactly cancels a uniform node/rest_pos zero-point
shift (same offset everywhere, by construction). It only approximately
cancels an actual physical board move (translation/rotation) - accuracy
degrades the further a square is from the touched reference point,
worse if the board rotated. That's exactly why the plan is to verify
against a second, different square before trusting this for real
moves - see darm_feature_backlog.md.
"""
import json
import os
import sys
import time
import urllib.request

ODRIVE_TOOLS_DIR = os.path.expanduser('~/dARM/odrive_tools')
DEFAULT_BASENAME = 'chess_catalogue'  # the original board (MinecraftChess)
POS_URL = 'http://localhost:8080/positions'
NODES = [0, 1, 3, 4, 5, 6]


def read_live_positions():
    with urllib.request.urlopen(POS_URL, timeout=2) as f:
        data = json.load(f)
    if not data.get('_healthy', False):
        raise RuntimeError("pos_stream_server reports unhealthy - check it's actually running/connected")
    return {n: float(data[str(n)]) for n in NODES}


def main():
    if len(sys.argv) not in (2, 3):
        print("Usage: reanchor_catalogue.py <reference_square> [catalogue_basename]")
        print(f"  catalogue_basename defaults to '{DEFAULT_BASENAME}' (the original board).")
        print("  e.g. reanchor_catalogue.py d4 chess_catalogue_48mmRoll")
        sys.exit(1)
    ref_sq = sys.argv[1].strip().lower()
    basename = sys.argv[2].strip() if len(sys.argv) == 3 else DEFAULT_BASENAME
    catalogue_file = os.path.join(ODRIVE_TOOLS_DIR, f'{basename}.json')
    output_file = os.path.join(ODRIVE_TOOLS_DIR, f'{basename}_reanchored.json')

    with open(catalogue_file) as f:
        catalogue = json.load(f)

    if ref_sq not in catalogue['squares']:
        print(f"'{ref_sq}' is not in the catalogue. Catalogued squares: {sorted(catalogue['squares'].keys())}")
        sys.exit(1)

    old_raw = catalogue['squares'][ref_sq]['raw_nodes']
    print(f"Old catalogued raw values for {ref_sq}: {old_raw}")
    print(f"\nManually move the arm (disarmed) so the fingertip is at the TRUE physical")
    print(f"position for square {ref_sq} right now. Press Enter when in position...")
    input()

    live = read_live_positions()
    new_raw = {
        'node0': live[0], 'node1': live[1], 'node3': live[3],
        'node4': live[4], 'node5': live[5], 'node6': live[6],
    }
    print(f"\nLive raw values just read: {new_raw}")

    delta = {k: new_raw[k] - old_raw[k] for k in old_raw}
    print(f"\nComputed delta (new - old) to apply to every square: {delta}")
    print("\nDoes that look like a reasonable-size shift, not something wild? Type 'yes' to apply it, anything else to abort.")
    confirm = input().strip().lower()
    if confirm != 'yes':
        print("Aborted - no file written.")
        sys.exit(0)

    new_catalogue = {'squares': {}}
    for sq, entry in catalogue['squares'].items():
        old = entry['raw_nodes']
        new_catalogue['squares'][sq] = {
            'raw_nodes': {k: old[k] + delta[k] for k in old},
            'achieved_xyz': entry.get('achieved_xyz'),  # stale after reanchor - not used by gamecontroller.py, kept for reference only
        }

    with open(output_file, 'w') as f:
        json.dump(new_catalogue, f, indent=2, sort_keys=True)

    print(f"\nWrote reanchored catalogue to {output_file} ({len(new_catalogue['squares'])} squares).")
    print(f"Original {catalogue_file} untouched. Next: verify against a DIFFERENT square")
    print("before swapping this in for gamecontroller.py to actually use.")


if __name__ == '__main__':
    main()
