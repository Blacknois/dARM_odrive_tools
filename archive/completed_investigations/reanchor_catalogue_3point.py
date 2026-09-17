#!/usr/bin/env python3
"""Three-point, two-axis reanchor for a chess catalogue.

The two-point reanchor (reanchor_catalogue_2point.py) only captures how
the board's error varies along ONE axis (the line between the two
reference squares) - confirmed 2026-08-20 on the 48mmRoll board: a
fit built from a1/h1 (both rank 1) predicted rank-3/4 squares badly
(errors up to 0.32 rad against real touches), because it had zero
information about how anything varies with rank, only file.

This version uses THREE reference squares forming an L-shape (a shared
corner + one arm along file, one arm along rank - e.g. a1, h1, a8) to
fit a full 2D affine correction per joint:

    correction(file, rank) = c0 + c1*file + c2*rank

Three points is exactly enough to solve this uniquely per node (3
unknowns, 3 equations) - not an average, not a guess, an exact fit
through all three measurements.

Does NOT touch the live control path - reads live positions from
pos_stream_server.py, only ever writes a NEW catalogue file.
gamecontroller.py keeps reading the original until Carla explicitly
swaps this in, after checking a FOURTH, independent square.
"""
import json
import os
import sys
import urllib.request

ODRIVE_TOOLS_DIR = os.path.expanduser('~/dARM/odrive_tools')
POS_URL = 'http://localhost:8080/positions'
NODES = [0, 1, 3, 4, 5, 6]
FILE_NUM = {c: i + 1 for i, c in enumerate('abcdefgh')}


def coords(square):
    return FILE_NUM[square[0]], int(square[1])


def det3(m):
    return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))


def solve3(A, y):
    """Solve the 3x3 linear system A @ x = y via Cramer's rule - no numpy needed."""
    d = det3(A)
    if abs(d) < 1e-9:
        return None
    x = []
    for col in range(3):
        Ai = [row[:] for row in A]
        for r in range(3):
            Ai[r][col] = y[r]
        x.append(det3(Ai) / d)
    return x


def read_live_positions():
    with urllib.request.urlopen(POS_URL, timeout=2) as f:
        data = json.load(f)
    if not data.get('_healthy', False):
        raise RuntimeError("pos_stream_server reports unhealthy - check it's actually running/connected")
    return {n: float(data[str(n)]) for n in NODES}


def touch(square, catalogue):
    if square not in catalogue['squares']:
        print(f"'{square}' is not in the catalogue. Catalogued squares: {sorted(catalogue['squares'].keys())}")
        sys.exit(1)
    old_raw = catalogue['squares'][square]['raw_nodes']
    print(f"\nOld catalogued raw values for {square}: {old_raw}")
    print(f"Manually move the arm (disarmed) so the fingertip is at the TRUE physical")
    print(f"position for square {square} right now. Press Enter when in position...")
    input()
    live = read_live_positions()
    new_raw = {f'node{n}': live[n] for n in NODES}
    print(f"Live raw values just read: {new_raw}")
    return old_raw, new_raw


def main():
    if len(sys.argv) not in (4, 5):
        print("Usage: reanchor_catalogue_3point.py <ref_sq_1> <ref_sq_2> <ref_sq_3> [catalogue_basename]")
        print("  Pick 3 squares forming an L-shape (shared corner + one arm along")
        print("  file, one arm along rank) for a good 2D fit, e.g.: a1 h1 a8")
        sys.exit(1)
    refs = [sys.argv[1].strip().lower(), sys.argv[2].strip().lower(), sys.argv[3].strip().lower()]
    basename = sys.argv[4].strip() if len(sys.argv) == 5 else 'chess_catalogue'
    catalogue_file = os.path.join(ODRIVE_TOOLS_DIR, f'{basename}.json')
    output_file = os.path.join(ODRIVE_TOOLS_DIR, f'{basename}_reanchored_3pt.json')

    with open(catalogue_file) as f:
        catalogue = json.load(f)

    olds, news = [], []
    for i, sq in enumerate(refs, 1):
        print(f"\n=== Reference square {i}/3: {sq} ===")
        old, new = touch(sq, catalogue)
        olds.append(old)
        news.append(new)

    deltas = [{k: news[i][k] - olds[i][k] for k in olds[i]} for i in range(3)]
    for sq, d in zip(refs, deltas):
        print(f"Delta at {sq}: {d}")

    # design matrix [1, file, rank] for the 3 reference points
    A = [[1.0, *coords(sq)] for sq in refs]
    if abs(det3(A)) < 1e-6:
        print("\n[ERROR] The 3 reference squares are collinear (or too close together) - "
              "can't fit a unique 2D plane from them. Pick an L-shape, not a straight line.")
        sys.exit(1)

    coefs = {}
    for k in deltas[0]:
        y = [d[k] for d in deltas]
        c0, c1, c2 = solve3(A, y)
        coefs[k] = (c0, c1, c2)
        print(f"{k}: correction(file,rank) = {c0:+.5f} {c1:+.5f}*file {c2:+.5f}*rank")

    print("\nDoes that look reasonable (not wild)? Type 'yes' to apply it, anything else to abort.")
    if input().strip().lower() != 'yes':
        print("Aborted - no file written.")
        sys.exit(0)

    new_catalogue = {'squares': {}}
    for sq, entry in catalogue['squares'].items():
        old = entry['raw_nodes']
        f, r = coords(sq)
        new_raw = {}
        for k, (c0, c1, c2) in coefs.items():
            new_raw[k] = old[k] + c0 + c1 * f + c2 * r
        new_catalogue['squares'][sq] = {
            'raw_nodes': new_raw,
            'achieved_xyz': entry.get('achieved_xyz'),  # stale after reanchor, kept for reference only
        }

    with open(output_file, 'w') as f_out:
        json.dump(new_catalogue, f_out, indent=2, sort_keys=True)

    print(f"\nWrote 3-point reanchored catalogue to {output_file} ({len(new_catalogue['squares'])} squares).")
    print(f"Original {catalogue_file} untouched. Verify against a FOURTH, independent square before swapping in.")


if __name__ == '__main__':
    main()
