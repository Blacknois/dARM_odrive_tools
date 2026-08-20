#!/usr/bin/env python3
"""
move_pick_place.py <from_square> <to_square>

Writes the first target, waits for you to physically complete the move
+ pick, then writes the second target and waits for you to physically
complete the move + place. Removes the friction of retyping
set_ik_target.py mid-sequence - it does NOT remove any physical safety
gate. You still hold Triangle to move, Square to pick, Triangle to
move again, Cross to place - exactly like doing it by hand, just
without breaking away from the controller to type a second command in
the middle.

Does NOT touch the live control path itself - purely writes the same
small JSON file set_ik_target.py already writes, at two points instead
of one. Never arms, disarms, or triggers anything by itself.

Usage:
    python3 move_pick_place.py d5 f7
"""
import json
import sys
import os

CALIBRATION_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    'chess_board_calibration_2026-08-20_48mmRoll.json',
)
IK_TARGET_FILE = os.path.expanduser('~/dARM/odrive_tools/ik_target.json')


def write_target(calib, square):
    if square not in calib['squares']:
        print(f"Unknown square '{square}' - expected a1-h8")
        sys.exit(1)
    x, y, z = calib['squares'][square]
    with open(IK_TARGET_FILE, 'w') as f:
        json.dump({'x': x, 'y': y, 'z': z, 'square': square}, f, indent=2)
    print(f"Target set: {square} -> x={x}, y={y}, z={z}")


def main():
    if len(sys.argv) != 3:
        print("Usage: python3 move_pick_place.py <from_square> <to_square>  (e.g. d5 f7)")
        sys.exit(1)

    from_sq = sys.argv[1].strip().lower()
    to_sq = sys.argv[2].strip().lower()

    with open(CALIBRATION_FILE) as f:
        calib = json.load(f)

    print(f"=== Step 1/2: {from_sq} (pick) ===")
    write_target(calib, from_sq)
    print("Hold L1+Triangle on the controller to drive there, then L1+Square to pick.")
    input(f"Press Enter here once the pick at {from_sq} is done >> ")

    print(f"\n=== Step 2/2: {to_sq} (place) ===")
    write_target(calib, to_sq)
    print("Hold L1+Triangle on the controller to drive there, then L1+Cross to place.")
    input(f"Press Enter here once the place at {to_sq} is done >> ")

    print(f"\nDone: {from_sq} -> {to_sq}")


if __name__ == '__main__':
    main()
