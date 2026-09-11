#!/usr/bin/env python3
"""
set_ik_target.py <square>

Looks up a chess square (e.g. "a5") in the board calibration file and
writes its real-world coordinates to ik_target.json, which
gamecontroller.py's IK-mode reads the next time Triangle is held.

Does NOT touch the live control path itself - purely writes a small
JSON file that gamecontroller.py reads defensively (falls back to its
own hardcoded default on any problem with this file). Does not arm,
move, or trigger anything by itself - the human still has to hold
Triangle to actually command the move.

Usage:
    python3 set_ik_target.py a5
"""
import json
import sys
import os

CALIBRATION_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    'chess_board_calibration_2026-09-09_4corner.json',
)
IK_TARGET_FILE = os.path.expanduser('~/dARM/odrive_tools/ik_target.json')


def main():
    if len(sys.argv) != 2:
        print("Usage: python3 set_ik_target.py <square>  (e.g. a5)")
        sys.exit(1)

    square = sys.argv[1].strip().lower()

    with open(CALIBRATION_FILE) as f:
        calib = json.load(f)

    if square not in calib['squares']:
        print(f"Unknown square '{square}' - expected a1-h8")
        sys.exit(1)

    x, y, z = calib['squares'][square]
    with open(IK_TARGET_FILE, 'w') as f:
        json.dump({'x': x, 'y': y, 'z': z, 'square': square}, f, indent=2)

    print(f"Target set: {square} -> x={x}, y={y}, z={z}")
    print("Now hold Triangle on the controller to drive there.")


if __name__ == '__main__':
    main()
