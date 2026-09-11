#!/usr/bin/env python3
"""
chess_move_sequencer.py {move|pick|place} ... [--grip VALUE] [--hover-mm N]

Steps a pick-and-place chess move one waypoint at a time. Split
2026-08-17 into two standard, reusable patterns so each half can be
tested standalone:
  - pick(square):  hover -> descend -> grip -> lift
  - place(square): hover -> descend -> release -> lift
  - move(from,to) = pick(from) then place(to) - same as the old
    single-command behavior.

Each step writes ik_target.json (the SAME file/interface set_ik_target.py
already uses) - it does NOT touch gamecontroller.py, does not arm, and
does not trigger any motion itself. After each waypoint is written, YOU
still have to hold L1 + Triangle on the real controller to actually
execute it - this script only prepares "what's next," never removes that
per-step human confirmation (per explicit project rule, 2026-08-16).

Between each waypoint, the script waits for you to type 'y' here before
writing the next one (hardened 2026-08-17 from a bare Enter-press, which
a stray terminal input event was able to race through unconfirmed - see
session notes), so you have time to trigger it on the controller AND
visually confirm the result (especially real fingertip contact on
descend, and actual grip on a real piece) before moving on.

Usage:
    python3 chess_move_sequencer.py move d2 d4
    python3 chess_move_sequencer.py move d2 d4 --grip -0.5 --hover-mm 40
    python3 chess_move_sequencer.py pick d2
    python3 chess_move_sequencer.py place d4 --grip -0.5
"""
import json
import sys
import os
import argparse
import math
import time
import urllib.request

CALIBRATION_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    'chess_board_calibration_2026-09-09_4corner.json',
)
IK_TARGET_FILE = os.path.expanduser('~/dARM/odrive_tools/ik_target.json')
LOG_FILE = os.path.expanduser('~/dARM/odrive_tools/chess_move_log.jsonl')
POSITIONS_URL = 'http://localhost:8080/positions'

GRIP_OPEN = 0.0     # matches gamecontroller.py's GRIPPER_RELEASE_OPEN
GRIP_CLOSED_DEFAULT = -0.565  # 2026-08-16: new compliant chess_spring fingers (82mm).
# Reverted to an explicit active-close grip step (not passive capture-on-descend) -
# current positioning repeatability isn't tight enough yet to trust a passive funnel
# capture at a precise height; an active close is more forgiving of position error.

# Same raw-node -> URDF joint angle conversion as ros2_joint_bridge_v2.py
# on redPi (kept in sync by hand - source of truth is that file).
DEG = math.pi / 180.0
SCALE_8308 = 40.06
SCALE_SHOULDER = 41.6
SCALE_ELBOW = 40.73
SCALE_WRIST_BEND = 29.1
SCALE_WRIST_ROTATE = 27.7
SHOULDER_CAD_MAX_DEG = 226.0
ELBOW_CAD_MAX_DEG = 245.8
BASE_OFFSET_DEG = 26.59

# finger_1 -> finger_tip fixed offset (see darm.joints on redPi,
# finger_tip_joint). CORRECTED 2026-08-16 evening: this was still the
# 54.5mm-finger value from the FIRST recalibration today - the URDF's
# finger_tip_joint z was updated to 0.082 (real measured 82mm) at
# 18:32 today for the new chess_spring_RIGID fingers, but this
# separately-hardcoded copy was never updated to match. Verified via
# 3 independent IK-solver test poses: constant +27.5mm Z-axis
# discrepancy in every case (0.082 - 0.0545 = 0.0275m exactly) between
# this script's own FK and the solver's URDF-sourced chain. This ONLY
# affected this script's own achieved-position logging/accuracy
# metric (get_live_achieved) - it never touched what target gets
# written to ik_target.json, so real commanded moves were unaffected,
# but every error_mm logged today before this fix has ~27.5mm of
# phantom error baked in and should not be trusted for tuning grip/
# hover values. NOTE: sync_urdf.sh does NOT cover this file - it only
# syncs redPi's description/ <-> the MoveIt VM. Any future CAD/finger
# geometry change needs this constant updated here by hand too.
FINGER_TIP_OFFSET = (-0.039715, 0.004364, 0.082)


def raw_to_joint_angles(pos):
    n0 = pos.get('0', 0.0) or 0.0
    n12 = pos.get('1', 0.0) or 0.0
    n3 = pos.get('3', 0.0) or 0.0
    n4 = pos.get('4', 0.0) or 0.0
    n5 = pos.get('5', 0.0) or 0.0
    n6 = pos.get('6', 0.0) or 0.0
    bend_pos = (n5 - n6) / 2.0
    rotate_pos = (n5 + n6) / 2.0
    return {
        'link_1': -n0 * SCALE_8308 * DEG + BASE_OFFSET_DEG * DEG,
        'link_2': (SHOULDER_CAD_MAX_DEG + n12 * SCALE_SHOULDER) * DEG,
        'link_3': -n3 * SCALE_8308 * DEG,
        'forearm': (ELBOW_CAD_MAX_DEG - n4 * SCALE_ELBOW) * DEG,
        'differential': bend_pos * SCALE_WRIST_BEND * DEG,
        'gripper': rotate_pos * SCALE_WRIST_ROTATE * DEG,
    }


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return ((1, 0, 0), (0, c, -s), (0, s, c))


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, -s, 0), (s, c, 0), (0, 0, 1))


def mat_mult(A, B):
    return tuple(tuple(sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def mat_vec(A, v):
    return tuple(sum(A[i][k] * v[k] for k in range(3)) for i in range(3))


def compose(T1, T2):
    """T = (R, t). Composing T1 after T2: point -> T1(T2(point))."""
    R1, t1 = T1
    R2, t2 = T2
    R = mat_mult(R1, R2)
    t = add(mat_vec(R1, t2), t1)
    return (R, t)


IDENTITY = (((1, 0, 0), (0, 1, 0), (0, 0, 1)), (0, 0, 0))


def fk_finger_tip(angles):
    """Same chain validated by hand earlier this session - base_link
    through finger_1, plus the finger_tip fixed offset."""
    T = IDENTITY
    T = compose(T, (rz(angles['link_1']), (0, 0, 0.14444)))
    T = compose(T, (mat_mult(rx(-113 * DEG), rx(angles['link_2'])), (0, 0, 0.07556)))
    T = compose(T, (rz(angles['link_3']), (0, 0, 0.18708)))
    T = compose(T, (mat_mult(rx(122.9 * DEG), rx(-angles['forearm'])), (0, 0, 0.06292)))
    T = compose(T, (rx(angles['differential']), (0, 0, 0.34)))
    T = compose(T, (rz(-angles['gripper']), (0, 0, 0.02)))
    T = compose(T, (IDENTITY[0], (0.05, -0.0035, 0.091365)))  # finger_1_joint origin (finger1_val=0)
    T = compose(T, (IDENTITY[0], FINGER_TIP_OFFSET))
    R, t = T
    return t


def get_live_achieved():
    """Returns real fingertip world position from live joint positions,
    or None if pos_stream_server.py isn't reachable/healthy. Read-only."""
    try:
        with urllib.request.urlopen(POSITIONS_URL, timeout=2.0) as resp:
            data = json.loads(resp.read())
        if not data.get('_healthy', True):
            return None
        angles = raw_to_joint_angles(data)
        return fk_finger_tip(angles)
    except Exception:
        return None


def cross(a, b):
    return (a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0])


def norm(v):
    return math.sqrt(sum(c*c for c in v))


def sub(a, b):
    return tuple(a[i] - b[i] for i in range(3))


def add(a, b):
    return tuple(a[i] + b[i] for i in range(3))


def scale(v, s):
    return tuple(c * s for c in v)


def board_normal(calib):
    f = tuple(calib['file_unit_vector'])
    r = tuple(calib['rank_unit_vector'])
    n = cross(f, r)
    mag = norm(n)
    n = tuple(c / mag for c in n)
    if n[2] < 0:
        n = tuple(-c for c in n)
    return n


def write_target(x, y, z, grip, label):
    with open(IK_TARGET_FILE, 'w') as f:
        json.dump({'x': round(float(x), 4), 'y': round(float(y), 4),
                   'z': round(float(z), 4), 'grip': round(float(grip), 4),
                   'square': label}, f, indent=2)


def log_result(entry):
    with open(LOG_FILE, 'a') as f:
        f.write(json.dumps(entry) + '\n')


def step(n, total, label, x, y, z, grip, note):
    write_target(x, y, z, grip, label)
    print(f"\n--- Step {n}/{total}: {label} ---")
    print(f"  target: x={x:.4f} y={y:.4f} z={z:.4f} grip={grip:.3f}")
    print(f"  {note}")
    # Hardened 2026-08-17: was a bare input() accepting ANY keystroke as
    # confirmation - real incident earlier today where a stray terminal
    # mouse-escape-code flood raced through all 8 steps of a prior run in
    # under a second with no real physical confirmation (see session
    # notes). Now real pieces are being gripped, so a misfire has real
    # physical stakes (dropped/crushed piece), not just a bad log entry -
    # require a deliberate typed 'y', reject and re-ask on anything else.
    while True:
        resp = input("  Hold L1 + Triangle on the controller now. Type y once settled and confirmed >> ").strip().lower()
        if resp == 'y':
            break
        print("  (not confirmed - type exactly 'y' when ready)")

    achieved = get_live_achieved()
    entry = {
        'time': time.strftime('%Y-%m-%d %H:%M:%S'),
        'step': f'{n}/{total}',
        'label': label,
        'target': [round(x, 4), round(y, 4), round(z, 4)],
        'grip_target': round(grip, 4),
    }
    if achieved is not None:
        error_mm = norm(sub(achieved, (x, y, z))) * 1000.0
        entry['achieved'] = [round(v, 4) for v in achieved]
        entry['error_mm'] = round(error_mm, 1)
        print(f"  achieved: x={achieved[0]:.4f} y={achieved[1]:.4f} z={achieved[2]:.4f}  (error: {error_mm:.1f}mm)")
    else:
        entry['achieved'] = None
        entry['error_mm'] = None
        print("  (couldn't reach pos_stream_server.py to log achieved position)")
    log_result(entry)


def pick_steps(square, xyz, hover_offset, grip):
    """Standard repeatable pick pattern: hover -> descend -> grip -> lift,
    same relative motion regardless of which square it's applied to."""
    hover = add(xyz, hover_offset)
    return [
        (f"{square} hover",   hover, GRIP_OPEN, "Approaching above the square."),
        (f"{square} descend", xyz,   GRIP_OPEN, "CHECK: real fingertip contact on the piece, not just 'arrived'."),
        (f"{square} grip",    xyz,   grip,      "CHECK: gripper actually holding the piece before lifting."),
        (f"{square} lift",    hover, grip,      "Lifting with piece held."),
    ]


def place_steps(square, xyz, hover_offset, grip):
    """Standard repeatable place pattern: hover -> descend -> release ->
    lift, same relative motion regardless of which square it's applied to.
    'grip' is whatever the gripper should hold approaching/leaving with
    (normally the same closed value it picked up with) - only the
    descend/release step itself opens it."""
    hover = add(xyz, hover_offset)
    return [
        (f"{square} hover",   hover, grip,      "Moving above the destination square, still holding piece."),
        (f"{square} descend", xyz,   grip,      "CHECK: real contact at destination before releasing."),
        (f"{square} release", xyz,   GRIP_OPEN, "CHECK: piece released cleanly, sitting on the square."),
        (f"{square} lift",    hover, GRIP_OPEN, "Clearing the board."),
    ]


def run_steps(steps):
    for i, (label, xyz, grip, note) in enumerate(steps, 1):
        step(i, len(steps), label, xyz[0], xyz[1], xyz[2], grip, note)


def load_calib_and_normal(hover_mm):
    with open(CALIBRATION_FILE) as f:
        calib = json.load(f)
    normal = board_normal(calib)
    hover_offset = scale(normal, hover_mm / 1000.0)
    return calib, hover_offset


def check_square(calib, square):
    sq = square.strip().lower()
    if sq not in calib['squares']:
        print(f"Unknown square '{sq}' - expected a1-h8")
        sys.exit(1)
    return sq


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest='cmd', required=True)

    p_move = sub.add_parser('move', help='full pick-and-place: pick(from) then place(to)')
    p_move.add_argument('from_square')
    p_move.add_argument('to_square')
    p_move.add_argument('--grip', type=float, default=GRIP_CLOSED_DEFAULT)
    p_move.add_argument('--hover-mm', type=float, default=40.0)

    p_pick = sub.add_parser('pick', help='pick only: hover -> descend -> grip -> lift, standalone')
    p_pick.add_argument('square')
    p_pick.add_argument('--grip', type=float, default=GRIP_CLOSED_DEFAULT)
    p_pick.add_argument('--hover-mm', type=float, default=40.0)

    p_place = sub.add_parser('place', help='place only: hover -> descend -> release -> lift, standalone (assumes a piece is already held)')
    p_place.add_argument('square')
    p_place.add_argument('--grip', type=float, default=GRIP_CLOSED_DEFAULT,
                          help='the closed-grip value the piece is currently held at')
    p_place.add_argument('--hover-mm', type=float, default=40.0)

    args = parser.parse_args()
    calib, hover_offset = load_calib_and_normal(args.hover_mm)

    if args.cmd == 'pick':
        sq = check_square(calib, args.square)
        steps = pick_steps(sq, tuple(calib['squares'][sq]), hover_offset, args.grip)
        print(f"Pick only: {sq}, {len(steps)} steps, hover={args.hover_mm}mm, grip={args.grip}")
        run_steps(steps)
        print("\nPick complete.")

    elif args.cmd == 'place':
        sq = check_square(calib, args.square)
        steps = place_steps(sq, tuple(calib['squares'][sq]), hover_offset, args.grip)
        print(f"Place only: {sq}, {len(steps)} steps, hover={args.hover_mm}mm, grip={args.grip}")
        run_steps(steps)
        print("\nPlace complete.")

    else:  # move
        fs = check_square(calib, args.from_square)
        ts = check_square(calib, args.to_square)
        steps = (pick_steps(fs, tuple(calib['squares'][fs]), hover_offset, args.grip)
                 + place_steps(ts, tuple(calib['squares'][ts]), hover_offset, args.grip))
        print(f"Sequencing move {fs} -> {ts}, {len(steps)} steps, hover={args.hover_mm}mm, grip={args.grip}")
        run_steps(steps)
        print("\nMove sequence complete.")


if __name__ == '__main__':
    main()
