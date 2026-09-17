#!/usr/bin/env python3
"""
calibrate_board.py

Two ways to run:

  FULL recalibration (new board, or the board's real shape may have
  changed - e.g. after physically remeasuring square size):
      python3 calibrate_board.py --squares a1,h1,a8 --square-size-mm 51

  FAST re-anchor (same board, robot just rebooted - this hardware has no
  permanent zero reference, see CLAUDE.md - or the board got nudged
  slightly): reuses an existing calibration's SHAPE entirely (unit
  vectors, square size, error-fit corrections all untouched) and just
  repositions the whole map with ONE fresh touch:
      python3 calibrate_board.py --reanchor chess_board_calibration_2026-08-17_auto.json --anchor-square a1

Run the full recalibration every time the board's real geometry needs
re-establishing (first setup, or refining the map with more reference
points over time). Run --reanchor every time nodes reset or the board
moves but is otherwise the same physical board - much faster, one touch
instead of a full multi-point pass.

Deliberately combines BOTH real error sources into a single correction
in the full recalibration, instead of fixing one and losing the other
(real incident 2026-08-17: a target-accuracy-only refit silently lost
the powered-execution flex correction the original map had, and the
same ~30-85mm gap reappeared at squares that had looked solid). For each
reference square: a MANUAL touch (unpowered, by hand - real physical
ground truth for where that square actually is) AND a POWERED
drive-there (Triangle, live-solved - real ground truth for where the
robot actually lands when commanded, including flex/execution error).
The correction fit uses (manual - powered) at each point, since that
single number already captures both target accuracy and execution flex
together - operationally what matters is "if I tell the robot to go to
square X, where does it actually end up."

Never touches gamecontroller.py or arms/disarms anything. Each powered
step still requires holding L1+Triangle on the real controller - this
script only writes ik_target.json and waits for typed confirmation,
exactly like set_ik_target.py/chess_move_sequencer.py already do.
Nothing here removes that per-step human confirmation.

After finishing, it writes a new dated calibration file AND updates
chess_move_sequencer.py / set_ik_target.py to point to it automatically.
"""
import argparse
import importlib.util
import json
import math
import os
import re
import sys
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
POSITIONS_URL = 'http://localhost:8080/positions'
IK_TARGET_FILE = os.path.join(HERE, 'ik_target.json')
FILES = 'abcdefgh'
RANKS = '12345678'

_spec = importlib.util.spec_from_file_location('seq', os.path.join(HERE, 'chess_move_sequencer.py'))
seq = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(seq)


def confirm(prompt):
    while True:
        resp = input(prompt).strip().lower()
        if resp == 'y':
            return
        print("  (type exactly 'y' when ready)")


# Auto-detect stillness instead of requiring a typed 'y' immediately -
# real ergonomic problem found 2026-08-17: manual touches need both hands
# to hold/steady the arm, and powered drives need both hands on the
# controller, so typing a confirmation right after either is genuinely
# hard while balancing. Threshold is deliberately tuned for realistic
# HUMAN hold steadiness (natural hand tremor), not raw encoder precision
# (which is far finer than any human can literally hold to) - a real
# hold against a fixed touch point, or a real settled Triangle-mode
# arrival, should easily clear this; only genuine ongoing motion should
# fail it.
STILL_THRESHOLD = 0.01       # raw units, summed abs diff across tracked axes between polls
STILL_DURATION_S = 1.5       # how long it must stay under threshold to count as "held"
POLL_INTERVAL_S = 0.15
STILLNESS_TIMEOUT_S = 90.0


HAS_MOVED_THRESHOLD = 0.05   # raw units, summed abs diff from where this wait STARTED


def wait_for_stillness(label, target_xyz=None, target_tolerance_m=0.15):
    """target_xyz: if given (only for POWERED steps, never manual touches),
    a captured position must land within target_tolerance_m of it to be
    accepted - real bug found 2026-08-17: without this, a DISARMED/
    unmoving arm is trivially "still" from the very first poll, so a
    forgotten-to-arm run silently captured the arm's stationary starting
    position at every step and reported perfect (0.0mm) residuals for
    data that never moved at all. 150mm default is generous (this is a
    sanity check against "didn't move even close to there," not a
    precision check - Phase 3's fit is what actually measures real
    accuracy).

    Second real bug found the same day: stillness ALONE isn't enough
    either - if the arm was already sitting motionless from before this
    step even started (hadn't been touched/driven yet), it's trivially
    "still" from the very first poll too, capturing the OLD position
    before the user had any chance to actually move it. Now requires
    real movement AWAY from wherever it was when this wait began before
    the stillness countdown is allowed to start at all."""
    print(f'  {label}: move/hold into position now - will auto-capture once steady for {STILL_DURATION_S}s...')
    start = time.time()
    start_pos = get_live_raw()
    last = None
    still_since = None
    moved_away = False
    while True:
        if time.time() - start > STILLNESS_TIMEOUT_S:
            raise RuntimeError(f'timed out waiting for stillness ({label}) - try again')
        raw = get_live_raw()
        if not moved_away:
            if sum(abs(raw[k] - start_pos[k]) for k in raw) >= HAS_MOVED_THRESHOLD:
                moved_away = True
            else:
                last = raw
                time.sleep(POLL_INTERVAL_S)
                continue
        if last is not None:
            diff = sum(abs(raw[k] - last[k]) for k in raw)
            if diff < STILL_THRESHOLD:
                if still_since is None:
                    still_since = time.time()
                elif time.time() - still_since >= STILL_DURATION_S:
                    if target_xyz is not None:
                        achieved = fk_from_raw(raw)
                        err = math.sqrt(sum((achieved[i] - target_xyz[i]) ** 2 for i in range(3)))
                        if err > target_tolerance_m:
                            print(f'  {label}: still, but {err*1000:.0f}mm from target - not armed/moved yet? still waiting...')
                            still_since = None
                            last = raw
                            time.sleep(POLL_INTERVAL_S)
                            continue
                    print(f'  {label}: captured.')
                    return raw
            else:
                still_since = None
        last = raw
        time.sleep(POLL_INTERVAL_S)


def get_live_raw():
    with urllib.request.urlopen(POSITIONS_URL, timeout=2.0) as resp:
        data = json.loads(resp.read())
    if not data.get('_healthy', True):
        raise RuntimeError('pos_stream_server reports unhealthy')
    return {k: data[k] for k in ('0', '1', '3', '4', '5', '6')}


def fk_from_raw(raw):
    angles = seq.raw_to_joint_angles(raw)
    return seq.fk_finger_tip(angles)


def write_target(xyz, label):
    with open(IK_TARGET_FILE, 'w') as f:
        json.dump({'x': round(xyz[0], 4), 'y': round(xyz[1], 4), 'z': round(xyz[2], 4),
                   'square': label}, f, indent=2)


def solve3(M, b):
    M = [row[:] for row in M]
    b = b[:]
    for i in range(3):
        piv = M[i][i]
        for j in range(i, 3):
            M[i][j] /= piv
        b[i] /= piv
        for k in range(3):
            if k != i:
                f = M[k][i]
                for j in range(i, 3):
                    M[k][j] -= f * M[i][j]
                b[k] -= f * b[i]
    return b


def fit_plane(rows):
    """rows: list of (fi, ri, value). Fits value = a + b*fi + c*ri."""
    ATA = [[0.0] * 3 for _ in range(3)]
    ATy = [0.0] * 3
    for fi, ri, y in rows:
        a = [1.0, fi, ri]
        for i in range(3):
            ATy[i] += a[i] * y
            for j in range(3):
                ATA[i][j] += a[i] * a[j]
    return solve3(ATA, ATy)


def norm(v): return math.sqrt(sum(c * c for c in v))
def sub(a, b): return tuple(a[i] - b[i] for i in range(3))
def add(a, b): return tuple(a[i] + b[i] for i in range(3))
def scale(v, s): return tuple(c * s for c in v)
def unit(v):
    n = norm(v)
    return tuple(c / n for c in v)


def deploy(out_path):
    fname = os.path.basename(out_path)
    for target_file in ('chess_move_sequencer.py', 'set_ik_target.py'):
        path = os.path.join(HERE, target_file)
        with open(path) as f:
            content = f.read()
        new_content, n = re.subn(
            r"'chess_board_calibration_[^']*\.json',",
            f"'{fname}',",
            content, count=1)
        if n == 1:
            with open(path, 'w') as f:
                f.write(new_content)
            print(f'  updated {target_file} -> {fname}')
        else:
            print(f'  WARNING: could not find calibration file reference in {target_file} - update manually')


def reanchor(existing_path, anchor_square):
    """Fast path: reuse an existing calibration's map SHAPE entirely and
    just re-anchor it in space with ONE fresh touch. Assumes a pure
    translation, not a rotation - if the board actually rotated, use the
    full recalibration path instead."""
    with open(existing_path) as f:
        old = json.load(f)
    if anchor_square not in old['squares']:
        print(f"'{anchor_square}' not in {existing_path} - can't re-anchor from it.")
        sys.exit(1)
    old_pos = tuple(old['squares'][anchor_square])

    new_pos = fk_from_raw(wait_for_stillness(f'Touch {anchor_square} by hand (unpowered)'))
    shift = sub(new_pos, old_pos)
    print(f'  Shift from last known position: {[round(v,4) for v in shift]}  (mag {norm(shift)*1000:.1f}mm)')

    squares_out = {sq: [round(xyz[i] + shift[i], 4) for i in range(3)]
                   for sq, xyz in old['squares'].items()}

    date_str = time.strftime('%Y-%m-%d')
    out_path = os.path.join(HERE, f'chess_board_calibration_{date_str}_reanchored.json')
    out = dict(old)
    out['calibrated'] = f'{date_str} (re-anchored from {os.path.basename(existing_path)} via {anchor_square}, shape unchanged)'
    out['squares'] = squares_out
    with open(out_path, 'w') as f:
        json.dump(out, f, indent=2)
    print(f'\nWrote {out_path}')
    deploy(out_path)


def full_recalibration(ref_squares, square_size_mm, rows):
    row_lo, row_hi = (int(x) for x in rows.split('-'))
    square_size = square_size_mm / 1000.0

    manual = {}
    print(f'=== Phase 1: MANUAL touches ({len(ref_squares)} squares) ===')
    for sq in ref_squares:
        manual[sq] = fk_from_raw(wait_for_stillness(f'Touch {sq} by hand (unpowered)'))
        print(f'  {sq}: {[round(v, 4) for v in manual[sq]]}')

    anchor_sq = ref_squares[0]
    anchor = manual[anchor_sq]

    file_rows, rank_rows = [], []
    for sq in ref_squares:
        fi = FILES.index(sq[0]) - FILES.index(anchor_sq[0])
        ri = RANKS.index(sq[1]) - RANKS.index(anchor_sq[1])
        d = sub(manual[sq], anchor)
        file_rows.append((fi, d))
        rank_rows.append((ri, d))

    def fit_direction(rows_):
        num = [0.0, 0.0, 0.0]
        den = 0.0
        for n, d in rows_:
            if n == 0:
                continue
            for k in range(3):
                num[k] += n * d[k]
            den += n * n
        if den == 0:
            raise RuntimeError('reference squares do not vary enough along this axis')
        return unit(tuple(c / den for c in num))

    file_u = fit_direction(file_rows)
    rank_u = fit_direction(rank_rows)

    def prelim_square(sq):
        fi = FILES.index(sq[0]) - FILES.index(anchor_sq[0])
        ri = RANKS.index(sq[1]) - RANKS.index(anchor_sq[1])
        return add(add(anchor, scale(file_u, fi * square_size)), scale(rank_u, ri * square_size))

    # NOTE 2026-08-17: this used to compute a "retreat to a safe height"
    # step from gamecontroller.py's SAFE_UP_* raw joint values before each
    # descend, to avoid a big direct diagonal jump between distant
    # squares (real incident: h1->a8 swept toward the board). That
    # computation had a real bug - fk_finger_tip()'s hardcoded rotation
    # offsets (-113deg, 122.9deg) nearly cancel those particular SAFE_UP
    # values, producing an almost fully-extended straight-line pose
    # (~1.003m out, stacking every link length) instead of the compact
    # folded pose those values actually produce on the real robot. Live-
    # tested: the arm visibly strained toward near-full physical extension
    # trying to reach the bogus computed target before being stopped.
    # Removed for now rather than patched under time pressure - back to a
    # single direct step, same as every other Triangle-driven move today,
    # visually supervised. The big-jump safety problem is real and still
    # unsolved - see session notes / feature backlog.

    print('\n=== Manual touches done. Before Phase 2: ===')
    print('  1. The arm is disarmed and sitting wherever the last touch left it -')
    print('     manually return it BY HAND to its normal rest_pos posture')
    print('     (arming is refused unless it is actually at rest_pos).')
    print('  2. Arm the robot (L1+Circle, wait for green flashing).')
    confirm('  Type y once armed and at rest_pos >> ')

    print('\n=== Phase 2: POWERED drives to the same reference squares ===')
    print('(hold L1+Triangle on the controller for each, exactly like normal -')
    print('a single direct move per square, watch it visually and let go if')
    print('anything looks wrong, same as any other Triangle-driven target.)\n')
    powered = {}
    for sq in ref_squares:
        target = prelim_square(sq)
        write_target(target, sq)
        print(f'  Target for {sq}: {[round(v, 4) for v in target]}')
        print('  Hold L1+Triangle...')
        powered[sq] = fk_from_raw(wait_for_stillness(f'{sq}', target_xyz=target))
        err = norm(sub(powered[sq], manual[sq])) * 1000.0
        print(f'  {sq}: powered={[round(v, 4) for v in powered[sq]]}  (vs manual, {err:.1f}mm off)')

    print('\n=== Phase 3: fitting combined correction ===')
    rows_by_axis = [[] for _ in range(3)]
    for sq in ref_squares:
        fi = FILES.index(sq[0]) - FILES.index(anchor_sq[0])
        ri = RANKS.index(sq[1]) - RANKS.index(anchor_sq[1])
        corr = sub(manual[sq], powered[sq])
        for ax in range(3):
            rows_by_axis[ax].append((fi, ri, corr[ax]))
    coefs = [fit_plane(rows_by_axis[ax]) for ax in range(3)]

    print('residuals after fit:')
    for sq in ref_squares:
        fi = FILES.index(sq[0]) - FILES.index(anchor_sq[0])
        ri = RANKS.index(sq[1]) - RANKS.index(anchor_sq[1])
        corr = sub(manual[sq], powered[sq])
        pred = [coefs[ax][0] + coefs[ax][1] * fi + coefs[ax][2] * ri for ax in range(3)]
        resid = [corr[ax] - pred[ax] for ax in range(3)]
        print(f'  {sq}: residual = {norm(resid)*1000:.1f}mm')

    print('\n=== Phase 4: generating full board ===')
    squares_out = {}
    for f_ in FILES:
        for r_i, r_ in enumerate(RANKS):
            r_num = r_i + 1
            if not (row_lo <= r_num <= row_hi):
                continue
            sq = f_ + r_
            fi = FILES.index(f_) - FILES.index(anchor_sq[0])
            ri = r_i - RANKS.index(anchor_sq[1])
            base = add(add(anchor, scale(file_u, fi * square_size)), scale(rank_u, ri * square_size))
            corr = [coefs[ax][0] + coefs[ax][1] * fi + coefs[ax][2] * ri for ax in range(3)]
            squares_out[sq] = [round(base[i] + corr[i], 4) for i in range(3)]

    date_str = time.strftime('%Y-%m-%d')
    out_path = os.path.join(HERE, f'chess_board_calibration_{date_str}_auto.json')
    out = {
        'calibrated': f'{date_str} (automated calibrate_board.py - combined manual+powered correction)',
        'reference_frame': 'world (robot base_link-anchored, Z-up)',
        'touch_reference': 'finger_tip TF frame',
        'square_size_m': square_size,
        'reference_squares': ref_squares,
        'file_unit_vector': list(file_u),
        'rank_unit_vector': list(rank_u),
        'error_fit_coefficients': {str(ax): coefs[ax] for ax in range(3)},
        'squares': squares_out,
    }
    with open(out_path, 'w') as f:
        json.dump(out, f, indent=2)
    print(f'\nWrote {out_path}')
    deploy(out_path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--squares', help='comma-separated reference squares for a FULL recalibration, e.g. a1,h1,a8')
    ap.add_argument('--square-size-mm', type=float, help='real measured square size in mm (full recalibration only)')
    ap.add_argument('--rows', default='1-7', help='rank range to generate, e.g. 1-7 (deferring row 8 by default)')
    ap.add_argument('--reanchor', help='path to an existing calibration file to reposition instead of a full recalibration')
    ap.add_argument('--anchor-square', help='which square to touch for --reanchor (must exist in that file)')
    args = ap.parse_args()

    if args.reanchor:
        if not args.anchor_square:
            print('--reanchor requires --anchor-square too')
            sys.exit(1)
        reanchor(args.reanchor, args.anchor_square.strip().lower())
        return

    if not args.squares or not args.square_size_mm:
        print('Full recalibration needs --squares and --square-size-mm (or use --reanchor for the fast path)')
        sys.exit(1)

    ref_squares = [s.strip().lower() for s in args.squares.split(',')]
    if len(ref_squares) < 3:
        print('Need at least 3 reference squares (not all on the same file or rank) to define the board plane.')
        sys.exit(1)

    full_recalibration(ref_squares, args.square_size_mm, args.rows)


if __name__ == '__main__':
    main()
