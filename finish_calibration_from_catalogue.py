#!/usr/bin/env python3
"""Finishes a calibrate_board.py full recalibration using ALREADY-CAPTURED
powered touches from tonight's real catalogue work, instead of doing a
fresh automated Triangle-drive per reference square (which isn't safe to
do blind right now - no path planning, board is close to the arm).

Reuses calibrate_board.py's own fk_from_raw/fit_plane/solve3 - same math,
same output format, just fed powered data from a source that's already
real and armed/powered, rather than a new live move.
"""
import importlib.util
import json
import math
import os
import time

HERE = os.path.expanduser('~/dARM/odrive_tools')
FILES = 'abcdefgh'
RANKS = '12345678'

_spec = importlib.util.spec_from_file_location('cb', os.path.join(HERE, 'calibrate_board.py'))
cb = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cb)

# --- Phase 1 (manual touches) - already done live, pasted from the terminal ---
manual = {
    'a1': [0.1116, 0.2337, 0.2355],
    'h1': [0.2539, -0.0485, 0.2346],
    'a8': [0.4219, 0.3582, 0.2848],
}
anchor_sq = 'a1'
square_size = 0.048  # 48mm

# --- Phase 2 (powered touches) - reused from tonight's real catalogue captures ---
catalogue = json.load(open(os.path.join(HERE, 'chess_catalogue_48mmRoll_current.json')))['squares']

def raw_from_catalogue(sq):
    rn = catalogue[sq]['raw_nodes']
    return {'0': rn['node0'], '1': rn['node1'], '3': rn['node3'],
            '4': rn['node4'], '5': rn['node5'], '6': rn['node6']}

powered = {}
for sq in manual:
    powered[sq] = cb.fk_from_raw(raw_from_catalogue(sq))
    err = math.dist(powered[sq], manual[sq]) * 1000.0
    print(f'{sq}: manual={[round(v,4) for v in manual[sq]]}  powered={[round(v,4) for v in powered[sq]]}  ({err:.1f}mm apart)')

# --- Phase 3: fit file/rank unit vectors from the manual touches (same as calibrate_board.py) ---
anchor = manual[anchor_sq]

def sub(a, b): return [a[i]-b[i] for i in range(3)]
def add(a, b): return [a[i]+b[i] for i in range(3)]
def scale(a, s): return [a[i]*s for i in range(3)]
def norm(a): return math.sqrt(sum(v*v for v in a))
def unit(a):
    n = norm(a)
    return [v/n for v in a]

file_rows, rank_rows = [], []
for sq in manual:
    fi = FILES.index(sq[0]) - FILES.index(anchor_sq[0])
    ri = RANKS.index(sq[1]) - RANKS.index(anchor_sq[1])
    d = sub(manual[sq], anchor)
    if fi != 0:
        file_rows.append((fi, d))
    if ri != 0:
        rank_rows.append((ri, d))

def fit_direction(rows_):
    num = [0.0, 0.0, 0.0]
    den = 0.0
    for n, d in rows_:
        for k in range(3):
            num[k] += n * d[k]
        den += n * n
    return unit([c/den for c in num])

file_u = fit_direction(file_rows)
rank_u = fit_direction(rank_rows)

# --- Phase 3b: fit the manual-minus-powered correction plane, exactly like calibrate_board.py ---
rows_by_axis = [[] for _ in range(3)]
for sq in manual:
    fi = FILES.index(sq[0]) - FILES.index(anchor_sq[0])
    ri = RANKS.index(sq[1]) - RANKS.index(anchor_sq[1])
    corr = sub(manual[sq], powered[sq])
    for ax in range(3):
        rows_by_axis[ax].append((fi, ri, corr[ax]))
coefs = [cb.fit_plane(rows_by_axis[ax]) for ax in range(3)]

print('\nresiduals after fit:')
for sq in manual:
    fi = FILES.index(sq[0]) - FILES.index(anchor_sq[0])
    ri = RANKS.index(sq[1]) - RANKS.index(anchor_sq[1])
    corr = sub(manual[sq], powered[sq])
    pred = [coefs[ax][0] + coefs[ax][1]*fi + coefs[ax][2]*ri for ax in range(3)]
    resid = sub(corr, pred)
    print(f'  {sq}: residual = {norm(resid)*1000:.1f}mm')

# --- Phase 4: generate the full board ---
squares_out = {}
for f_ in FILES:
    for r_i, r_ in enumerate(RANKS):
        r_num = r_i + 1
        if not (1 <= r_num <= 7):  # matches calibrate_board.py's default --rows 1-7
            continue
        sq = f_ + r_
        fi = FILES.index(f_) - FILES.index(anchor_sq[0])
        ri = r_i - RANKS.index(anchor_sq[1])
        base = add(add(anchor, scale(file_u, fi*square_size)), scale(rank_u, ri*square_size))
        corr = [coefs[ax][0] + coefs[ax][1]*fi + coefs[ax][2]*ri for ax in range(3)]
        squares_out[sq] = [round(base[i]+corr[i], 4) for i in range(3)]

date_str = time.strftime('%Y-%m-%d')
out_path = os.path.join(HERE, f'chess_board_calibration_{date_str}_48mmRoll.json')
out = {
    'calibrated': f'{date_str} (finish_calibration_from_catalogue.py - manual touches live, '
                  'powered touches reused from real armed catalogue captures, not a fresh Triangle-drive)',
    'reference_frame': 'world (robot base_link-anchored, Z-up)',
    'touch_reference': 'finger_tip TF frame',
    'square_size_m': square_size,
    'reference_squares': list(manual.keys()),
    'file_unit_vector': list(file_u),
    'rank_unit_vector': list(rank_u),
    'error_fit_coefficients': {str(ax): coefs[ax] for ax in range(3)},
    'squares': squares_out,
}
with open(out_path, 'w') as f:
    json.dump(out, f, indent=2)
print(f'\nWrote {out_path} ({len(squares_out)} squares)')
