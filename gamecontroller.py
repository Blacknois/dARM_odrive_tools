#!/usr/bin/env python3

import time
import os
import threading
import signal
import urwid
import pygame
import can
import builtins
import urllib.request
import json
from datetime import datetime

from src.can_utils import discover_node_ids
from src.control import move_odrive_to_position, set_closed_loop_control, set_idle_mode
from src.metrics import get_metrics, METRIC_ENDPOINTS
from src.configure import load_endpoints, read_config, write_config

# ------------------------------------------------------------------------------
# Background session logger - purely additive, does not touch any control,
# arming, or safety logic. urwid redraws over the whole terminal constantly,
# so scrollback is unreliable; this file is not. Every print() call already
# in this script also gets appended here, with a timestamp.
_session_log = open("session_log.txt", "a", buffering=1)
_session_log.write(f"\n===== session started {datetime.now().isoformat(timespec='seconds')} =====\n")
_real_print = builtins.print
def _logging_print(*args, **kwargs):
    _real_print(*args, **kwargs)
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    text = kwargs.get("sep", " ").join(str(a) for a in args)
    _session_log.write(f"[{ts}] {text}\n")
builtins.print = _logging_print
# ------------------------------------------------------------------------------
# 1) Button and Axis Definitions - https://www.pygame.org/docs/ref/joystick.html
# ------------------------------------------------------------------------------
DEAD_MAN_BUTTON_INDEX   = 4  # LB
MODE_TOGGLE_BUTTON_INDEX = 5 # RB
ARM_BUTTON_INDEX        = 1     # Circle
PS_BUTTON_INDEX         = 10    # PS button
# Arm gesture used to be D-pad-left + Circle. Switched to L1 + Circle
# 2026-08-10: DualSense's d-pad has known-flaky hat/button reporting on
# Linux (real SDL compatibility issue, not a hardware fault - see
# libsdl-org/SDL#8754), which combined with this gesture's strict
# same-frame-required hold logic caused unreliable arming. L1 (already
# DEAD_MAN_BUTTON_INDEX) is a plain button with no such quirk.
ARM_HOLD_SECONDS        = 1.5
DISARM_HOLD_SECONDS     = 1.0

# IK-mode trigger (2026-08-13, first version). Triangle - unused by
# anything else (RB is already wrist_mode, L1+Circle is arming).
# Drives base/shoulder/elbow-roll/elbow toward a solved target using
# the SAME taper_increment() safety logic as manual moves - not new
# motion math, just a different source of "which way to move".
# Deliberately excludes wrist (bend/rotate) - that joint's dynamic
# envelope math (limits that depend on the OTHER axis's live value)
# needs its own careful integration, not done yet. Also excludes
# gripper - IK solves for reaching a point, not for grasping, so
# there is no gripper target to drive toward here.
# ANY manual stick input immediately cancels IK-mode back to the
# human - it never fights a human input.
IK_TRIGGER_BUTTON_INDEX = 2  # Triangle - empirically confirmed 2026-08-13 via
                             # button_finder.py, NOT 3 as standard PS-controller
                             # ordering would suggest. This DualSense/SDL/Linux
                             # setup has already proven not to reliably follow
                             # standard button ordering (see the d-pad SDL
                             # compatibility issue that moved the arm gesture
                             # off the d-pad originally) - don't assume, verify.
IK_HOLD_SECONDS         = 1.5
IK_VELOCITY_SCALING     = 0.45  # 2026-08-13: was 0.6, lowered further per DrJones's
                                 # own report ("still about the fastest I drive it") -
                                 # his manual driving already runs through the
                                 # squared/expo dead-zone curve below, so his typical
                                 # EFFECTIVE speed at his usual 30-70% stick travel is
                                 # often well under even this. This is the flat ceiling
                                 # the ramp-up below builds toward, not commanded
                                 # instantly - see IK_RAMP_UP_SECONDS.
IK_RAMP_UP_SECONDS      = 1.5  # 2026-08-13: real first-test finding - moving all 4
                                # IK-controlled joints simultaneously from a standing
                                # start produced audible creaking/cracking (DrJones's
                                # own report) - a plausible real contributor is
                                # commanding near-full speed on all 4 joints at once
                                # with no ramp, unlike manual driving's stick-travel-
                                # based easing. Smoothly ramps commanded speed from 0
                                # to IK_VELOCITY_SCALING over this many seconds from
                                # trigger, synchronized across all 4 joints (all
                                # reference the same ik_start_time, so none jerks
                                # ahead of the others).
TARGET_ARRIVAL_DECEL_ZONE = 0.8  # 2026-08-17: programmed moves (Triangle AND the pick/
                                  # place buttons) were reaching cruising speed only once
                                  # within 1.0 raw units of target (the implicit clamp in
                                  # max(-1.0, min(1.0, diff / TARGET_ARRIVAL_DECEL_ZONE))), then decelerating
                                  # proportionally the entire rest of the way - for
                                  # smaller-range joints (shoulder span 5.5, elbow span
                                  # 6.05) that's ~20% of the whole joint spent decelerating,
                                  # not just a final approach. Dividing diff by this before
                                  # clamping reaches full speed sooner (once remaining
                                  # distance >= 0.8) and narrows the decel window - global
                                  # for now (same for every joint), not yet per-joint like
                                  # the existing DECEL_ZONE*/DECEL_ZONE_WRIST_*/
                                  # DECEL_ZONE_GRIPPER constants are for limit-deceleration.
IK_ARRIVAL_TOLERANCE    = 0.08  # same units as joint_positions/shoulder_ctrl.value.
                                 # History: started 0.05 (too close to documented real
                                 # per-event creep of ~0.025-0.074, risked chatter) ->
                                 # widened to 0.15 -> confirmed working on a real
                                 # successful run 2026-08-13 -> tightened here to 0.08,
                                 # still safely above the noise floor but closer than
                                 # 0.15's margin, per DrJones's own call while testing live.
IK_SERVICE_URL          = "http://192.168.1.119:8901/solve"
# Overall hard timeout - covers BOTH failure modes with one backstop: a
# target that's valid per the IK service's own bounds check but not
# actually reachable within this script's real hand-measured limits
# (two independently-maintained limit systems, never cross-validated),
# and a settle-check that never passes. Either way, give up cleanly by
# this point rather than running indefinitely.
IK_TIMEOUT_SECONDS      = 20.0
# Settle-check: once the COMMANDED trajectory reaches target, wait this
# long, then do ONE real live-position read (not a repeating correction
# loop - see design discussion 2026-08-13) against a WIDER tolerance
# that accounts for real mechanical flex/backlash, not just encoder
# noise. Never issues a fresh movement command based on this check -
# only confirms-or-relies-on-the-overall-timeout-above.
IK_SETTLE_WAIT_SECONDS  = 0.5
IK_SETTLE_TOLERANCE     = 0.3
# Catalogued targets are real values already proven reachable (captured
# live via calibrate_board.py), not a solver's estimate - so they can be
# held to a much tighter arrival/settle check than a live-solved target.
# 2026-08-17: found live that catalogue replays approaching from a
# different direction than the original capture landed up to half a
# square off, worse (and higher above the board) approaching from the
# right - the shared 0.08/0.3 IK tolerances were letting the move stop
# well short. Reusing GRIP_ARRIVAL_TOLERANCE/GRIP_SETTLE_TOLERANCE's
# already-proven values rather than inventing new ones.
CATALOGUE_ARRIVAL_TOLERANCE = 0.02
CATALOGUE_SETTLE_TOLERANCE  = 0.05
# First-version target is hardcoded - no camera/sensor integration
# exists yet to provide a real one. Same target already validated
# end-to-end today (manual math + a real robot test).
IK_TEST_TARGET_XYZ      = (0.4, 0.2, 0.3)
# External chess-controller integration (2026-08-15): if this file
# exists and parses, its x/y/z override IK_TEST_TARGET_XYZ for the next
# Triangle-triggered IK request - written by set_ik_target.py, a
# separate tool outside the live control path. Falls back to the
# hardcoded default on ANY problem (missing file, bad JSON, missing
# keys) - a stale/bad file can never crash this loop or silently do
# something unexpected, it just behaves exactly as before this change.
IK_TARGET_FILE = os.path.expanduser('~/dARM/odrive_tools/ik_target.json')

def read_ik_target():
    try:
        with open(IK_TARGET_FILE) as f:
            data = json.load(f)
        grip = None
        if 'grip' in data:
            try:
                grip = float(data['grip'])
            except Exception:
                grip = None
        return (float(data['x']), float(data['y']), float(data['z']), grip)
    except Exception:
        return (*IK_TEST_TARGET_XYZ, None)


# Real captured powered positions (2026-08-17) - see calibrate_board.py.
# For a catalogued square, Triangle drives DIRECTLY to the exact raw
# joint values that were actually captured getting there live, no
# live-solve at all - skips the whole real-world accuracy problem for
# these specific squares entirely, since it's not predicting/computing
# anything, just replaying a real measured result. Falls back to the
# normal live-solve path (unchanged) for any square not yet catalogued.
CATALOGUE_FILE = os.path.expanduser('~/dARM/odrive_tools/chess_catalogue.json')
CATALOGUE_REPLAY_LOG_FILE = os.path.expanduser('~/dARM/odrive_tools/chess_catalogue_replay_log.jsonl')


def read_catalogue_target():
    try:
        with open(IK_TARGET_FILE) as f:
            data = json.load(f)
        sq = str(data.get('square', '')).strip().lower()
        if not sq:
            return None
        with open(CATALOGUE_FILE) as f:
            catalogue = json.load(f)
        entry = catalogue['squares'][sq]['raw_nodes']
        return {0: entry['node0'], 1: entry['node1'], 3: entry['node3'],
                4: entry['node4'], 5: entry['node5'], 6: entry['node6']}
    except Exception:
        return None

# --- Pick/Place buttons (2026-08-17, corrected design) ---
# Square and Cross (X) - empirically confirmed via button_finder.py on
# 2026-08-17, NOT the standard-ordering-expected indices (same caution
# as Triangle above - this DualSense/SDL setup has repeatedly proven not
# to follow standard ordering, see the SDL#8754 issue referenced in the
# d-pad comment elsewhere in this file).
PICK_BUTTON_INDEX  = 3  # Square
PLACE_BUTTON_INDEX = 0  # Cross (X)
PICK_PLACE_HOLD_SECONDS = 0.5
# Reuse the same proven tolerances/timings as IK-mode rather than
# inventing new ones.
SEQUENCE_ARRIVAL_TOLERANCE  = IK_ARRIVAL_TOLERANCE
SEQUENCE_SETTLE_WAIT_SECONDS = IK_SETTLE_WAIT_SECONDS
SEQUENCE_SETTLE_TOLERANCE   = IK_SETTLE_TOLERANCE
SEQUENCE_WP_TIMEOUT_SECONDS = 30.0  # per-waypoint backstop, not per-sequence.
                                     # Widened 2026-08-17 from 15.0 - real evidence the old
                                     # value was cutting the grip step off before it finished:
                                     # closing near TRIGGER_MIN is deliberately slow (the same
                                     # decel-zone safety taper used everywhere else), and a firm
                                     # grip needs more time than 15s to actually get there.
# Gripper needs a much tighter arrival check than the arm joints - real
# bug found 2026-08-17 live: the shared 0.08 tolerance let the grip step
# declare "arrived" as soon as it got within 0.08 of the -0.85 target
# (i.e. anywhere past -0.77), which is nowhere near firm enough to
# actually hold a piece - confirmed live ("didn't quite pick it up").
GRIP_ARRIVAL_TOLERANCE = 0.02
GRIP_SETTLE_TOLERANCE  = 0.05

# Real design intent (corrected 2026-08-17 after first version wrongly
# jumped to an absolute pre-solved square position and produced a big
# unexpected whole-arm sweep): Triangle already gets the arm TO a target
# via a live solve - Square/Cross do NOT repeat that. They perform a
# standard, repeatable LOCAL descend/grip/lift starting from wherever
# the arm already is the moment the button is pressed. "How far down is
# down" can't be a fixed raw-joint number (needs coordinated multi-joint
# movement that depends on the arm's current pose), so this fixed delta
# was derived ONCE (not solved live) by averaging the real hover->descend
# offset across several known-good, low-error solved squares near the
# board's working area (d2/d4/e4/d3/e3/c3/f3) - see session notes. This
# is an approximation, not exact for every possible arm pose, but the
# spread across those squares was small, so it should be close enough
# across the actual working region above the board.
PICK_PLACE_DESCEND_DELTA = {
    0: 0.0376, 1: -0.0475, 3: 0.0, 4: -0.1245, 5: 0.2422, 6: -0.2421,
}
PICK_PLACE_GRIP_DEFAULT = -0.85  # tightened 2026-08-17 after live test - old -0.565 (chess_move_
                                  # sequencer.py's default) was too wide to actually hold the piece,
                                  # confirmed live ("would have worked if it closed farther").
                                  # Close to TRIGGER_MIN (-0.9037, the real full-close pinion limit)
                                  # with a small safety margin.
PICK_PLACE_RELEASE_VALUE = -0.65  # tightened 2026-08-17 - the new compliant chess_spring fingers
                                  # don't need to go to GRIPPER_RELEASE_OPEN (0.0, fully open) to
                                  # actually let go of a piece, per live confirmation.


def read_pick_place_grip():
    """Reads the closed-grip value from ik_target.json's 'grip' field (the
    SAME file Triangle/set_ik_target.py/chess_move_sequencer.py already
    write) - falls back to PICK_PLACE_GRIP_DEFAULT on any problem, same
    defensive pattern as read_ik_target()."""
    try:
        with open(IK_TARGET_FILE) as f:
            data = json.load(f)
        if data.get('grip') is not None:
            return float(data['grip'])
    except Exception:
        pass
    return PICK_PLACE_GRIP_DEFAULT


def current_raw_nodes(joint_positions, shoulder_ctrl, wrist_ctrl):
    """Snapshots the arm's actual current commanded position as a
    raw-node dict, in the same shape/units as the pre-solved lookup
    table entries - this is the reference 'hover' point pick/place
    build off of, captured fresh at the moment the button is pressed."""
    bend = wrist_ctrl.bend_pos if wrist_ctrl else 0.0
    rotate = wrist_ctrl.rotate_pos if wrist_ctrl else 0.0
    return {
        0: joint_positions[0],
        1: shoulder_ctrl.value if shoulder_ctrl else 0.0,
        3: joint_positions[3],
        4: joint_positions[4],
        5: bend + rotate,   # inverse of bend=(n5-n6)/2, rotate=(n5+n6)/2
        6: rotate - bend,
    }


def build_pick_waypoints(current_raw, grip_closed):
    """Standard repeatable pick pattern (hover -> descend -> grip -> lift),
    RELATIVE to wherever the arm currently is - hover IS the current
    position, descend is current + PICK_PLACE_DESCEND_DELTA."""
    hover = dict(current_raw)
    descend = {k: current_raw[k] + PICK_PLACE_DESCEND_DELTA[k] for k in current_raw}
    return [
        (hover,   PICK_PLACE_RELEASE_VALUE, "hover (current position)"),
        (descend, PICK_PLACE_RELEASE_VALUE, "descend"),
        (descend, grip_closed,              "grip"),
        (hover,   grip_closed,              "lift"),
    ]


def build_place_waypoints(current_raw, grip_closed):
    """Standard repeatable place pattern (hover -> descend -> release ->
    lift), same relative-to-current-position basis as build_pick_waypoints()."""
    hover = dict(current_raw)
    descend = {k: current_raw[k] + PICK_PLACE_DESCEND_DELTA[k] for k in current_raw}
    return [
        (hover,   grip_closed,              "hover (current position)"),
        (descend, grip_closed,              "descend"),
        (descend, PICK_PLACE_RELEASE_VALUE, "release"),
        (hover,   PICK_PLACE_RELEASE_VALUE, "lift"),
    ]

AXIS_LEFT_X       = 0  # Left stick horizontal
AXIS_LEFT_Y       = 1  # Left stick vertical
AXIS_LEFT_TRIGGER = 2  # L2 (physical left trigger)  
AXIS_RIGHT_X      = 3  # Right stick horizontal
AXIS_RIGHT_Y      = 4  # Right stick vertical
AXIS_RIGHT_TRIGGER= 5  # R2 (physical right trigger)  

UPDATE_RATE              = 30.0
MAX_DT                   = 0.1  # seconds - caps any single frame's
                                  # position delta regardless of stalls
                                  # (GC pause, thread contention, slow
                                  # CAN call); ~3x the nominal 1/30s frame
DEAD_ZONE                = 0.25
VELOCITY_SCALING         = 1.35  # was 1.5 - 10% across-the-board reduction,
                                        # structure was flexing noticeably at the
                                        # old speed
SHOULDER_VELOCITY_SCALING = 1.2825  # 2026-08-17: shoulder (nodes 1,2) only, a
                                     # further 5% below the shared VELOCITY_SCALING -
                                     # felt violent at full extension. Base rotation
                                     # (node0) intentionally unaffected - still uses
                                     # VELOCITY_SCALING directly.
FOREARM_VELOCITY_SCALING = 1.8   # was 2.0 - same 10% reduction
GRIPPER_SCALING          = 0.75  # 2026-08-17: was 0.5, raised for a faster manual grip

# ------------------------------------------------------------------------------
# 2) Joint Range Definitions
#    Node0     => Joint 0
#    Node(1,2) => Joint 1 (Shoulder)
#    Node3     => Joint 2
#    Node4     => Joint 3
#    Node(5,6) => Wrist (bend + rotate)
#    Node(7)   => Gripper squeeze/release
# ------------------------------------------------------------------------------
JOINT0_MIN, JOINT0_MAX = -8.22,  7.91
JOINT1_MIN, JOINT1_MAX = -5.5,  0.0  # updated 2026-08-06 - measured real weight-contact point at -5.66, 0.16 margin
JOINT2_MIN, JOINT2_MAX = -8.85,  12.58  # Elbow roll - MIN updated 2026-08-06 (new measured wire limit -8.99, was -10.14 on 2026-07-31 - something physically changed since then, 0.14 margin). MAX still from 2026-07-31 (wire limit 12.72, 0.14 margin).
JOINT3_MIN, JOINT3_MAX =  0.0,   6.05  # MAX updated 2026-08-15 - real measured hard stop ~6.12-6.15 raw (two independent disarmed hand-tested marks), 6.05 leaves a real margin back from it. MIN unchanged - confirmed 2026-08-15 as a genuine hard stop already at its true limit, no room to extend.

BEND_MIN,   BEND_MAX     =  -3.7113,  3.7113 # Wrist - widened 2026-08-17 from +/-90deg to +/-108deg
                             # (3.7113 raw = 108deg at the 29.1 deg/raw scale). Reason: real
                             # touch-off at a8 hit the old +/-90deg limit before the fingers were
                             # perpendicular to the board - needed more bend range for that reach.
                             # Checked against real contact data first (live raw readout showing
                             # bend~130deg + rotate~41deg together at first contact, matching the
                             # documented CAD +/-110deg figure) - 108deg stays a couple degrees under
                             # that, so this alone should not reach contact. IMPORTANT: contact was
                             # only observed with bend AND rotate pushed together (matches the known
                             # swashplate-coupled wrist mechanism, see CLAUDE.md Known open issues
                             # section) - this widening does not by itself guarantee safety at high
                             # bend combined with high rotate simultaneously, that combined boundary
                             # is still uncharacterized. ROTATE_MIN/MAX unchanged by this edit.
ROTATE_MIN, ROTATE_MAX   = -14.0, 6.5 # Wrist - MIN widened 2026-08-07: real motor-power probe at rest (bend=0) reached -14.7 raw (node5/6) with zero real current draw (ibus flat) - DrJones stopped there deliberately (harness visual check, well short of any resistance, exceeds the ~360deg-total design target), not at a found strain limit. -14.0 keeps a real margin back from the reached point. MAX note (tilt-side asymmetry, needs combined boundary) still applies.
TRIGGER_MIN, TRIGGER_MAX = -0.9037, 0.032  # MIN measured 2026-08-06 (real full-close pinion limit, no margin - wants full closure); MAX also measured 2026-08-06
                                       # frame (post 2026-07-30 recalibration).
                                       # 0.0 = fully open, confirmed stable and
                                       # repeatable across a power cycle. -0.7
                                       # is a conservative pull-back from the
                                       # ~-0.91 to -0.94 hand-closed position -
                                       # NOT a confirmed true hard stop. Proper
                                       # torque-based limit-finding is separate
                                       # future work.

# ------------------------------------------------------------------------
# Named positions (####_pos convention). rest_pos is the folded stow
# pose the arm physically sits in when powered on/off (target 0.0 for
# every node, including the gripper - now that its encoder mount is
# tightened and recalibrated, 0.0 is a stable, repeatable reference
# for it too, same as every other joint).
# safe_up_pos is a known-safe vertical/untwisted waypoint used before
# ever folding down to rest_pos, so an automated return never has to
# guess a path through whatever pose the arm was left in - measured
# 2026-07-30 by manually posing the arm (unarmed) and reading live
# pos_estimate for each node.
# ------------------------------------------------------------------------
SAFE_UP_SHOULDER_VALUE = -2.834   # ShoulderController.value (node1=+value, node2=-value)
SAFE_UP_ELBOW_POS      = 3.029    # node 4
SAFE_UP_WRIST_BEND     = -0.0353  # WriteController.bend_pos; rotate_pos is left
                                    # alone at this stage and zeroed later in the
                                    # axial un-spin stage
# Catalogue staging waypoint (2026-08-17): always routing through one
# fixed, known pose before the final approach fixes direction-dependent
# backlash (approaching a catalogued square from a different side than
# it was captured from landed up to half a square off) - the final
# short leg into every catalogued target now always arrives from the
# same direction, regardless of where the arm started. Confirmed
# working live with safe_up_pos as the staging point (a7, d4 both
# landed correctly). Replaced here with a purpose-captured pose
# ("stage pos chess 2" in Carla's recording tool, 2026-08-17) - less
# looming than safe_up_pos's full vertical extension, wrist picked
# nearly straight rather than sharply folded specifically so it won't
# hit the board/pieces if it drifts, while still clearing piece height
# on the way in (manually posed and captured live, same method as
# safe_up_pos originally was - not derived through any FK chain).
CATALOGUE_STAGING_TARGET = {
    0: -3.128562, 1: -4.275810, 3: 0.0,
    4: 5.663924, 5: 0.421712, 6: -0.405434,
}
GRIPPER_RELEASE_OPEN   = 0.0       # Fully open in the NEW reference frame
                                    # (2026-07-30, after the encoder mount was
                                    # fixed and recalibrated). The old value of
                                    # 0.65 was relative to the pre-recalibration
                                    # frame and no longer corresponds to
                                    # anything physically meaningful.
REST_POS_TOLERANCE     = 0.1      # radians - how close counts as "at rest_pos"
                                    # for the arming gate (proxy check only -
                                    # pos_estimate re-zeros at boot regardless
                                    # of true physical position, so this can't
                                    # replace physically verifying the arm is
                                    # at the marked resting pose)

# Absolute ceiling for trap_traj vel/accel/decel during the safe-return
# sequence - used INSTEAD of half the node's configured limit whenever
# half the configured limit would exceed this. Halving alone is not a
# safe bound: if the configured limit is already fast, half of it is
# still fast (observed: shoulder covered the full 2.834 rad rest->safe_up
# swing in ~1s with only halving applied). These are a conservative
# starting guess, NOT a measured safe speed for this hardware - verify
# the first post-patch run with a hand on the power switch and adjust.
SAFE_RETURN_VEL_LIMIT   = 0.4
SAFE_RETURN_ACCEL_LIMIT = 0.4
SAFE_RETURN_DECEL_LIMIT = 0.4

# Node5/6 raw safety envelope, measured empirically from a combined
# tilt+rotation test, plus a large safety margin. This is the
# authoritative clamp applied in WriteController.apply() - BEND/ROTATE
# above are only soft bounds on the internal accumulator and are NOT
# sufficient on their own to guarantee this range.
MOTOR5_MIN, MOTOR5_MAX = -28.41, 6.8  # MAX raised again 2026-08-07: second motor-power probe same session reached 6.96 (rest, pure rotate) with zero strain - 6.8 keeps margin back from the reached point.
MOTOR6_MIN, MOTOR6_MAX = -27.36, 6.8  # MAX raised again 2026-08-07: same rest/pure-rotate validated stop as MOTOR5_MAX (node5=node6 at bend=0), reached 6.96

# Soft-limit deceleration: commanded speed scales down within this
# distance (same units as the joint ranges above) of a min/max limit.
DECEL_ZONE         = 1.0
DECEL_ZONE_WRIST_BEND   = 0.45  # 2026-08-16: shrunk (not removed) per explicit request - keeps a real, if tighter, glide-to-stop instead of zeroing it out entirely. Roughly half the previous 0.93 (15% of span). Global ik_speed cap + the separate hard Wrist Envelope position clamp (bend_min_dyn/bend_max_dyn) both still fully in effect regardless.
DECEL_ZONE_WRIST_ROTATE = 1.5   # 2026-08-16: same reasoning as bend above. Roughly half the previous 3.08 (15% of span).
MIN_WRIST_IK_STEP = 0.01  # 2026-08-16: real live-observed bug - on an IK-mode retry with a small remaining diff, `step = diff * ik_speed * dt` shrinks proportionally with no floor, and got small enough that the wrist barely moved at all (suspected: too small to reliably overcome real motor cogging/static friction). This floors the wrist's commanded step magnitude (never the direction/sign) so a retry always produces a command big enough to actually move the motor. Deliberately kept smaller than IK_ARRIVAL_TOLERANCE (0.08) to avoid overshoot. Wrist-only - node0/3/4's step formulas are untouched.
DECEL_ZONE_GRIPPER = 0.08  # 2026-08-17: was 0.2, compressed - more of the range at full speed

def taper_increment(current_val, increment, min_val, max_val, decel_zone):
    """
    Scales down `increment` as `current_val` approaches min_val/max_val,
    so movement eases into a limit instead of arriving at full speed.
    """
    if increment > 0:
        remaining = max_val - current_val
    elif increment < 0:
        remaining = current_val - min_val
    else:
        return increment
    if decel_zone > 0 and remaining < decel_zone:
        scale = max(0.05, remaining / decel_zone)
        increment *= scale
    return increment

def ik_request_worker(x, y, z, result_holder):
    """
    Runs the network call to redPi's IK solver service on its OWN
    thread, so it can never block the main control loop - in
    particular, never blocks PS-hold responsiveness. Same reasoning as
    ArmingSequence already running on its own thread for its own
    multi-step operation. Writes into result_holder (a plain dict) so
    the main loop can poll it without blocking - never touches CAN,
    the bus, or anything safety-related itself.
    """
    try:
        url = f"{IK_SERVICE_URL}?x={x}&y={y}&z={z}"
        with urllib.request.urlopen(url, timeout=2.0) as resp:
            result_holder['solve'] = json.loads(resp.read())
    except Exception as e:
        result_holder['error'] = str(e)
    result_holder['done'] = True

stop_event = threading.Event()

# Joystick states for UI display
joystick_states = {
    "LB": False,
    "Circle": False,
    "Triangle": False,
    "PS": False,
    "status": "",
    "axes": {
        AXIS_LEFT_X:  0.0,
        AXIS_LEFT_Y:  0.0,
        AXIS_RIGHT_X: 0.0,
        AXIS_RIGHT_Y: 0.0,
        AXIS_LEFT_TRIGGER:  0.0,  # <--- For debugging display
        AXIS_RIGHT_TRIGGER: 0.0   # <--- For debugging display
    }
}

# ------------------------------------------------------------------------------
# 3) Helper Functions
# ------------------------------------------------------------------------------
def apply_dead_zone(value):
    """
    Below DEAD_ZONE, returns 0. Above it, rescales the remaining travel
    back onto the full -1..1 range, so the first bit of stick motion
    past the dead zone doesn't jump straight to a value of ~DEAD_ZONE -
    previously there was no fine/slow-creep range at all, just a jump
    from 0 to a moderate speed the instant the stick left center.

    Squared (expo) response on top of the rescale, 2026-08-11: small
    deflections past the dead zone command proportionally less speed
    than the previous 1:1 linear mapping did (e.g. 30% stick -> ~9%
    speed instead of 30%), giving finer control for slow/precise moves
    - full deflection still gives full speed either way. Chosen over
    shrinking DEAD_ZONE itself, since a smaller dead zone would trade
    away real noise margin against stall-amplified stick residuals
    (see the 2026-08-10/11 frame-stall "tell" investigation) - this
    curve gets the same finer-control result without that tradeoff.
    """
    if abs(value) < DEAD_ZONE:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    rescaled = (abs(value) - DEAD_ZONE) / (1.0 - DEAD_ZONE)
    return sign * (rescaled ** 2)

TRIGGER_DEAD_ZONE = 0.05  # small - just enough to ignore rest-position noise

def apply_trigger_dead_zone(value):
    """
    Triggers rest at -1.0 (released) and move toward +1.0 (fully
    pressed) - not at 0 like the sticks, so apply_dead_zone() (built
    for sticks) doesn't fit them: it was silently requiring a trigger
    to be pressed more than halfway before producing any output at
    all. This normalizes -1..1 to 0..1 (0 = released, 1 = fully
    pressed) first, then applies a small dead zone near the released
    end only, so response starts almost immediately off rest.
    """
    normalized = (value + 1.0) / 2.0
    if normalized < TRIGGER_DEAD_ZONE:
        return 0.0
    return (normalized - TRIGGER_DEAD_ZONE) / (1.0 - TRIGGER_DEAD_ZONE)

def signal_handler(sig, frame):
    raise KeyboardInterrupt

def handle_input(key, loop, node_ids, bus, joint_positions):
    if key == 'esc':
        stop_event.set()
        raise urwid.ExitMainLoop()

def clean_shutdown(node_ids, bus, joint_positions, endpoints, safe_return, arming_seq, lockout_event, shoulder_ctrl, wrist_ctrl):
    """
    Exit path. Never runs a second, independent safe-return move
    concurrently with one that's already active - that race (an
    orphaned PStap sequence thread still running while shutdown fired
    its own move) is what sheared the wrist assembly. Instead:
      1. If arming is still in progress on its own thread, abort it and
         wait briefly - no point completing an arm attempt on the way
         out the door.
      2. If the sequence is running/paused, wait for it to finish.
      3. If it doesn't finish in time, stop here - Estop and manual
         placement at rest_pos is required, not a second move.
      4. If nothing was ever armed, there's nothing to do.
      5. If something is armed and the sequence never ran (e.g. ESC
         pressed without ever tapping PStap), fall back to the same
         staged sequence rather than the old naive all-at-once move.
    """
    print("\nExiting...")

    if arming_seq.is_running():
        print("[INFO] Arming still in progress - aborting it before exit...")
        arming_seq.abort()
        if arming_seq.thread is not None:
            arming_seq.thread.join(timeout=15)

    if lockout_event.is_set():
        print("[CRITICAL] Exiting while locked out (an unconfirmed disarm or an unexpected "
              "node dropout was detected and never resolved). NOT running the automated "
              "return sequence - it is not safe to assume coordinated motion is OK when we "
              "don't know why a node dropped. Whatever is still armed will hold its last "
              "position on its own until power is cut - it does not need Python running to "
              "do that. Use check_armed.py to see exactly what's armed, then resolve manually "
              "before next power-on.")
        if bus:
            bus.shutdown()
        return

    if safe_return.is_running():
        if safe_return.pause_event.is_set():
            print("[INFO] Safe-return sequence is paused - resuming it so it can finish before exiting...")
            safe_return.pause_event.clear()
        print("[INFO] Safe-return sequence still active - waiting for it to finish before exiting...")
        safe_return.join(timeout=90)
        if safe_return.is_running():
            print("[ERROR] Safe-return sequence did not finish in time. Estop and manually "
                  "place the arm at rest_pos before next power-on. NOT running a second move.")
            if bus:
                bus.shutdown()
            return

    armed_ep = endpoints['endpoints']['axis0.is_armed']
    armed_ids = [nid for nid in node_ids if read_config(bus, nid, armed_ep['id'], armed_ep['type'])]

    if not armed_ids:
        print("[INFO] No nodes armed - nothing to do.")
        if bus:
            bus.shutdown()
        return

    print(f"[INFO] Nodes still armed {armed_ids} and safe-return never ran - running it now before exit...")
    run_safe_return_sequence(bus, armed_ids, endpoints, shoulder_ctrl, wrist_ctrl,
                              joint_positions, threading.Event(), threading.Event())
    if bus:
        bus.shutdown()

def force_disarm_all(bus, node_ids, endpoints):
    """
    Immediately cuts torque on every listed node (IDLE), with no slow
    move first. Used when PS is held continuously past
    DISARM_HOLD_SECONDS, overriding the gentler stop-and-return-to-rest.
    Verifies each node actually confirms disarmed via is_armed read-back
    rather than trusting the CAN send alone.
    """
    print("\n>>> FORCED DISARM - TORQUE CUT IMMEDIATELY <<<\n")
    failed = []
    for nid in node_ids:
        if not disarm_verified(bus, nid, endpoints):
            failed.append(nid)
    if failed:
        print(f"[WARNING] Nodes NOT confirmed disarmed: {failed} - check manually (check_armed.py).")

# ------------------------------------------------------------------------------
# 4) Shoulder & Gripper Classes
# ------------------------------------------------------------------------------
def read_position(bus, node_id, endpoints):
    """
    Reads a node's actual current position (axis0.pos_estimate).
    """
    ep = endpoints['endpoints']['axis0.pos_estimate']
    return read_config(bus, node_id, ep['id'], ep['type'])

# 2026-08-01: settle_timeout changed from 0.3 to 0.22 (distinct from
# disarm_verified/write_verified below) so each function's worst-case
# blocking time is numerically distinguishable in "Frame stall detected"
# warnings - lets us tell which retry loop actually caused a given stall.
def arm_node_verified(bus, node_id, endpoints, retries=5, settle_timeout=0.22,
                       confirm_window=0.2, confirm_interval=0.05):
    """
    Requests CLOSED_LOOP_CONTROL and confirms via axis0.is_armed that the
    node actually latched into it, instead of trusting that the CAN send
    alone succeeded. A single True read isn't enough on its own - once
    is_armed first reads True, it has to stay True for confirm_window
    seconds (checked every confirm_interval) before this counts as a
    real success. Catches a node that briefly reports armed and then
    drops back out during some internal settling process, rather than
    only finding out later once motion is already underway.
    """
    armed_ep  = endpoints['endpoints']['axis0.is_armed']
    disarm_ep = endpoints['endpoints']['axis0.disarm_reason']
    errors_ep = endpoints['endpoints']['axis0.active_errors']
    for attempt in range(retries):
        set_closed_loop_control(bus, node_id)
        start = time.time()
        while time.time() - start < settle_timeout:
            armed = read_config(bus, node_id, armed_ep['id'], armed_ep['type'])
            if armed:
                confirm_start = time.time()
                stayed_armed = True
                while time.time() - confirm_start < confirm_window:
                    time.sleep(confirm_interval)
                    still = read_config(bus, node_id, armed_ep['id'], armed_ep['type'])
                    if not still:
                        stayed_armed = False
                        break
                if stayed_armed:
                    return True
                print(f"[WARN] Node {node_id} reported armed but did not stay armed - retrying.")
                break
            time.sleep(0.05)
    disarm_reason = read_config(bus, node_id, disarm_ep['id'], disarm_ep['type'])
    active_errors = read_config(bus, node_id, errors_ep['id'], errors_ep['type'])
    print(f"[ERROR] Node {node_id} did not confirm armed after {retries} attempts "
          f"(disarm_reason={disarm_reason}, active_errors={active_errors})")
    # Giving up here does NOT guarantee the node is actually disarmed - a
    # "did not stay armed" retry above can be triggered by a flaky read
    # rather than a real drop, meaning the node could still be genuinely
    # armed even though every retry looked like a failure. Force a
    # verified disarm before returning, so this function's contract
    # holds: returning False always means the node is confirmed safe,
    # never just "we're not sure." (2026-07-31: node 7 was left
    # physically armed after this function gave up, because nothing
    # here ever actually asked it to disarm.)
    if not disarm_verified(bus, node_id, endpoints):
        print(f"[CRITICAL] Node {node_id} could not be confirmed disarmed after "
              f"giving up on arming - DO NOT operate the arm. Check manually "
              f"(check_armed.py).")
    return False

# 2026-08-01: settle_timeout changed from 0.3 to 0.34 (distinct from
# arm_node_verified/write_verified) - same reasoning, see arm_node_verified.
def disarm_verified(bus, node_id, endpoints, retries=5, settle_timeout=0.34):
    """
    Requests IDLE and confirms via axis0.is_armed that the node actually
    latched into it, instead of trusting that the CAN send alone
    succeeded. Mirrors arm_node_verified() for the opposite transition -
    a fire-and-forget IDLE command can silently fail to take effect on a
    lossy CAN bus, leaving a node armed with no indication of failure.
    """
    armed_ep = endpoints['endpoints']['axis0.is_armed']
    for attempt in range(retries):
        set_idle_mode(bus, node_id)
        start = time.time()
        while time.time() - start < settle_timeout:
            armed = read_config(bus, node_id, armed_ep['id'], armed_ep['type'])
            if armed is not None and not armed:
                return True
            time.sleep(0.05)
    print(f"[ERROR] Node {node_id} did not confirm disarmed after {retries} attempts.")
    return False

# 2026-08-01: settle_timeout changed from 0.3 to 0.46 (distinct from
# arm_node_verified/disarm_verified) - same reasoning, see arm_node_verified.
def write_verified(bus, node_id, endpoint_id, endpoint_type, value, label="",
                    retries=5, settle_timeout=0.46, tolerance=1e-3):
    """
    Writes a value and confirms via read-back that it actually landed,
    instead of trusting a single fire-and-forget CAN write. Mirrors
    disarm_verified()/arm_node_verified() - CAN writes on this bus are
    unreliable and can silently fail with no error raised. This is what
    was missing from move_to_neutral_slowly()'s halve/restore steps,
    which caused trap_traj limits to be repeatedly halved over time.
    """
    for attempt in range(retries):
        write_config(bus, node_id, endpoint_id, endpoint_type, value)
        start = time.time()
        while time.time() - start < settle_timeout:
            readback = read_config(bus, node_id, endpoint_id, endpoint_type)
            if readback is not None and abs(readback - value) <= tolerance:
                return True
            time.sleep(0.05)
    print(f"[ERROR] Node {node_id} {label} did not confirm write of {value} after {retries} attempts.")
    return False

def wait_for_position(bus, node_ids, endpoints, targets, tolerance=0.05, timeout=5.0,
                       pause_event=None, abort_event=None):
    """
    Polls axis0.pos_estimate for each node in node_ids until all are
    within `tolerance` of their target, or until `timeout` elapses.

    If `pause_event` becomes set while waiting, every still-moving node
    is immediately re-commanded to its own CURRENT live position -
    freezing it right there, mid-move if needed - and this blocks until
    `pause_event` clears, at which point the original targets are
    re-sent and the timeout clock restarts (so a long pause doesn't
    eat into it). If `abort_event` is set (Estop fired), returns False
    immediately without sending anything further.

    Returns True if every node confirmed within tolerance, False on
    timeout or abort.
    """
    pos_ep = endpoints['endpoints']['axis0.pos_estimate']
    start = time.time()
    remaining = set(node_ids)
    was_paused = False
    while remaining:
        if abort_event is not None and abort_event.is_set():
            return False
        if pause_event is not None and pause_event.is_set():
            if not was_paused:
                for nid in remaining:
                    pos = read_config(bus, nid, pos_ep['id'], pos_ep['type'])
                    if pos is not None:
                        move_odrive_to_position(bus, nid, pos)
                was_paused = True
            time.sleep(0.05)
            continue
        if was_paused:
            for nid in remaining:
                move_odrive_to_position(bus, nid, targets[nid])
            was_paused = False
            start = time.time()
        for nid in list(remaining):
            pos = read_config(bus, nid, pos_ep['id'], pos_ep['type'])
            if pos is not None and abs(pos - targets[nid]) <= tolerance:
                remaining.discard(nid)
        if remaining and (time.time() - start > timeout):
            print(f"[WARNING] Nodes did not confirm reaching position within {timeout}s: {sorted(remaining)}")
            return False
        if remaining:
            time.sleep(0.1)
    return True


def run_safe_return_sequence(bus, node_ids, endpoints, shoulder_ctrl, wrist_ctrl,
                              joint_positions, pause_event, abort_event):
    """
    Staged, coordinated return sequence - triggered by PStap, and used
    as the exit fallback if the arm isn't already at rest_pos. Always
    passes through the same known-safe safe_up_pos waypoint (vertical,
    untwisted) before folding down to rest_pos, instead of moving every
    joint independently straight to its final target - a real incident
    showed that a direct all-at-once return from full extension has no
    awareness of the arm's actual swept path and can sweep it through
    the table, which is what sheared the wrist assembly's shaft.

    Stages:
      1. Open the gripper fully (release anything held).
      2. Shoulder -> safe_up.
      3. Elbow -> safe_up.
      4. Wrist bend -> safe_up (rotate left untouched for now).
      5. Un-spin the axial joints: base, elbow-roll, wrist rotate -> 0.
         The arm is vertical and clear at this point, so spinning these
         in place is safe regardless of order.
      6. Fold everything down to rest_pos.
      7. Disarm.

    trap_traj limits are halved for the whole sequence (verified writes,
    same pattern as before) and always restored at the end, even if the
    sequence is aborted partway - so an Estop mid-sequence can never
    leave speed limits stuck halved.

    `pause_event` is set/cleared externally (PStap) to pause/resume -
    pausing freezes whatever is currently moving immediately, it does
    not wait for the current move to finish. `abort_event` is set
    externally if Estop fires; remaining stages are skipped, but the
    speed-limit restore step still always runs.
    """
    def aborted():
        return abort_event is not None and abort_event.is_set()

    pos_ep = endpoints['endpoints']['axis0.pos_estimate']
    already_there = True
    for nid in node_ids:
        pos = read_config(bus, nid, pos_ep['id'], pos_ep['type'])
        if pos is None or abs(pos - 0.0) > REST_POS_TOLERANCE:
            already_there = False
            break
    if already_there:
        print("[SAFE_UP] Already at rest_pos (within tolerance) - skipping safe_up_pos waypoint, disarming directly.")
        failed = []
        for nid in node_ids:
            if not disarm_verified(bus, nid, endpoints):
                failed.append(nid)
        if failed:
            print(f"[WARNING] Nodes NOT confirmed disarmed: {failed} - check manually (check_armed.py).")
        else:
            print("[SAFE_UP] Sequence complete - already at rest_pos, all nodes disarmed.")
        return

    vel_ep   = endpoints['endpoints']['axis0.trap_traj.config.vel_limit']
    accel_ep = endpoints['endpoints']['axis0.trap_traj.config.accel_limit']
    decel_ep = endpoints['endpoints']['axis0.trap_traj.config.decel_limit']

    orig_values = {}
    failed_halve = []
    for nid in node_ids:
        vel   = read_config(bus, nid, vel_ep['id'], vel_ep['type'])
        accel = read_config(bus, nid, accel_ep['id'], accel_ep['type'])
        decel = read_config(bus, nid, decel_ep['id'], decel_ep['type'])
        orig_values[nid] = (vel, accel, decel)
        vel_target   = min(vel / 2, SAFE_RETURN_VEL_LIMIT) if vel is not None else None
        accel_target = min(accel / 2, SAFE_RETURN_ACCEL_LIMIT) if accel is not None else None
        decel_target = min(decel / 2, SAFE_RETURN_DECEL_LIMIT) if decel is not None else None
        if vel_target is not None and not write_verified(bus, nid, vel_ep['id'], vel_ep['type'], vel_target, label="trap_vel cap"):
            failed_halve.append((nid, "vel"))
        if accel_target is not None and not write_verified(bus, nid, accel_ep['id'], accel_ep['type'], accel_target, label="trap_accel cap"):
            failed_halve.append((nid, "accel"))
        if decel_target is not None and not write_verified(bus, nid, decel_ep['id'], decel_ep['type'], decel_target, label="trap_decel cap"):
            failed_halve.append((nid, "decel"))
    if failed_halve:
        print(f"[WARNING] Speed-limit cap not confirmed for: {failed_halve} - proceeding anyway.")

    if 7 in node_ids and not aborted():
        print("[SAFE_UP] Stage 1: opening gripper...")
        move_odrive_to_position(bus, 7, GRIPPER_RELEASE_OPEN)
        wait_for_position(bus, [7], endpoints, {7: GRIPPER_RELEASE_OPEN},
                           pause_event=pause_event, abort_event=abort_event)

    if shoulder_ctrl and (1 in node_ids) and (2 in node_ids) and not aborted():
        print("[SAFE_UP] Stage 2: shoulder to vertical...")
        shoulder_ctrl.value = SAFE_UP_SHOULDER_VALUE
        shoulder_ctrl.apply()
        wait_for_position(bus, [1, 2], endpoints,
                           {1: SAFE_UP_SHOULDER_VALUE, 2: -SAFE_UP_SHOULDER_VALUE},
                           pause_event=pause_event, abort_event=abort_event)

    if 4 in node_ids and not aborted():
        print("[SAFE_UP] Stage 3: elbow to vertical...")
        move_odrive_to_position(bus, 4, SAFE_UP_ELBOW_POS)
        wait_for_position(bus, [4], endpoints, {4: SAFE_UP_ELBOW_POS},
                           pause_event=pause_event, abort_event=abort_event)

    if wrist_ctrl and (5 in node_ids) and (6 in node_ids) and not aborted():
        print("[SAFE_UP] Stage 4: wrist bend to vertical...")
        wrist_ctrl.bend_pos = SAFE_UP_WRIST_BEND
        wrist_ctrl.apply()
        t5 = wrist_ctrl.rotate_pos + wrist_ctrl.bend_pos
        t6 = wrist_ctrl.rotate_pos - wrist_ctrl.bend_pos
        wait_for_position(bus, [5, 6], endpoints, {5: t5, 6: t6},
                           pause_event=pause_event, abort_event=abort_event)

    if not aborted():
        print("[SAFE_UP] Stage 5: un-spinning base / elbow-roll / wrist rotate...")
        axial_targets = {}
        if 0 in node_ids:
            move_odrive_to_position(bus, 0, 0.0)
            axial_targets[0] = 0.0
        if 3 in node_ids:
            move_odrive_to_position(bus, 3, 0.0)
            axial_targets[3] = 0.0
        if wrist_ctrl and (5 in node_ids) and (6 in node_ids):
            wrist_ctrl.rotate_pos = 0.0
            wrist_ctrl.apply()
            axial_targets[5] = wrist_ctrl.rotate_pos + wrist_ctrl.bend_pos
            axial_targets[6] = wrist_ctrl.rotate_pos - wrist_ctrl.bend_pos
        if axial_targets:
            axial_done = wait_for_position(bus, list(axial_targets.keys()), endpoints, axial_targets,
                                            tolerance=0.1, timeout=30.0,
                                            pause_event=pause_event, abort_event=abort_event)
            if not axial_done:
                print("[WARNING] Axial un-spin (base/elbow-roll/wrist-rotate) not "
                      "confirmed within timeout - NOT proceeding to fold-down. Nodes "
                      "still ARMED and continuing toward their axial targets. Check "
                      "manually (check_armed.py) before assuming the sequence finished.")
                return

    if not aborted():
        print("[SAFE_UP] Stage 6: folding down to rest_pos...")
        rest_targets = {nid: 0.0 for nid in node_ids}
        for nid in node_ids:
            move_odrive_to_position(bus, nid, rest_targets[nid])
        if shoulder_ctrl:
            shoulder_ctrl.value = 0.0
            shoulder_ctrl.apply()
        if wrist_ctrl:
            wrist_ctrl.bend_pos   = 0.0
            wrist_ctrl.rotate_pos = 0.0
            wrist_ctrl.apply()
        reached_rest = wait_for_position(bus, node_ids, endpoints, rest_targets,
                           tolerance=0.1, timeout=30.0,
                           pause_event=pause_event, abort_event=abort_event)
        for nid in node_ids:
            joint_positions[nid] = rest_targets[nid]

    failed_restore = []
    for nid in node_ids:
        vel, accel, decel = orig_values[nid]
        if vel is not None and not write_verified(bus, nid, vel_ep['id'], vel_ep['type'], vel, label="trap_vel restore"):
            failed_restore.append((nid, "vel"))
        if accel is not None and not write_verified(bus, nid, accel_ep['id'], accel_ep['type'], accel, label="trap_accel restore"):
            failed_restore.append((nid, "accel"))
        if decel is not None and not write_verified(bus, nid, decel_ep['id'], decel_ep['type'], decel, label="trap_decel restore"):
            failed_restore.append((nid, "decel"))
    if failed_restore:
        print(f"[WARNING] Speed limits NOT confirmed restored for: {failed_restore} - check manually (diag_node_compare2.py).")

    if aborted():
        print("[SAFE_UP] Sequence aborted (Estop) - speed limits restored, no further motion sent.")
        return

    if not reached_rest:
        print("[WARNING] Rest position not confirmed within the wait timeout - "
              "nodes are still ARMED and continuing under their own commanded "
              "position, NOT disarmed. If the arm looks stopped short of "
              "rest_pos, give it a few more seconds - it is still actively "
              "holding/moving, not free-falling. Check manually (check_armed.py) "
              "rather than assuming the sequence finished.")
        return

    print("[SAFE_UP] Stage 7: disarming...")
    failed = []
    for nid in node_ids:
        if not disarm_verified(bus, nid, endpoints):
            failed.append(nid)
    if failed:
        print(f"[WARNING] Nodes NOT confirmed disarmed: {failed} - check manually (check_armed.py).")
    else:
        print("[SAFE_UP] Sequence complete - arm at rest_pos, all nodes disarmed.")


def run_arming_sequence(bus, node_ids, endpoints, abort_event):
    """
    Validates every node, then arms them one at a time, verified - runs
    on its own thread (via ArmingSequence) so the main joystick loop is
    never blocked waiting on it and can keep checking PS-hold every
    frame no matter how long arming takes. Same all-or-nothing rule as
    before: any single validation failure, arm failure, or an abort
    (PS-hold fired mid-arming) rolls back everything armed so far and
    ends with nothing armed.

    Returns a dict:
      {"success": True,  "armed_ids": [...], "positions": {...}}
      {"success": False, "reason": "...", "still_armed": [...]}
    still_armed non-empty means the rollback disarm did NOT confirm
    clean - the caller must lock out further arming attempts.
    """
    def aborted():
        return abort_event is not None and abort_event.is_set()

    positions = {}
    failures = []
    for nid in node_ids:
        if aborted():
            return {"success": False, "reason": "aborted during validation", "still_armed": []}
        pos = None
        for attempt in range(3):
            pos = read_position(bus, nid, endpoints)
            if pos is not None:
                break
            time.sleep(0.05)
        if pos is None:
            failures.append((nid, "position read failed"))
            continue
        positions[nid] = pos
        # Node 7 (gripper) used to be exempt here since its pos_estimate
        # wasn't a stable reference across power cycles - traced to a
        # loose encoder board mount, now fixed and recalibrated
        # (2026-07-30), so it gets the same check as every other node.
        if abs(pos - 0.0) > REST_POS_TOLERANCE:
            failures.append((nid, f"not at rest_pos ({round(pos, 4)})"))

    if failures:
        print(f"[ERROR] Arming aborted - not every node is ready: {failures}. "
              f"NO nodes armed. Verify the arm is physically at the marked "
              f"resting pose and retry.")
        return {"success": False, "reason": f"validation failed: {failures}", "still_armed": []}

    newly_armed = []
    failed_nid = None
    for nid in node_ids:
        if aborted():
            failed_nid = nid
            print(f"[INFO] Arming aborted (PS-hold) before node {nid} - rolling back.")
            break
        move_odrive_to_position(bus, nid, positions[nid])
        if arm_node_verified(bus, nid, endpoints):
            newly_armed.append(nid)
            print(f"[INFO] Node {nid} confirmed armed.")
        else:
            print(f"[WARN] Node {nid} did NOT arm.")
            failed_nid = nid
            break

    if failed_nid is not None:
        print(f"[ERROR] Arming stopped at node {failed_nid} - disarming the "
              f"{len(newly_armed)} node(s) that had already armed "
              f"({newly_armed}) so nothing is left partially armed.")
        for armed_nid in newly_armed:
            if not disarm_verified(bus, armed_nid, endpoints):
                print(f"[WARNING] Node {armed_nid} did NOT confirm disarmed - "
                      f"check manually (check_armed.py).")
        armed_ep = endpoints['endpoints']['axis0.is_armed']
        still_armed = [nid for nid in node_ids
                       if read_config(bus, nid, armed_ep['id'], armed_ep['type']) is not False]
        return {"success": False, "reason": f"node {failed_nid}", "still_armed": still_armed}

    return {"success": True, "armed_ids": newly_armed, "positions": positions}


class ArmingSequence:
    """
    Runs run_arming_sequence() on its own thread - the same pattern as
    SafeReturnSequence - so arming (which can legitimately take several
    seconds if nodes are slow to confirm) never blocks the main
    joystick loop, which must always stay free to check PS-hold every
    frame. PS-hold sets abort_event, same as it does for
    SafeReturnSequence.
    """
    def __init__(self):
        self.thread = None
        self.abort_event = threading.Event()
        self.result = None

    def is_running(self):
        return self.thread is not None and self.thread.is_alive()

    def start(self, bus, node_ids, endpoints):
        self.abort_event.clear()
        self.result = None
        def _run():
            self.result = run_arming_sequence(bus, node_ids, endpoints, self.abort_event)
        self.thread = threading.Thread(target=_run, daemon=True)
        self.thread.start()

    def abort(self):
        self.abort_event.set()


class SafeReturnSequence:
    """
    Shared handle for the safe-return sequence, so both
    joystick_thread_func (which starts/pauses/resumes it via PStap) and
    main()'s shutdown path (which must wait for it rather than racing
    it) see the same running thread. This replaces the old
    return_thread local variable, which main() could never see or
    join - the exact gap that let clean_shutdown() fire a second,
    concurrent neutral-move while the first was still active, which is
    what sheared the wrist assembly.
    """
    def __init__(self):
        self.thread = None
        self.pause_event = threading.Event()
        self.abort_event = threading.Event()

    def is_running(self):
        return self.thread is not None and self.thread.is_alive()

    def start(self, bus, node_ids, endpoints, shoulder_ctrl, wrist_ctrl, joint_positions):
        self.pause_event.clear()
        self.abort_event.clear()
        def _run():
            run_safe_return_sequence(bus, node_ids, endpoints, shoulder_ctrl,
                                      wrist_ctrl, joint_positions,
                                      self.pause_event, self.abort_event)
        self.thread = threading.Thread(target=_run, daemon=True)
        self.thread.start()

    def toggle_pause(self):
        if self.pause_event.is_set():
            self.pause_event.clear()
            print("[SAFE_UP] Resumed.")
        else:
            self.pause_event.set()
            print("[SAFE_UP] Paused - motion frozen. Tap PS again to resume.")

    def abort(self):
        self.abort_event.set()

    def join(self, timeout=None):
        if self.thread is not None:
            self.thread.join(timeout=timeout)

class ShoulderController:
    """
    Node(1,2) => single shoulder joint with two motors in opposite directions.
    We track 'value' => motorA = +value, motorB = -value.
    """
    def __init__(self, bus, node_ids):
        self.bus      = bus
        self.node_ids = node_ids
        self.value    = 0.0

    def apply(self):
        nA, nB = self.node_ids
        motorA = self.value
        motorB = -self.value
        move_odrive_to_position(self.bus, nA, motorA)
        move_odrive_to_position(self.bus, nB, motorB)

class WriteController:
    """
    Node(5,6) => Wrist with bend_pos and rotate_pos
    """
    def __init__(self, bus, node_ids):
        self.bus        = bus
        self.node_ids   = node_ids  # [5,6]
        self.bend_pos   = 0.0
        self.rotate_pos = 0.0

    def clamp(self, val, min_val, max_val):
        return max(min_val, min(max_val, val))

    def apply(self):
        nA, nB = self.node_ids
        motorA = self.rotate_pos + self.bend_pos
        motorB = self.rotate_pos - self.bend_pos
        # Authoritative raw safety clamp - see MOTOR5/6_MIN/MAX above.
        motorA = self.clamp(motorA, MOTOR5_MIN, MOTOR5_MAX)
        motorB = self.clamp(motorB, MOTOR6_MIN, MOTOR6_MAX)
        move_odrive_to_position(self.bus, nA, motorA)
        move_odrive_to_position(self.bus, nB, motorB)

# ------------------------------------------------------------------------------
# 5) UI Update Thread
# ------------------------------------------------------------------------------
def update_ui_thread(bus, node_ids, endpoints, metrics_text, joystick_text, loop):
    col_widths = {}
    for metric in METRIC_ENDPOINTS:
        col_widths[metric] = max(len(metric), 4) + 3

    node_col_w = 6
    header = f"{'Node':<{node_col_w}}" + "".join(
        f"{m:<{col_widths[m]}}" for m in METRIC_ENDPOINTS
    )

    axis_names = ["LeftX", "LeftY", "RightX", "RightY", "LTrig", "RTrig"]
    axis_col_w = max(len(n) for n in axis_names) + 3
    joy_header_line = "LB".ljust(8) + "".join(x.ljust(axis_col_w) for x in axis_names)

    while not stop_event.is_set():
        # ODrive metrics
        lines = [header]
        for nid in node_ids:
            data = get_metrics(bus, nid, endpoints)
            row = f"{nid:<{node_col_w}}"
            for metric in METRIC_ENDPOINTS:
                val = data.get(metric, None)
                if isinstance(val, (int, float)):
                    sign_space = ' ' if val >= 0 else ''
                    row += f"{sign_space}{val:.2f}".ljust(col_widths[metric])
                else:
                    row += f"{'None':<{col_widths[metric]}}"
            lines.append(row)
        metrics_text.set_text("\n".join(lines))

        # Joystick line
        lb_str = "Pressed" if joystick_states["LB"] else "NotPress"
        joy_line = lb_str.ljust(8)

        # We display the 6 axes we track
        axis_values = [
            joystick_states["axes"].get(AXIS_LEFT_X, 0.0),
            joystick_states["axes"].get(AXIS_LEFT_Y, 0.0),
            joystick_states["axes"].get(AXIS_RIGHT_X, 0.0),
            joystick_states["axes"].get(AXIS_RIGHT_Y, 0.0),
            joystick_states["axes"].get(AXIS_LEFT_TRIGGER, 0.0),
            joystick_states["axes"].get(AXIS_RIGHT_TRIGGER, 0.0),
        ]
        for val in axis_values:
            joy_line += f"{val:>6.2f}".ljust(axis_col_w)

        l1_str = "HELD" if joystick_states["LB"] else "-"
        circle_str = "HELD" if joystick_states["Circle"] else "-"
        ps_str = "HELD" if joystick_states["PS"] else "-"
        triangle_str = "HELD" if joystick_states["Triangle"] else "-"
        gesture_line = f"L1:{l1_str}  Circle:{circle_str}  PS:{ps_str}  Triangle(IK):{triangle_str}"
        status_line = f"Status: {joystick_states['status']}"
        joystick_text.set_text(joy_header_line + "\n" + joy_line + "\n\n" + gesture_line + "\n" + status_line)

        # 0.3s (~3Hz) instead of 0.1s (10Hz) - this loop is pure display,
        # calling get_metrics() for all 8 nodes each pass (13 individual
        # CAN reads per node = ~1,040 round-trips/sec at 10Hz). Slowing it
        # cuts that standing CAN load roughly 3x with no functional risk -
        # this thread never sends anything that moves, arms, or disarms
        # anything, so it can't affect safety-relevant behavior.
        time.sleep(0.3)
        try:
            loop.draw_screen()
        except (urwid.ExitMainLoop, RuntimeError):
            break

# ------------------------------------------------------------------------------
# 6) Main Joystick Logic Thread
# ------------------------------------------------------------------------------
def joystick_thread_func(
    bus, node_ids, joint_positions,
    shoulder_ctrl, wrist_ctrl, endpoints, safe_return, arming_seq, lockout_event,
    update_rate = UPDATE_RATE
):
    """
    - If LB pressed         => normal mode
    - If LB and RB pressed  => wrist mode
    """
    clock = pygame.time.Clock()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    connected = True

    arm_hold_start        = None
    arm_triggered_this_hold = False
    awaiting_l1_reset     = False
    l1_released_since_arm = False
    wrist_mode_prev        = False
    awaiting_stick_neutral = False
    pick_hold_start        = None
    place_hold_start       = None
    sequence_active        = False
    sequence_kind          = None
    sequence_square        = None
    sequence_waypoints     = None
    sequence_index         = 0
    sequence_start_time    = None
    sequence_wp_reached_time = None
    sequence_button_released_since_start = False
    ps_prev               = False
    ps_press_time         = None
    all_armed             = False
    was_seq_running        = False
    was_arming_running     = False
    disarm_check_counter  = 0
    watchdog_pending_dropped = set()

    ik_hold_start   = None
    ik_mode_active  = False
    ik_mode_is_catalogued = False  # True only while driving to a catalogued
                                    # (real captured) target - see
                                    # CATALOGUE_ARRIVAL_TOLERANCE below.
    ik_catalogue_staging = False   # True only during the first (staging) leg
                                    # of a catalogued move - see
                                    # CATALOGUE_STAGING_TARGET above.
    ik_catalogue_final_target = None  # holds the real catalogued target while
                                       # ik_targets points at the staging pose
    ik_catalogue_square_name = None   # which square, for replay-accuracy logging only
    ik_catalogue_settle_snapshot = {}  # {node_id: (target, live)}, captured during
                                        # the settle-check below, logged once settled
    ik_targets      = None
    ik_pending_grip = None  # optional node7 target, carried from trigger to ik_targets construction
    ik_triangle_released_since_start = False
    ik_start_time             = None  # overall timeout clock, set at trigger
    ik_commanded_reached_time = None  # set once the commanded trajectory first reaches target
    ik_request_thread = None  # holds the in-flight background request thread, if any
    ik_request_result = {}    # written by ik_request_worker, polled here - never blocks
    ik_last_debug_print = 0.0  # throttles the diagnostic print below to ~2x/sec

    while not stop_event.is_set():
        dt = clock.tick(update_rate) / 1000.0
        if dt > MAX_DT:
            print(f"[WARNING] Frame stall detected: dt={dt:.3f}s clamped to {MAX_DT}s "
                  f"- one or more joints may have been held at their last commanded position.")
            dt = MAX_DT

        # Watchdog: detect controller disconnect via the actual SDL event
        # (more reliable than hoping a stale read errors out on its own).
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEREMOVED:
                if connected:
                    print("\n[SAFETY] Controller disconnected! Freezing all motion - no further commands will be sent until it reconnects.\n")
                connected = False
            elif event.type == pygame.JOYDEVICEADDED:
                try:
                    joystick = pygame.joystick.Joystick(0)
                    joystick.init()
                    if not connected:
                        print("\n[INFO] Controller reconnected. Resuming control.\n")
                    connected = True
                except pygame.error:
                    connected = False

        if not connected:
            # No new commands sent at all while disconnected - ODrives hold
            # their last commanded position in closed loop and do not move.
            time.sleep(0.05)
            continue

        try:
            lb = joystick.get_button(DEAD_MAN_BUTTON_INDEX)
            rb = joystick.get_button(MODE_TOGGLE_BUTTON_INDEX)
            circle = joystick.get_button(ARM_BUTTON_INDEX)
            ps = joystick.get_button(PS_BUTTON_INDEX)
            triangle = joystick.get_button(IK_TRIGGER_BUTTON_INDEX)
            pick_btn = joystick.get_button(PICK_BUTTON_INDEX)
            place_btn = joystick.get_button(PLACE_BUTTON_INDEX)
        except pygame.error:
            if connected:
                print("\n[SAFETY] Lost contact with controller! Freezing all motion.\n")
            connected = False
            time.sleep(0.05)
            continue

        joystick_states["LB"] = bool(lb)
        joystick_states["Circle"] = bool(circle)
        joystick_states["Triangle"] = bool(triangle)
        joystick_states["PS"] = bool(ps)

        # Left stick: X (horizontal) => rotate, Y (vertical) => bend
        raw_bend   = apply_dead_zone(joystick.get_axis(AXIS_LEFT_Y))
        raw_rotate = apply_dead_zone(joystick.get_axis(AXIS_LEFT_X))

        # Right stick
        rx = apply_dead_zone(joystick.get_axis(AXIS_RIGHT_X))
        ry = apply_dead_zone(joystick.get_axis(AXIS_RIGHT_Y))

        # New triggers
        raw_lt = apply_trigger_dead_zone(joystick.get_axis(AXIS_LEFT_TRIGGER))
        raw_rt = apply_trigger_dead_zone(joystick.get_axis(AXIS_RIGHT_TRIGGER))

        joystick_states["axes"][AXIS_LEFT_X]        = raw_rotate
        joystick_states["axes"][AXIS_LEFT_Y]        = raw_bend
        joystick_states["axes"][AXIS_RIGHT_X]       = rx
        joystick_states["axes"][AXIS_RIGHT_Y]       = ry
        joystick_states["axes"][AXIS_LEFT_TRIGGER]  = raw_lt
        joystick_states["axes"][AXIS_RIGHT_TRIGGER] = raw_rt

        # --- Arm gesture: L1 + Circle held 1.5s. Arming itself
        # runs on its own thread (ArmingSequence) so this loop is never
        # blocked waiting on it and keeps checking PS-hold every frame no
        # matter how long arming takes.
        if lb and circle and not lockout_event.is_set():
            if arm_hold_start is None:
                arm_hold_start = time.time()
            elif (not arm_triggered_this_hold) and (time.time() - arm_hold_start) >= ARM_HOLD_SECONDS:
                if not arming_seq.is_running():
                    print("\n[INFO] Arm gesture held - arming all nodes in the background "
                          "(validate, then arm one at a time, verified)...\n")
                    joystick_states["status"] = "ARMING: validating and arming all nodes..."
                    arming_seq.start(bus, node_ids, endpoints)
                arm_triggered_this_hold = True
        else:
            arm_hold_start = None
            arm_triggered_this_hold = False

        # --- Pick up the arming thread's result once it finishes. This
        # runs every frame regardless of gesture state, since arming
        # keeps going in the background even if you release the gesture
        # right after triggering it.
        if was_arming_running and not arming_seq.is_running():
            result = arming_seq.result or {"success": False, "reason": "no result", "still_armed": []}
            if result["success"]:
                armed_ids = result["armed_ids"]
                positions = result["positions"]
                for nid in armed_ids:
                    joint_positions[nid] = positions[nid]
                if shoulder_ctrl and (1 in armed_ids) and (2 in armed_ids):
                    shoulder_ctrl.value = (joint_positions[1] - joint_positions[2]) / 2.0
                if wrist_ctrl and (5 in armed_ids) and (6 in armed_ids):
                    wrist_ctrl.rotate_pos = (joint_positions[5] + joint_positions[6]) / 2.0
                    wrist_ctrl.bend_pos   = (joint_positions[5] - joint_positions[6]) / 2.0
                # TEMPORARILY DISABLED: node 7 was faulting (disarm_reason=2048)
                # partway through this auto-open move on nearly every arm
                # attempt. Only ever a visual "armed and ready" convenience
                # signal, not a safety requirement - re-enable once the
                # underlying fault is root-caused.
                # if 7 in armed_ids:
                #     move_odrive_to_position(bus, 7, GRIPPER_RELEASE_OPEN)
                #     joint_positions[7] = GRIPPER_RELEASE_OPEN
                print(f"[INFO] All nodes armed: {sorted(armed_ids)}\n")
                print("[SAFETY] Release and re-press L1 before any motion will be accepted.\n")
                joystick_states["status"] = f"ARMED: {sorted(armed_ids)} -- release & re-press L1 to move"
                all_armed = True
                awaiting_l1_reset = True
                l1_released_since_arm = False
            else:
                still_armed = result.get("still_armed", [])
                if still_armed:
                    lockout_event.set()
                    print(f"\n[CRITICAL] Arming aborted, and node(s) {still_armed} did NOT "
                          f"confirm disarmed. DO NOT operate the arm. Run check_armed.py and "
                          f"resolve this manually, then restart gamecontroller.py - the arm "
                          f"gesture is locked out for the rest of this session.\n")
                    joystick_states["status"] = (
                        f"!!! DISARM NOT CONFIRMED: {still_armed} - RUN check_armed.py - "
                        f"ARM GESTURE LOCKED OUT, RESTART REQUIRED !!!"
                    )
                else:
                    print(f"[ERROR] Arming failed ({result.get('reason', '?')}) - all nodes confirmed disarmed.\n")
                    joystick_states["status"] = f"ARMING FAILED ({result.get('reason', '?')}) - all nodes confirmed disarmed"
        was_arming_running = arming_seq.is_running()

        # --- PStap: start / pause / resume the safe-return sequence.
        # PShold past DISARM_HOLD_SECONDS is Estop - always available,
        # regardless of sequence state - and aborts the sequence too.
        if ps and not ps_prev:
            ps_press_time = time.time()
            if safe_return.is_running():
                safe_return.toggle_pause()
                joystick_states["status"] = ("SAFE RETURN: paused - tap PS to resume"
                                              if safe_return.pause_event.is_set()
                                              else "SAFE RETURN: running (all other input ignored)")
            elif all_armed:
                armed_ep = endpoints['endpoints']['axis0.is_armed']
                armed_ids = [nid for nid in node_ids
                             if read_config(bus, nid, armed_ep['id'], armed_ep['type'])]
                print(f"\n>>> PS TAPPED - starting safe-return sequence for armed nodes {armed_ids} <<<\n")
                joystick_states["status"] = "SAFE RETURN: running (all other input ignored)"
                safe_return.start(bus, armed_ids, endpoints, shoulder_ctrl, wrist_ctrl, joint_positions)
            else:
                # Not fully armed, arming still in progress, or locked
                # out - PS-tap does nothing. Deliberately does not touch
                # joystick_states["status"], so whatever alert is already
                # showing (frozen/locked-out/arming) stays visible.
                print("\n[INFO] PS tapped but ignored - not in a fully-armed, ready state.\n")
        if ps:
            if ps_press_time is not None and (time.time() - ps_press_time) >= DISARM_HOLD_SECONDS:
                safe_return.abort()
                arming_seq.abort()
                force_disarm_all(bus, node_ids, endpoints)
                all_armed = False
                ik_mode_active = False
                ik_mode_is_catalogued = False
                ik_catalogue_staging = False
                ik_catalogue_final_target = None
                ik_targets = None
                ik_pending_grip = None
                ik_request_thread = None  # discard any in-flight request - see 2026-08-13 note below
                sequence_active = False  # 2026-08-17: pick/place sequence killed by E-stop too
                joystick_states["status"] = "FORCED DISARM -- TORQUE CUT IMMEDIATELY"
                ps_press_time = None
        else:
            ps_press_time = None
        ps_prev = ps

        # --- Resync all_armed once the safe-return sequence stops running
        # (finished normally, disarming everyone in its own last stage, or
        # was just aborted by Estop above) - a quiet state resync, not a
        # fault, so no alert here.
        if was_seq_running and not safe_return.is_running():
            armed_ep = endpoints['endpoints']['axis0.is_armed']
            all_armed = bool(node_ids) and all(
                read_config(bus, nid, armed_ep['id'], armed_ep['type']) for nid in node_ids
            )
            # Any safe-return sequence completing (PS-hold or otherwise)
            # always clears IK-mode - never silently resume chasing a
            # stale target after a full reset, regardless of the
            # resulting all_armed value. Requires a completely fresh
            # Triangle-hold after any safe-return event. Also discards
            # any in-flight background request (2026-08-13): if one
            # completes AFTER this point, ik_request_thread being None
            # means the polling check below simply ignores the result -
            # the orphaned thread finishes harmlessly on its own, never
            # resurrecting ik_mode_active after a real disarm.
            ik_mode_active = False
            ik_mode_is_catalogued = False
            ik_catalogue_staging = False
            ik_catalogue_final_target = None
            ik_targets = None
            ik_pending_grip = None
            ik_request_thread = None
            sequence_active = False  # 2026-08-17: any safe-return completing also clears an in-progress pick/place sequence
        was_seq_running = safe_return.is_running()

        # --- Watchdog: while driving the arm normally (not during the
        # automated return-to-rest sequence, which already has its own
        # 15s-per-move stall detection), recheck a few times a second
        # that every node is still armed. If one unexpectedly drops, all
        # motion stops immediately - the still-armed nodes just hold
        # their last position, nothing gets disarmed - and arming is
        # locked out until restart. One flat response, no retry.
        if all_armed and not safe_return.is_running():
            disarm_check_counter += 1
            if disarm_check_counter % 6 == 0:
                armed_ep = endpoints['endpoints']['axis0.is_armed']
                readings = {nid: read_config(bus, nid, armed_ep['id'], armed_ep['type'])
                            for nid in node_ids}
                # 2026-08-01: an explicit False is unambiguous - the node
                # itself reported not armed - so it now acts immediately
                # (faster than before, which waited for a 2nd confirmation
                # even on a clear False). A timeout (None) is ambiguous -
                # real-world testing proved a single timed-out read can
                # happen on a provably healthy, error-free bus - so it
                # still needs to repeat on the next check before being
                # trusted, same as the original debounce behavior. Applied
                # 2026-08-18 (patch_watchdog_ambiguous_timeout.py existed
                # unapplied since 2026-08-01; hand-applied here since this
                # block has since grown the catalogue/sequence reset lines
                # below that the original patch's exact-text match didn't
                # account for).
                confirmed_false = set(nid for nid, v in readings.items() if v is False)
                timed_out = set(nid for nid, v in readings.items() if v is None)
                confirmed_timeout = timed_out & watchdog_pending_dropped
                watchdog_pending_dropped = timed_out
                confirmed = confirmed_false | confirmed_timeout
                if confirmed:
                    dropped = sorted(confirmed)
                    all_armed = False
                    ik_mode_active = False
                    ik_mode_is_catalogued = False
                    ik_catalogue_staging = False
                    ik_catalogue_final_target = None
                    ik_targets = None
                    ik_pending_grip = None
                    ik_request_thread = None
                    sequence_active = False  # 2026-08-17: an unexpected disarm mid-operation also kills a pick/place sequence
                    lockout_event.set()
                    print(f"\n[CRITICAL] Node(s) {dropped} unexpectedly disarmed during operation! "
                          f"All motion halted - the rest of the arm holds its last position. "
                          f"Restart gamecontroller.py and check hardware (check_armed.py) before "
                          f"continuing.\n")
                    joystick_states["status"] = (
                        f"!!! UNEXPECTED DISARM: {dropped} - ALL MOTION HALTED - RESTART REQUIRED !!!"
                    )
        else:
            watchdog_pending_dropped = set()

        motion_allowed = lb
        if not all_armed:
            motion_allowed = False
        if safe_return.is_running():
            motion_allowed = False
        if lockout_event.is_set():
            motion_allowed = False
        if awaiting_l1_reset:
            if not lb:
                l1_released_since_arm = True
            if l1_released_since_arm and lb:
                awaiting_l1_reset = False
                joystick_states["status"] = "L1 RE-PRESSED: motion enabled"
            else:
                motion_allowed = False

        # --- IK-mode trigger: Triangle held IK_HOLD_SECONDS under the
        # exact same conditions manual motion is already allowed under
        # (armed, not mid-safe-return, not locked out, L1 held - the
        # dead-man switch). Starts a BACKGROUND thread for the network
        # call to the IK solver service on redPi (2026-08-13: this used
        # to be a direct blocking call right here, which could freeze
        # this ENTIRE loop - including PS-hold responsiveness - for up
        # to its 2s timeout. Same fix pattern ArmingSequence already
        # uses for its own multi-step operation.) The main loop never
        # waits on it - just polls the result each frame below.
        if motion_allowed and triangle and not ik_mode_active and ik_request_thread is None:
            if ik_hold_start is None:
                ik_hold_start = time.time()
            elif time.time() - ik_hold_start >= IK_HOLD_SECONDS:
                catalogued = read_catalogue_target()
                if catalogued is not None:
                    _x, _y, _z, ik_pending_grip = read_ik_target()
                    try:
                        with open(IK_TARGET_FILE) as _f:
                            ik_catalogue_square_name = str(json.load(_f).get('square', '')).strip().lower()
                    except Exception:
                        ik_catalogue_square_name = None
                    ik_catalogue_settle_snapshot = {}
                    ik_catalogue_final_target = catalogued
                    ik_targets = CATALOGUE_STAGING_TARGET
                    ik_mode_active = True
                    ik_mode_is_catalogued = True
                    ik_catalogue_staging = True  # loose tolerance while True - see use_tight_tol below
                    ik_start_time = time.time()
                    ik_commanded_reached_time = None
                    ik_triangle_released_since_start = False
                    joystick_states["status"] = "IK MODE: catalogued target - staging first"
                    print(f"[IK] Catalogued target found, staging via CATALOGUE_STAGING_TARGET first, then: {ik_catalogue_final_target}")
                else:
                    ik_mode_is_catalogued = False
                    ik_catalogue_staging = False
                    ik_catalogue_final_target = None
                    x, y, z, ik_pending_grip = read_ik_target()
                    ik_request_result = {}
                    ik_request_thread = threading.Thread(
                        target=ik_request_worker, args=(x, y, z, ik_request_result), daemon=True
                    )
                    ik_request_thread.start()
                    joystick_states["status"] = "IK MODE: requesting target..."
                ik_hold_start = None
        elif ik_request_thread is None:
            ik_hold_start = None

        # --- Poll the background request, never block on it. Once it
        # reports done (success, rejection, or network failure), process
        # the result this frame and clear the thread handle.
        if ik_request_thread is not None and ik_request_result.get('done'):
            solve = ik_request_result.get('solve')
            parsed_ok = False
            if solve and solve.get("ok") and solve.get("within_urdf_bounds"):
                # Wrapped: a malformed/unexpected response shape (e.g. a
                # missing key) would otherwise raise uncaught here, same
                # crash-the-whole-loop risk as the unwrapped settle-check
                # reads found earlier. Any parse failure is treated the
                # same as a rejected solve - safe default, never moves.
                try:
                    raw = solve["raw_nodes"]
                    ik_targets = {0: raw["node0"], 1: raw["node1"], 3: raw["node3"], 4: raw["node4"], 5: raw["node5"], 6: raw["node6"]}
                    if ik_pending_grip is not None:
                        ik_targets[7] = ik_pending_grip
                    parsed_ok = True
                except Exception as e:
                    solve = {"parse_error": str(e)}
            if parsed_ok:
                ik_mode_active = True
                ik_start_time = time.time()
                ik_commanded_reached_time = None
                # Requires a real release-then-press before Triangle can
                # stop it - otherwise the same continuous hold that just
                # triggered START would immediately also trigger STOP
                # the instant ik_mode_active flips True.
                ik_triangle_released_since_start = False
                joystick_states["status"] = f"IK MODE: moving to {IK_TEST_TARGET_XYZ}"
                print(f"[IK] Target acquired: {ik_targets}")
            else:
                err = ik_request_result.get('error') or solve
                joystick_states["status"] = f"IK MODE: request failed/rejected ({err})"
                print(f"[IK] Solve rejected/failed: {err}")
            ik_request_thread = None
            ik_request_result = {}

        # --- IK-mode stop, two independent paths, either one cancels:
        # 1) any manual stick input - IK-mode never fights a human input.
        # 2) a single fresh Triangle press (release-then-press, tracked
        #    below) - the one deliberate, single-key "stop this" action.
        # Checked every frame it's active, before movement is applied.
        if ik_mode_active and not triangle:
            ik_triangle_released_since_start = True
        if ik_mode_active and triangle and ik_triangle_released_since_start:
            ik_mode_active = False
            ik_mode_is_catalogued = False
            ik_catalogue_staging = False
            ik_catalogue_final_target = None
            ik_targets = None
            ik_pending_grip = None
            joystick_states["status"] = "IK MODE: stopped (Triangle pressed)"
            print("[IK] Stopped - Triangle pressed, returning control.")
        elif ik_mode_active and (abs(raw_bend) > 0 or abs(raw_rotate) > 0 or abs(rx) > 0 or abs(ry) > 0):
            ik_mode_active = False
            ik_mode_is_catalogued = False
            ik_catalogue_staging = False
            ik_catalogue_final_target = None
            ik_targets = None
            ik_pending_grip = None
            joystick_states["status"] = "IK MODE: cancelled (manual input detected)"
            print("[IK] Cancelled - manual stick input detected, returning control.")

        # --- IK-mode overall timeout: a single hard backstop covering
        # every possible way this could otherwise run indefinitely (a
        # target that's unreachable within this script's real limits
        # despite passing the IK service's own bounds check, a
        # settle-check that never passes, anything). Always wins,
        # regardless of cause.
        if ik_mode_active and ik_start_time is not None and (time.time() - ik_start_time) > IK_TIMEOUT_SECONDS:
            ik_mode_active = False
            ik_mode_is_catalogued = False
            ik_catalogue_staging = False
            ik_catalogue_final_target = None
            ik_targets = None
            ik_pending_grip = None
            joystick_states["status"] = "IK MODE: timed out - did not reach target in time"
            print("[IK] Timed out - stopping, did not confirm arrival in time.")

        if motion_allowed and ik_mode_active:
            # IK-driven movement toward the fetched target - reuses the
            # exact same taper_increment() safety/decel logic as manual
            # moves, just with a fixed direction-to-target each frame
            # instead of live stick input. Wrist and gripper are NOT
            # touched here - they remain fully manual (see IK_TRIGGER_
            # BUTTON_INDEX comment above for why).
            all_arrived = True
            use_tight_tol = ik_mode_is_catalogued and not ik_catalogue_staging
            arrival_tol = CATALOGUE_ARRIVAL_TOLERANCE if use_tight_tol else IK_ARRIVAL_TOLERANCE
            settle_tol  = CATALOGUE_SETTLE_TOLERANCE  if use_tight_tol else IK_SETTLE_TOLERANCE

            # Smooth ramp from 0 to IK_VELOCITY_SCALING over IK_RAMP_UP_
            # SECONDS, synchronized across all 4 joints (all reference
            # the same ik_start_time) - see IK_RAMP_UP_SECONDS comment
            # above for why: avoids commanding near-full speed on all 4
            # joints simultaneously from a standing start.
            ramp_factor = 1.0
            if ik_start_time is not None:
                ramp_factor = min(1.0, (time.time() - ik_start_time) / IK_RAMP_UP_SECONDS)
            ik_speed = IK_VELOCITY_SCALING * ramp_factor

            # Diagnostic only - throttled to ~2x/sec so it doesn't spam
            # the log at 30Hz. Added 2026-08-13 after the first live
            # test ended with no visibility into what happened DURING
            # the run, only the end state - this prints live diff-to-
            # target for every IK-controlled joint plus the current
            # ramp/speed, so next time the actual in-progress behavior
            # is visible, not just guessed at afterward.
            if time.time() - ik_last_debug_print > 0.5:
                ik_last_debug_print = time.time()
                dbg = {}
                if 0 in ik_targets: dbg['n0'] = round(ik_targets[0] - joint_positions[0], 4)
                if 1 in ik_targets and shoulder_ctrl: dbg['n1'] = round(ik_targets[1] - shoulder_ctrl.value, 4)
                if 3 in ik_targets: dbg['n3'] = round(ik_targets[3] - joint_positions[3], 4)
                if 4 in ik_targets: dbg['n4'] = round(ik_targets[4] - joint_positions[4], 4)
                print(f"[IK][debug] diffs={dbg} ramp={ramp_factor:.2f} speed={ik_speed:.3f} L1={'held' if lb else 'RELEASED'}")

            if 0 in node_ids and 0 in ik_targets:
                diff = ik_targets[0] - joint_positions[0]
                if abs(diff) > arrival_tol:
                    all_arrived = False
                    step = max(-1.0, min(1.0, diff / TARGET_ARRIVAL_DECEL_ZONE)) * ik_speed * dt
                    joint_positions[0] += taper_increment(joint_positions[0], step, JOINT0_MIN, JOINT0_MAX, DECEL_ZONE)
                    if joint_positions[0] < JOINT0_MIN: joint_positions[0] = JOINT0_MIN
                    if joint_positions[0] > JOINT0_MAX: joint_positions[0] = JOINT0_MAX
                move_odrive_to_position(bus, 0, joint_positions[0])

            if shoulder_ctrl and (1 in node_ids) and (2 in node_ids) and 1 in ik_targets:
                diff = ik_targets[1] - shoulder_ctrl.value
                if abs(diff) > arrival_tol:
                    all_arrived = False
                    step = max(-1.0, min(1.0, diff / TARGET_ARRIVAL_DECEL_ZONE)) * ik_speed * dt
                    new_val = shoulder_ctrl.value + taper_increment(shoulder_ctrl.value, step, JOINT1_MIN, JOINT1_MAX, DECEL_ZONE)
                    if new_val < JOINT1_MIN: new_val = JOINT1_MIN
                    if new_val > JOINT1_MAX: new_val = JOINT1_MAX
                    shoulder_ctrl.value = new_val
                shoulder_ctrl.apply()

            if 3 in node_ids and 3 in ik_targets:
                diff = ik_targets[3] - joint_positions[3]
                if abs(diff) > arrival_tol:
                    all_arrived = False
                    step = max(-1.0, min(1.0, diff / TARGET_ARRIVAL_DECEL_ZONE)) * ik_speed * dt
                    joint_positions[3] += taper_increment(joint_positions[3], step, JOINT2_MIN, JOINT2_MAX, DECEL_ZONE)
                    if joint_positions[3] < JOINT2_MIN: joint_positions[3] = JOINT2_MIN
                    if joint_positions[3] > JOINT2_MAX: joint_positions[3] = JOINT2_MAX
                move_odrive_to_position(bus, 3, joint_positions[3])

            if 4 in node_ids and 4 in ik_targets:
                diff = ik_targets[4] - joint_positions[4]
                if abs(diff) > arrival_tol:
                    all_arrived = False
                    step = max(-1.0, min(1.0, diff / TARGET_ARRIVAL_DECEL_ZONE)) * ik_speed * dt
                    joint_positions[4] += taper_increment(joint_positions[4], step, JOINT3_MIN, JOINT3_MAX, DECEL_ZONE)
                    if joint_positions[4] < JOINT3_MIN: joint_positions[4] = JOINT3_MIN
                    if joint_positions[4] > JOINT3_MAX: joint_positions[4] = JOINT3_MAX
                move_odrive_to_position(bus, 4, joint_positions[4])

            if wrist_ctrl and 5 in node_ids and 6 in node_ids and 5 in ik_targets and 6 in ik_targets:
                # Wrist (nodes 5/6) toward its IK-solved target - same
                # tapered-step-toward-diff pattern as the other IK joints
                # above, but reusing the EXISTING dynamic Wrist Envelope
                # clamp (see the manual-drive wrist block for the original
                # derivation/history) rather than a simplified clamp, so
                # this can never bypass that already-proven safety logic.
                target_bend = (ik_targets[5] - ik_targets[6]) / 2.0
                target_rotate = (ik_targets[5] + ik_targets[6]) / 2.0
                bend_diff = target_bend - wrist_ctrl.bend_pos
                rotate_diff = target_rotate - wrist_ctrl.rotate_pos
                if abs(bend_diff) > arrival_tol or abs(rotate_diff) > arrival_tol:
                    all_arrived = False
                    bend_max_dyn   = min(BEND_MAX,   MOTOR5_MAX - wrist_ctrl.rotate_pos, wrist_ctrl.rotate_pos - MOTOR6_MIN)
                    bend_min_dyn   = max(BEND_MIN,   MOTOR5_MIN - wrist_ctrl.rotate_pos, wrist_ctrl.rotate_pos - MOTOR6_MAX)
                    rotate_max_dyn = min(ROTATE_MAX, MOTOR5_MAX - wrist_ctrl.bend_pos,   MOTOR6_MAX + wrist_ctrl.bend_pos)
                    rotate_min_dyn = max(ROTATE_MIN, MOTOR5_MIN - wrist_ctrl.bend_pos,   MOTOR6_MIN + wrist_ctrl.bend_pos)

                    bend_step = max(-1.0, min(1.0, bend_diff / TARGET_ARRIVAL_DECEL_ZONE)) * ik_speed * dt
                    rotate_step = max(-1.0, min(1.0, rotate_diff / TARGET_ARRIVAL_DECEL_ZONE)) * ik_speed * dt
                    # Floor the step magnitude (never the sign/direction),
                    # only for an axis that itself still needs real
                    # movement (its own diff exceeds tolerance) - avoids
                    # nudging an axis that's already individually settled
                    # just because the other axis kept this block active.
                    if abs(bend_diff) > arrival_tol and 0 < abs(bend_step) < MIN_WRIST_IK_STEP:
                        bend_step = MIN_WRIST_IK_STEP if bend_step > 0 else -MIN_WRIST_IK_STEP
                    if abs(rotate_diff) > arrival_tol and 0 < abs(rotate_step) < MIN_WRIST_IK_STEP:
                        rotate_step = MIN_WRIST_IK_STEP if rotate_step > 0 else -MIN_WRIST_IK_STEP
                    new_bend = wrist_ctrl.bend_pos + taper_increment(wrist_ctrl.bend_pos, bend_step, bend_min_dyn, bend_max_dyn, DECEL_ZONE_WRIST_BEND)
                    new_rotate = wrist_ctrl.rotate_pos + taper_increment(wrist_ctrl.rotate_pos, rotate_step, rotate_min_dyn, rotate_max_dyn, DECEL_ZONE_WRIST_ROTATE)

                    if new_bend < bend_min_dyn: new_bend = bend_min_dyn
                    if new_bend > bend_max_dyn: new_bend = bend_max_dyn
                    if new_rotate < rotate_min_dyn: new_rotate = rotate_min_dyn
                    if new_rotate > rotate_max_dyn: new_rotate = rotate_max_dyn

                    wrist_ctrl.bend_pos = new_bend
                    wrist_ctrl.rotate_pos = new_rotate
                wrist_ctrl.apply()

            if 7 in node_ids and 7 in ik_targets:
                # Gripper (node7) toward an explicit target, only when the
                # IK request actually specified one (ik_targets only ever
                # gets a 7 key from the 'grip' field in ik_target.json -
                # see read_ik_target()). Same tapered-step pattern as the
                # other IK joints, reusing TRIGGER_MIN/MAX and
                # DECEL_ZONE_GRIPPER - the exact same bounds the manual
                # trigger-driven gripper block below already uses. That
                # manual block is intentionally never gated by
                # ik_mode_active (human can always override the grip), so
                # it still runs every frame after this one - harmless: with
                # no trigger pressed it just re-sends whatever this block
                # set, and a real trigger press still moves the gripper
                # further from wherever this block left it.
                diff = ik_targets[7] - joint_positions[7]
                if abs(diff) > IK_ARRIVAL_TOLERANCE:
                    all_arrived = False
                    step = max(-1.0, min(1.0, diff / TARGET_ARRIVAL_DECEL_ZONE)) * ik_speed * dt
                    joint_positions[7] += taper_increment(joint_positions[7], step, TRIGGER_MIN, TRIGGER_MAX, DECEL_ZONE_GRIPPER)
                    if joint_positions[7] < TRIGGER_MIN: joint_positions[7] = TRIGGER_MIN
                    if joint_positions[7] > TRIGGER_MAX: joint_positions[7] = TRIGGER_MAX
                move_odrive_to_position(bus, 7, joint_positions[7])

            if all_arrived:
                # Commanded trajectory has reached target. Don't declare
                # success yet - confirm against REAL live position first
                # (joint_positions[i]/shoulder_ctrl.value above are this
                # script's own commanded values, not a live encoder
                # read-back - see design discussion 2026-08-13). This
                # check is purely passive: it only ever reads position,
                # never issues a new movement command based on what it
                # finds - failing just means "check again next interval,"
                # bounded by the overall IK_TIMEOUT_SECONDS backstop
                # above, never "try to correct it."
                if ik_commanded_reached_time is None:
                    ik_commanded_reached_time = time.time()
                elif (time.time() - ik_commanded_reached_time) >= IK_SETTLE_WAIT_SECONDS:
                    # Wrapped: read_config() can return None on a CAN
                    # timeout (handled internally, not an exception), or
                    # raise on a deeper bus fault (not handled
                    # internally) - there is no outer exception handler
                    # around this whole control loop, so an uncaught
                    # error here would crash the entire thread, killing
                    # manual control too, not just IK-mode. Either
                    # failure just means "can't confirm this check, try
                    # again next interval" - same as a normal not-yet-
                    # settled result, never a crash.
                    settled = True
                    try:
                        for nid, target in ik_targets.items():
                            # Match the exact same node_ids membership
                            # guards the movement code above uses - a
                            # joint that was never actually part of this
                            # run (missing hardware) must never block
                            # settling forever waiting on a read that
                            # can never succeed.
                            if nid == 0 and 0 not in node_ids:
                                continue
                            if nid == 1 and not (shoulder_ctrl and 1 in node_ids and 2 in node_ids):
                                continue
                            if nid == 3 and 3 not in node_ids:
                                continue
                            if nid == 4 and 4 not in node_ids:
                                continue
                            live = read_position(bus, nid, endpoints)
                            if use_tight_tol:
                                # Catalogue-replay accuracy logging only -
                                # see the log write in "elif settled:"
                                # below. No-op (skipped entirely) for the
                                # normal live-solve path.
                                ik_catalogue_settle_snapshot[nid] = (target, live)
                            if live is None or abs(target - live) > settle_tol:
                                settled = False
                                break
                    except Exception as e:
                        settled = False
                        print(f"[IK] Settle-check read failed ({e}) - will retry next interval.")

                    if settled and ik_catalogue_staging:
                        # Staging leg done - now drive the second, tight-
                        # tolerance leg into the actual catalogued target,
                        # approaching from this same known pose every time.
                        ik_catalogue_staging = False
                        ik_targets = ik_catalogue_final_target
                        ik_catalogue_final_target = None
                        ik_start_time = time.time()  # fresh IK_TIMEOUT_SECONDS budget for this leg
                        ik_commanded_reached_time = None
                        joystick_states["status"] = "IK MODE: staged, now approaching catalogued target"
                        print(f"[IK] Staging leg settled - approaching final catalogued target: {ik_targets}")
                    elif settled:
                        if use_tight_tol:
                            # Catalogue-replay accuracy log - local file
                            # append only, no network, no solver call
                            # (catalogued moves never touch the solver).
                            # Wrapped so any failure (disk full, whatever)
                            # is silently skipped, same as any other log
                            # line - never something this loop waits on
                            # or can be blocked/obstructed by.
                            try:
                                with open(CATALOGUE_REPLAY_LOG_FILE, 'a') as _f:
                                    _f.write(json.dumps({
                                        'time': time.strftime('%Y-%m-%d %H:%M:%S'),
                                        'square': ik_catalogue_square_name,
                                        'nodes': {
                                            f'node{_nid}': {'target': _t, 'live': _l}
                                            for _nid, (_t, _l) in ik_catalogue_settle_snapshot.items()
                                        },
                                    }) + '\n')
                            except Exception:
                                pass
                        ik_mode_active = False
                        ik_targets = None
                        ik_pending_grip = None
                        ik_mode_is_catalogued = False
                        joystick_states["status"] = "IK MODE: target reached (settled)"
                        print("[IK] Target reached and settled, returning to manual control.")
                    else:
                        ik_commanded_reached_time = time.time()
                        joystick_states["status"] = "IK MODE: at target, confirming settled..."
            else:
                ik_commanded_reached_time = None

        # --- Pick/Place sequence trigger (2026-08-17): Square (pick) /
        # Cross (place) held PICK_PLACE_HOLD_SECONDS, same gating
        # conditions as Triangle/IK-mode (armed, not mid-safe-return, not
        # locked out, L1 held). Unlike Triangle, this does NOT call the
        # live solver - it drives through a short PRE-SOLVED sequence of
        # waypoints (see build_pick_waypoints/build_place_waypoints,
        # generated once offline, not live) for full repeatability, per
        # explicit request to remove solver variability from the standard
        # pick/place motion. Mutually exclusive with ik_mode_active
        # (Triangle) and with itself (can't start a second sequence while
        # one is already running).
        if motion_allowed and pick_btn and not sequence_active and not ik_mode_active:
            if pick_hold_start is None:
                pick_hold_start = time.time()
            elif time.time() - pick_hold_start >= PICK_PLACE_HOLD_SECONDS:
                grip_val = read_pick_place_grip()
                cur = current_raw_nodes(joint_positions, shoulder_ctrl, wrist_ctrl)
                wps = build_pick_waypoints(cur, grip_val)
                sequence_active = True
                sequence_kind = 'PICK'
                sequence_square = None
                sequence_waypoints = wps
                sequence_index = 0
                sequence_start_time = time.time()
                sequence_wp_reached_time = None
                sequence_button_released_since_start = False
                joystick_states["status"] = f"PICK: step 1/{len(wps)}"
                print(f"[SEQ] Starting PICK from current position, {len(wps)} steps")
                pick_hold_start = None
        elif not pick_btn:
            pick_hold_start = None

        if motion_allowed and place_btn and not sequence_active and not ik_mode_active:
            if place_hold_start is None:
                place_hold_start = time.time()
            elif time.time() - place_hold_start >= PICK_PLACE_HOLD_SECONDS:
                grip_val = read_pick_place_grip()
                cur = current_raw_nodes(joint_positions, shoulder_ctrl, wrist_ctrl)
                wps = build_place_waypoints(cur, grip_val)
                sequence_active = True
                sequence_kind = 'PLACE'
                sequence_square = None
                sequence_waypoints = wps
                sequence_index = 0
                sequence_start_time = time.time()
                sequence_wp_reached_time = None
                sequence_button_released_since_start = False
                joystick_states["status"] = f"PLACE: step 1/{len(wps)}"
                print(f"[SEQ] Starting PLACE from current position, {len(wps)} steps")
                place_hold_start = None
        elif not place_btn:
            place_hold_start = None

        # --- Sequence stop: any manual stick input always cancels
        # immediately (never fights a human input, same rule as
        # IK-mode). A fresh press (release-then-press) of whichever
        # button started this sequence also cancels it - same single
        # deliberate "stop" gesture pattern Triangle already uses.
        if sequence_active:
            trigger_btn = pick_btn if sequence_kind == 'PICK' else place_btn
            if not trigger_btn:
                sequence_button_released_since_start = True
            if trigger_btn and sequence_button_released_since_start:
                sequence_active = False
                btn_name = 'Square' if sequence_kind == 'PICK' else 'Cross'
                joystick_states["status"] = f"{sequence_kind} MODE: stopped ({btn_name} pressed)"
                print(f"[SEQ] Stopped - button pressed, returning control.")
            elif abs(raw_bend) > 0 or abs(raw_rotate) > 0 or abs(rx) > 0 or abs(ry) > 0:
                sequence_active = False
                joystick_states["status"] = f"{sequence_kind} MODE: cancelled (manual input detected)"
                print(f"[SEQ] Cancelled - manual stick input detected, returning control.")

        # --- Per-waypoint hard timeout - same reasoning as
        # IK_TIMEOUT_SECONDS: covers a waypoint that's unreachable or
        # never settles, so this can never run forever. Budget resets
        # fresh for each new waypoint (see sequence_start_time reset
        # below), not shared across the whole sequence.
        if (sequence_active and sequence_start_time is not None
                and (time.time() - sequence_start_time) > SEQUENCE_WP_TIMEOUT_SECONDS):
            sequence_active = False
            joystick_states["status"] = f"{sequence_kind} MODE: timed out on step {sequence_index + 1}/{len(sequence_waypoints)}"
            print(f"[SEQ] Timed out on step {sequence_index + 1} - stopping.")

        if motion_allowed and sequence_active:
            # Drive toward the CURRENT waypoint - same taper_increment
            # safety logic, ramp-up, and settle-check pattern as IK-mode
            # above, just looped across a short list of pre-solved
            # waypoints instead of a single live-solved target.
            wp, grip_target, _label = sequence_waypoints[sequence_index]

            all_arrived = True
            ramp_factor = 1.0
            if sequence_start_time is not None:
                ramp_factor = min(1.0, (time.time() - sequence_start_time) / IK_RAMP_UP_SECONDS)
            seq_speed = IK_VELOCITY_SCALING * ramp_factor

            if 0 in node_ids:
                diff = wp[0] - joint_positions[0]
                if abs(diff) > SEQUENCE_ARRIVAL_TOLERANCE:
                    all_arrived = False
                    step_amt = max(-1.0, min(1.0, diff / TARGET_ARRIVAL_DECEL_ZONE)) * seq_speed * dt
                    joint_positions[0] += taper_increment(joint_positions[0], step_amt, JOINT0_MIN, JOINT0_MAX, DECEL_ZONE)
                    if joint_positions[0] < JOINT0_MIN: joint_positions[0] = JOINT0_MIN
                    if joint_positions[0] > JOINT0_MAX: joint_positions[0] = JOINT0_MAX
                move_odrive_to_position(bus, 0, joint_positions[0])

            if shoulder_ctrl and (1 in node_ids) and (2 in node_ids):
                diff = wp[1] - shoulder_ctrl.value
                if abs(diff) > SEQUENCE_ARRIVAL_TOLERANCE:
                    all_arrived = False
                    step_amt = max(-1.0, min(1.0, diff / TARGET_ARRIVAL_DECEL_ZONE)) * seq_speed * dt
                    new_val = shoulder_ctrl.value + taper_increment(shoulder_ctrl.value, step_amt, JOINT1_MIN, JOINT1_MAX, DECEL_ZONE)
                    if new_val < JOINT1_MIN: new_val = JOINT1_MIN
                    if new_val > JOINT1_MAX: new_val = JOINT1_MAX
                    shoulder_ctrl.value = new_val
                shoulder_ctrl.apply()

            if 3 in node_ids:
                diff = wp[3] - joint_positions[3]
                if abs(diff) > SEQUENCE_ARRIVAL_TOLERANCE:
                    all_arrived = False
                    step_amt = max(-1.0, min(1.0, diff / TARGET_ARRIVAL_DECEL_ZONE)) * seq_speed * dt
                    joint_positions[3] += taper_increment(joint_positions[3], step_amt, JOINT2_MIN, JOINT2_MAX, DECEL_ZONE)
                    if joint_positions[3] < JOINT2_MIN: joint_positions[3] = JOINT2_MIN
                    if joint_positions[3] > JOINT2_MAX: joint_positions[3] = JOINT2_MAX
                move_odrive_to_position(bus, 3, joint_positions[3])

            if 4 in node_ids:
                diff = wp[4] - joint_positions[4]
                if abs(diff) > SEQUENCE_ARRIVAL_TOLERANCE:
                    all_arrived = False
                    step_amt = max(-1.0, min(1.0, diff / TARGET_ARRIVAL_DECEL_ZONE)) * seq_speed * dt
                    joint_positions[4] += taper_increment(joint_positions[4], step_amt, JOINT3_MIN, JOINT3_MAX, DECEL_ZONE)
                    if joint_positions[4] < JOINT3_MIN: joint_positions[4] = JOINT3_MIN
                    if joint_positions[4] > JOINT3_MAX: joint_positions[4] = JOINT3_MAX
                move_odrive_to_position(bus, 4, joint_positions[4])

            if wrist_ctrl and 5 in node_ids and 6 in node_ids:
                target_bend = (wp[5] - wp[6]) / 2.0
                target_rotate = (wp[5] + wp[6]) / 2.0
                bend_diff = target_bend - wrist_ctrl.bend_pos
                rotate_diff = target_rotate - wrist_ctrl.rotate_pos
                if abs(bend_diff) > SEQUENCE_ARRIVAL_TOLERANCE or abs(rotate_diff) > SEQUENCE_ARRIVAL_TOLERANCE:
                    all_arrived = False
                    bend_max_dyn   = min(BEND_MAX,   MOTOR5_MAX - wrist_ctrl.rotate_pos, wrist_ctrl.rotate_pos - MOTOR6_MIN)
                    bend_min_dyn   = max(BEND_MIN,   MOTOR5_MIN - wrist_ctrl.rotate_pos, wrist_ctrl.rotate_pos - MOTOR6_MAX)
                    rotate_max_dyn = min(ROTATE_MAX, MOTOR5_MAX - wrist_ctrl.bend_pos,   MOTOR6_MAX + wrist_ctrl.bend_pos)
                    rotate_min_dyn = max(ROTATE_MIN, MOTOR5_MIN - wrist_ctrl.bend_pos,   MOTOR6_MIN + wrist_ctrl.bend_pos)
                    bend_step = max(-1.0, min(1.0, bend_diff / TARGET_ARRIVAL_DECEL_ZONE)) * seq_speed * dt
                    rotate_step = max(-1.0, min(1.0, rotate_diff / TARGET_ARRIVAL_DECEL_ZONE)) * seq_speed * dt
                    if abs(bend_diff) > SEQUENCE_ARRIVAL_TOLERANCE and 0 < abs(bend_step) < MIN_WRIST_IK_STEP:
                        bend_step = MIN_WRIST_IK_STEP if bend_step > 0 else -MIN_WRIST_IK_STEP
                    if abs(rotate_diff) > SEQUENCE_ARRIVAL_TOLERANCE and 0 < abs(rotate_step) < MIN_WRIST_IK_STEP:
                        rotate_step = MIN_WRIST_IK_STEP if rotate_step > 0 else -MIN_WRIST_IK_STEP
                    new_bend = wrist_ctrl.bend_pos + taper_increment(wrist_ctrl.bend_pos, bend_step, bend_min_dyn, bend_max_dyn, DECEL_ZONE_WRIST_BEND)
                    new_rotate = wrist_ctrl.rotate_pos + taper_increment(wrist_ctrl.rotate_pos, rotate_step, rotate_min_dyn, rotate_max_dyn, DECEL_ZONE_WRIST_ROTATE)
                    if new_bend < bend_min_dyn: new_bend = bend_min_dyn
                    if new_bend > bend_max_dyn: new_bend = bend_max_dyn
                    if new_rotate < rotate_min_dyn: new_rotate = rotate_min_dyn
                    if new_rotate > rotate_max_dyn: new_rotate = rotate_max_dyn
                    wrist_ctrl.bend_pos = new_bend
                    wrist_ctrl.rotate_pos = new_rotate
                wrist_ctrl.apply()

            if 7 in node_ids:
                diff = grip_target - joint_positions[7]
                if abs(diff) > GRIP_ARRIVAL_TOLERANCE:
                    all_arrived = False
                    step_amt = max(-1.0, min(1.0, diff / TARGET_ARRIVAL_DECEL_ZONE)) * seq_speed * dt
                    joint_positions[7] += taper_increment(joint_positions[7], step_amt, TRIGGER_MIN, TRIGGER_MAX, DECEL_ZONE_GRIPPER)
                    if joint_positions[7] < TRIGGER_MIN: joint_positions[7] = TRIGGER_MIN
                    if joint_positions[7] > TRIGGER_MAX: joint_positions[7] = TRIGGER_MAX
                move_odrive_to_position(bus, 7, joint_positions[7])

            if all_arrived:
                # Commanded trajectory reached this waypoint - confirm
                # against REAL live position before advancing, same
                # passive-only-never-corrects pattern as IK-mode's own
                # settle-check.
                if sequence_wp_reached_time is None:
                    sequence_wp_reached_time = time.time()
                elif (time.time() - sequence_wp_reached_time) >= SEQUENCE_SETTLE_WAIT_SECONDS:
                    settled = True
                    try:
                        for nid, tgt in ((0, wp[0]), (3, wp[3]), (4, wp[4]), (7, grip_target)):
                            if nid not in node_ids:
                                continue
                            live = read_position(bus, nid, endpoints)
                            tol = GRIP_SETTLE_TOLERANCE if nid == 7 else SEQUENCE_SETTLE_TOLERANCE
                            if live is None or abs(tgt - live) > tol:
                                settled = False
                                break
                    except Exception as e:
                        settled = False
                        print(f"[SEQ] Settle-check read failed ({e}) - will retry next interval.")

                    if settled:
                        sequence_index += 1
                        sequence_wp_reached_time = None
                        sequence_start_time = time.time()  # fresh timeout budget for the next waypoint
                        if sequence_index >= len(sequence_waypoints):
                            joystick_states["status"] = f"{sequence_kind}: complete"
                            print(f"[SEQ] {sequence_kind} complete.")
                            sequence_active = False
                        else:
                            joystick_states["status"] = f"{sequence_kind}: step {sequence_index + 1}/{len(sequence_waypoints)}"
                    else:
                        sequence_wp_reached_time = time.time()
            else:
                sequence_wp_reached_time = None

        if motion_allowed:
            # Possibly controlling the normal joints or the gripper
            wrist_mode = (rb == 1)

            # Require the left stick back near neutral before it's allowed
            # to drive whichever axis set just became active (base/shoulder
            # <-> wrist). Real bug found 2026-08-17: rb is read fresh every
            # frame with no memory of the prior frame, so releasing R1 while
            # the stick was still held over (e.g. mid wrist-bend) let that
            # SAME stick deflection instantly drive base/shoulder instead,
            # unexpectedly - reported as "joint2 moves and smashes the
            # fingers into the board" right after releasing R1 near the
            # board. Same fix pattern as awaiting_l1_reset above. Applies
            # both directions (entering OR leaving wrist mode).
            if wrist_mode != wrist_mode_prev:
                awaiting_stick_neutral = True
            wrist_mode_prev = wrist_mode
            if awaiting_stick_neutral:
                if raw_bend == 0.0 and raw_rotate == 0.0:
                    awaiting_stick_neutral = False
                else:
                    raw_bend = 0.0
                    raw_rotate = 0.0

            # While IK-mode owns base/shoulder/elbow-roll/elbow (see IK
            # block above), skip their manual control here - but wrist
            # and gripper below are NEVER gated by ik_mode_active, they
            # stay fully manual throughout. Note: rx/ry/raw_rotate/
            # raw_bend are guaranteed ~0 whenever ik_mode_active is True
            # (any nonzero stick input cancels IK-mode the same frame,
            # above), so this guard is a belt-and-suspenders skip, not
            # load-bearing on its own.
            if not ik_mode_active:
                # Joint 2 => node3 => Right Stick X
                if 3 in node_ids:
                    joint_positions[3] += taper_increment(joint_positions[3], rx * VELOCITY_SCALING * dt, JOINT2_MIN, JOINT2_MAX, DECEL_ZONE)
                    if joint_positions[3] < JOINT2_MIN: joint_positions[3] = JOINT2_MIN
                    if joint_positions[3] > JOINT2_MAX: joint_positions[3] = JOINT2_MAX
                    move_odrive_to_position(bus, 3, joint_positions[3])

                # Joint 3 => node4 => Right Stick Y
                if 4 in node_ids:
                    joint_positions[4] += taper_increment(joint_positions[4], ry * VELOCITY_SCALING * dt, JOINT3_MIN, JOINT3_MAX, DECEL_ZONE)
                    if joint_positions[4] < JOINT3_MIN: joint_positions[4] = JOINT3_MIN
                    if joint_positions[4] > JOINT3_MAX: joint_positions[4] = JOINT3_MAX
                    move_odrive_to_position(bus, 4, joint_positions[4])

            if not wrist_mode:
                # Normal left-stick => Joint 0 (node0, horizontal) & Joint 1 (node1,2, vertical)
                if not ik_mode_active:
                    if 0 in node_ids:
                        joint_positions[0] += taper_increment(joint_positions[0], raw_rotate * VELOCITY_SCALING * dt, JOINT0_MIN, JOINT0_MAX, DECEL_ZONE)
                        if joint_positions[0] < JOINT0_MIN: joint_positions[0] = JOINT0_MIN
                        if joint_positions[0] > JOINT0_MAX: joint_positions[0] = JOINT0_MAX
                        move_odrive_to_position(bus, 0, joint_positions[0])

                    if shoulder_ctrl and (1 in node_ids) and (2 in node_ids):
                        new_val = shoulder_ctrl.value + taper_increment(shoulder_ctrl.value, raw_bend * SHOULDER_VELOCITY_SCALING * dt, JOINT1_MIN, JOINT1_MAX, DECEL_ZONE)
                        if new_val < JOINT1_MIN: new_val = JOINT1_MIN
                        if new_val > JOINT1_MAX: new_val = JOINT1_MAX
                        shoulder_ctrl.value = new_val
                        shoulder_ctrl.apply()

            else:
                # Wrist mode => node5,6
                if wrist_ctrl and (5 in node_ids) and (6 in node_ids):
                    # Left stick vertical => bend (tilt), horizontal => rotate (twist)
                    wrist_bend_input   = raw_bend 
                    wrist_rotate_input = raw_rotate   

                    # Wrist Envelope: dynamic bend/rotate limits derived from the
                    # motorA/motorB constraint algebra (motorA=rotate+bend,
                    # motorB=rotate-bend, each clamped to MOTOR5/6_MIN/MAX), so the
                    # motor-level clamp can never fire from a single-axis stick input
                    # alone. Previously, hitting the motor clamp while moving only one
                    # axis silently dragged the OTHER axis's decomposed value along
                    # with it (confirmed live: rotating while near BEND_MIN pulled the
                    # decomposed bend from -2.83 back to -1.25 with zero bend input,
                    # because node6/motorB froze at its ceiling while node5/motorA kept
                    # climbing). Cross-terms use the OTHER axis's value from the start
                    # of this frame (not yet updated), which is close enough at 100Hz.
                    bend_max_dyn   = min(BEND_MAX,   MOTOR5_MAX - wrist_ctrl.rotate_pos, wrist_ctrl.rotate_pos - MOTOR6_MIN)
                    bend_min_dyn   = max(BEND_MIN,   MOTOR5_MIN - wrist_ctrl.rotate_pos, wrist_ctrl.rotate_pos - MOTOR6_MAX)
                    rotate_max_dyn = min(ROTATE_MAX, MOTOR5_MAX - wrist_ctrl.bend_pos,   MOTOR6_MAX + wrist_ctrl.bend_pos)
                    rotate_min_dyn = max(ROTATE_MIN, MOTOR5_MIN - wrist_ctrl.bend_pos,   MOTOR6_MIN + wrist_ctrl.bend_pos)

                    new_bend   = wrist_ctrl.bend_pos   + taper_increment(wrist_ctrl.bend_pos, wrist_bend_input * VELOCITY_SCALING * FOREARM_VELOCITY_SCALING * dt, bend_min_dyn, bend_max_dyn, DECEL_ZONE_WRIST_BEND)
                    new_rotate = wrist_ctrl.rotate_pos + taper_increment(wrist_ctrl.rotate_pos, wrist_rotate_input * VELOCITY_SCALING * FOREARM_VELOCITY_SCALING * dt, rotate_min_dyn, rotate_max_dyn, DECEL_ZONE_WRIST_ROTATE)

                    if new_bend < bend_min_dyn: new_bend = bend_min_dyn
                    if new_bend > bend_max_dyn: new_bend = bend_max_dyn
                    if new_rotate < rotate_min_dyn: new_rotate = rotate_min_dyn
                    if new_rotate > rotate_max_dyn: new_rotate = rotate_max_dyn

                    wrist_ctrl.bend_pos   = new_bend
                    wrist_ctrl.rotate_pos = new_rotate
                    wrist_ctrl.apply()

            # Always handle the Gripper ODrive (node7) if present
            if 7 in node_ids:
                new_pos = joint_positions[7]
                # Pressing left trigger => move negative
                if raw_lt > 0: new_pos += taper_increment(new_pos, -raw_lt * VELOCITY_SCALING * GRIPPER_SCALING * dt, TRIGGER_MIN, TRIGGER_MAX, DECEL_ZONE_GRIPPER)
                # Pressing right trigger => move positive
                if raw_rt > 0: new_pos += taper_increment(new_pos, raw_rt * VELOCITY_SCALING * GRIPPER_SCALING * dt, TRIGGER_MIN, TRIGGER_MAX, DECEL_ZONE_GRIPPER)
                
                # Clamp in [TRIGGER_MIN, TRIGGER_MAX]
                if new_pos < TRIGGER_MIN: new_pos = TRIGGER_MIN
                elif new_pos > TRIGGER_MAX: new_pos = TRIGGER_MAX

                joint_positions[7] = new_pos
                move_odrive_to_position(bus, 7, joint_positions[7])

        time.sleep(0.01)

# ------------------------------------------------------------------------------
# 7) Main Function
# ------------------------------------------------------------------------------
def main():
    signal.signal(signal.SIGINT, signal_handler)

    bus = can.interface.Bus("can0", bustype = "socketcan")
    discovered = list(discover_node_ids(bus))
    endpoints  = load_endpoints()

    if not discovered:
        print("[ERROR] No ODrives found on the CAN bus.")
        return

    discovered.sort()
    print(f"Discovered ODrive Node IDs: {discovered}")

    max_id = max(discovered)
    joint_positions = [0.0] * (max_id + 1)

    # Sync: read each node's actual current position while still idle, and
    # write it back as its own target, so arming can't cause a snap toward
    # a stale/default setpoint.
    print("[INFO] Reading actual current positions...")
    for nid in discovered:
        pos = read_position(bus, nid, endpoints)
        if pos is None:
            print(f"[WARN] Could not read position for node {nid}, assuming 0.0")
            pos = 0.0
        joint_positions[nid] = pos
        print(f"    Node {nid}: {pos:.4f}")

    # No auto-arm here anymore. Nodes stay disarmed until you physically
    # perform the L1 + Circle gesture inside the live loop below,
    # which also gates on the arm actually being at rest_pos - this
    # guarantees nothing can move until you're holding the controller
    # and deliberately choose to arm it.

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("[ERROR] No joystick found.")
        pygame.quit()
        return

    stick = pygame.joystick.Joystick(0)
    stick.init()
    print(f"Joystick: {stick.get_name()}")
    print(f"# Axes: {stick.get_numaxes()}")

    # Shoulder => node1,2
    shoulder_ctrl = None
    if (1 in discovered) and (2 in discovered):
        shoulder_ctrl = ShoulderController(bus, [1,2])
        shoulder_ctrl.value = (joint_positions[1] - joint_positions[2]) / 2.0
        print("[INFO] ShoulderController for node1,node2 created.")

    # Wrist => node5,6
    wrist_ctrl = None
    if (5 in discovered) and (6 in discovered):
        wrist_ctrl = WriteController(bus, [5,6])
        wrist_ctrl.rotate_pos = (joint_positions[5] + joint_positions[6]) / 2.0
        wrist_ctrl.bend_pos   = (joint_positions[5] - joint_positions[6]) / 2.0
        print("[INFO] WriteController for node5,node6 created.")

    safe_return = SafeReturnSequence()
    arming_seq  = ArmingSequence()
    lockout_event = threading.Event()

    print("\n" + "="*70)
    print("About to enter live control. Nodes are NOT armed - nothing can")
    print("move until you perform L1 + Circle (held) with the")
    print("controller in hand. Arming will be refused unless the arm is")
    print("still at rest_pos.")
    print("="*70)
    input("Press Enter to continue...")

    metrics_text = urwid.Text("Metrics...", align = 'left')
    joystick_text = urwid.Text("", align = 'left')
    box_metrics = urwid.LineBox(metrics_text, title = "ODrive Metrics")
    box_joy     = urwid.LineBox(joystick_text, title = "Controller Inputs")

    pile = urwid.Pile([box_metrics, box_joy])
    foot = urwid.Text("Press ESC to exit", align = 'center')
    frame = urwid.Frame(pile, footer = foot)

    loop = urwid.MainLoop(
        frame,
        palette = [('reversed','standout','')],
        unhandled_input = lambda k: handle_input(k, loop, discovered, bus, joint_positions)
    )

    ui_thread = threading.Thread(
        target = update_ui_thread,
        args = (bus, discovered, endpoints, metrics_text, joystick_text, loop),
        daemon = True
    )
    ui_thread.start()

    joy_thread = threading.Thread(
        target = joystick_thread_func,
        args = (bus, discovered, joint_positions, shoulder_ctrl, wrist_ctrl, endpoints, safe_return, arming_seq, lockout_event),
        daemon = True
    )
    joy_thread.start()

    try:
        loop.run()
    except KeyboardInterrupt:
        pass
    finally:
        print("[INFO] Main loop ended => stopping threads.")
        stop_event.set()
        ui_thread.join()
        joy_thread.join()
        print("[INFO] Threads joined => final shutdown.")
        clean_shutdown(discovered, bus, joint_positions, endpoints, safe_return, arming_seq, lockout_event, shoulder_ctrl, wrist_ctrl)
        pygame.quit()


if __name__ == "__main__":
    main()