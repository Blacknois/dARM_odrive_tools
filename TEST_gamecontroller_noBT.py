#!/usr/bin/env python3

import time
import threading
import signal
import urwid
import pygame
import can
import builtins
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
DPAD_ARM_DIRECTION      = (-1, 0)  # D-pad left
ARM_HOLD_SECONDS        = 1.5
DISARM_HOLD_SECONDS     = 1.0

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
FOREARM_VELOCITY_SCALING = 1.8   # was 2.0 - same 10% reduction
GRIPPER_SCALING          = 0.5

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
JOINT1_MIN, JOINT1_MAX = -5.43,  0.0
JOINT2_MIN, JOINT2_MAX = -10.00,  12.58  # Elbow roll - measured 2026-07-31 (wire limit -10.14/12.72, 0.14 margin each side)
JOINT3_MIN, JOINT3_MAX =  0.0,   5.76

BEND_MIN,   BEND_MAX     =  -8.0,  8.0 # Wrist - confirmed final 2026-07-31 (tested throughout, no incident)
ROTATE_MIN, ROTATE_MAX   = -15.0, 15.0 # Wrist - confirmed final 2026-07-31 (tested throughout, no incident)
TRIGGER_MIN, TRIGGER_MAX = -0.85, 0.0  # placeholder range in the NEW reference
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
MOTOR5_MIN, MOTOR5_MAX = -28.41, 6.06
MOTOR6_MIN, MOTOR6_MAX = -27.36, 4.57

# Soft-limit deceleration: commanded speed scales down within this
# distance (same units as the joint ranges above) of a min/max limit.
DECEL_ZONE         = 1.0
DECEL_ZONE_WRIST   = 3.0
DECEL_ZONE_GRIPPER = 0.2

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

stop_event = threading.Event()

# Joystick states for UI display
joystick_states = {
    "LB": False,
    "Dpad": False,
    "Circle": False,
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
    """
    if abs(value) < DEAD_ZONE:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - DEAD_ZONE) / (1.0 - DEAD_ZONE)

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

def handle_input(key, loop, node_ids, bus, joint_positions,
                  endpoints=None, arming_seq=None, safe_return=None,
                  shoulder_ctrl=None, wrist_ctrl=None):
    if key == 'esc':
        stop_event.set()
        raise urwid.ExitMainLoop()
    # TEST MODE (noBT) keyboard triggers - call the exact same
    # underlying safety-critical functions the real Dpad+Circle
    # gesture and PS-tap use, just triggered by keyboard instead of
    # a physical controller.
    if key == ' ':
        if arming_seq is not None and endpoints is not None and not arming_seq.is_running():
            print("\n[TEST MODE] SPACEBAR pressed - arming all nodes directly "
                  "(validate, then arm one at a time, verified)...\n")
            arming_seq.start(bus, node_ids, endpoints)
    if key == 'q':
        if safe_return is not None and endpoints is not None:
            if safe_return.is_running():
                safe_return.toggle_pause()
            else:
                armed_ep = endpoints['endpoints']['axis0.is_armed']
                armed_ids = [nid for nid in node_ids
                             if read_config(bus, nid, armed_ep['id'], armed_ep['type'])]
                if armed_ids:
                    print(f"\n[TEST MODE] 'q' PRESSED - starting safe-return sequence "
                          f"for armed nodes {armed_ids} <<<\n")
                    safe_return.start(bus, armed_ids, endpoints, shoulder_ctrl, wrist_ctrl, joint_positions)
                else:
                    print("[TEST MODE] 'q' pressed but no nodes are armed - nothing to do.")

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
    # TEST MODE (noBT): always run the full staged sequence, even if
    # already at rest_pos - real gamecontroller.py/run_safe_return_sequence
    # is unaffected since this is a separate copy of the file.
    already_there = False
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
            wait_for_position(bus, list(axial_targets.keys()), endpoints, axial_targets,
                               pause_event=pause_event, abort_event=abort_event)

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

        dpad_str = "HELD" if joystick_states["Dpad"] else "-"
        circle_str = "HELD" if joystick_states["Circle"] else "-"
        ps_str = "HELD" if joystick_states["PS"] else "-"
        gesture_line = f"DpadLeft:{dpad_str}  Circle:{circle_str}  PS:{ps_str}"
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

    dpad_arm_hold_start   = None
    arm_triggered_this_hold = False
    awaiting_l1_reset     = False
    l1_released_since_arm = False
    ps_prev               = False
    ps_press_time         = None
    all_armed             = False
    was_seq_running        = False
    was_arming_running     = False
    disarm_check_counter  = 0
    watchdog_pending_dropped = set()

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
            dpad = joystick.get_hat(0)
            circle = joystick.get_button(ARM_BUTTON_INDEX)
            ps = joystick.get_button(PS_BUTTON_INDEX)
        except pygame.error:
            if connected:
                print("\n[SAFETY] Lost contact with controller! Freezing all motion.\n")
            connected = False
            time.sleep(0.05)
            continue

        joystick_states["LB"] = bool(lb)
        joystick_states["Dpad"] = (dpad == DPAD_ARM_DIRECTION)
        joystick_states["Circle"] = bool(circle)
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

        # --- Arm gesture: D-pad left + Circle held 1.5s. Arming itself
        # runs on its own thread (ArmingSequence) so this loop is never
        # blocked waiting on it and keeps checking PS-hold every frame no
        # matter how long arming takes.
        if dpad == DPAD_ARM_DIRECTION and circle and not lockout_event.is_set():
            if dpad_arm_hold_start is None:
                dpad_arm_hold_start = time.time()
            elif (not arm_triggered_this_hold) and (time.time() - dpad_arm_hold_start) >= ARM_HOLD_SECONDS:
                if not arming_seq.is_running():
                    print("\n[INFO] Arm gesture held - arming all nodes in the background "
                          "(validate, then arm one at a time, verified)...\n")
                    joystick_states["status"] = "ARMING: validating and arming all nodes..."
                    arming_seq.start(bus, node_ids, endpoints)
                arm_triggered_this_hold = True
        else:
            dpad_arm_hold_start = None
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
                dropped_now = set(nid for nid in node_ids
                                   if read_config(bus, nid, armed_ep['id'], armed_ep['type']) is not True)
                confirmed = dropped_now & watchdog_pending_dropped
                watchdog_pending_dropped = dropped_now
                if confirmed:
                    dropped = sorted(confirmed)
                    all_armed = False
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

        if motion_allowed:
            # Possibly controlling the normal joints or the gripper
            wrist_mode = (rb == 1)

            # Joint 2 => node3 => Right Stick X
            if 3 in node_ids:
                joint_positions[3] += taper_increment(joint_positions[3], rx * VELOCITY_SCALING * dt, JOINT2_MIN, JOINT2_MAX, DECEL_ZONE)
                if joint_positions[3] < JOINT2_MIN: joint_positions[3] = JOINT2_MIN 
                if joint_positions[3] > JOINT2_MAX: joint_positions[3] = JOINT2_MAX 
                move_odrive_to_position(bus, 3, joint_positions[3])  

            # Joint 3 => node4 => Right Stick Y
            if 4 in node_ids:
                joint_positions[4] += taper_increment(joint_positions[4], -ry * VELOCITY_SCALING * dt, JOINT3_MIN, JOINT3_MAX, DECEL_ZONE)
                if joint_positions[4] < JOINT3_MIN: joint_positions[4] = JOINT3_MIN  
                if joint_positions[4] > JOINT3_MAX: joint_positions[4] = JOINT3_MAX  
                move_odrive_to_position(bus, 4, joint_positions[4])  

            if not wrist_mode:
                # Normal left-stick => Joint 0 (node0, horizontal) & Joint 1 (node1,2, vertical)
                if 0 in node_ids:
                    joint_positions[0] += taper_increment(joint_positions[0], raw_rotate * VELOCITY_SCALING * dt, JOINT0_MIN, JOINT0_MAX, DECEL_ZONE)
                    if joint_positions[0] < JOINT0_MIN: joint_positions[0] = JOINT0_MIN
                    if joint_positions[0] > JOINT0_MAX: joint_positions[0] = JOINT0_MAX
                    move_odrive_to_position(bus, 0, joint_positions[0])

                if shoulder_ctrl and (1 in node_ids) and (2 in node_ids):
                    new_val = shoulder_ctrl.value + taper_increment(shoulder_ctrl.value, raw_bend * VELOCITY_SCALING * dt, JOINT1_MIN, JOINT1_MAX, DECEL_ZONE)
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

                    new_bend   = wrist_ctrl.bend_pos   + taper_increment(wrist_ctrl.bend_pos, wrist_bend_input * VELOCITY_SCALING * FOREARM_VELOCITY_SCALING * dt, BEND_MIN, BEND_MAX, DECEL_ZONE_WRIST)
                    new_rotate = wrist_ctrl.rotate_pos + taper_increment(wrist_ctrl.rotate_pos, wrist_rotate_input * VELOCITY_SCALING * FOREARM_VELOCITY_SCALING * dt, ROTATE_MIN, ROTATE_MAX, DECEL_ZONE_WRIST)

                    if new_bend < BEND_MIN: new_bend = BEND_MIN
                    if new_bend > BEND_MAX: new_bend = BEND_MAX
                    if new_rotate < ROTATE_MIN: new_rotate = ROTATE_MIN
                    if new_rotate > ROTATE_MAX: new_rotate = ROTATE_MAX

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
    # perform the Dpad-Left + Circle gesture inside the live loop below,
    # which also gates on the arm actually being at rest_pos - this
    # guarantees nothing can move until you're holding the controller
    # and deliberately choose to arm it.

    # TEST MODE (noBT): no physical joystick/Bluetooth controller is
    # used. pygame.joystick is monkeypatched globally so every call
    # site in this file (including the reconnect logic further up in
    # joystick_thread_func) gets a harmless no-op stick instead of
    # touching real hardware/Bluetooth.
    class _NoOpJoystick:
        def init(self): pass
        def get_hat(self, idx): return (0, 0)
        def get_button(self, idx): return False
        def get_axis(self, idx): return 0.0
        def get_name(self): return "TEST MODE - no physical joystick (noBT)"
        def get_numaxes(self): return 6

    pygame.joystick.Joystick = lambda idx: _NoOpJoystick()
    pygame.joystick.get_count = lambda: 1

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
    print("\n[TEST MODE] SPACEBAR = arm all nodes directly. 'q' = PS-tap (safe-return).\n")

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
    print("[TEST MODE - NO BLUETOOTH] Nodes are NOT armed - nothing can move")
    print("until you press SPACEBAR (arms all nodes directly, no hold needed).")
    print("Press 'q' for PS-tap (safe-return). Arming will be refused unless")
    print("the arm is still at rest_pos. Press ESC to exit.")
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
        unhandled_input = lambda k: handle_input(k, loop, discovered, bus, joint_positions,
                                                  endpoints, arming_seq, safe_return,
                                                  shoulder_ctrl, wrist_ctrl)
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