#!/usr/bin/env python3
"""
Compiles all agreed changes to gamecontroller.py into one pass:
  - safe_up_pos / rest_pos constants
  - a coordinated, pausable, Estop-aware staged safe-return sequence
    (replaces move_to_neutral_slowly), shared by PStap and by exit
  - PStap becomes a start/pause/resume state machine; all other input
    ignored while it's running or paused; Estop always available
  - startup no longer auto-arms; arming only happens via the existing
    Dpad-Left + Circle gesture, gated on the arm actually being at
    rest_pos
  - clean_shutdown() properly waits for an in-progress sequence instead
    of racing it with a second, independent move (the bug that sheared
    the wrist assembly)

Every block below is matched against the exact current file content and
asserted to occur exactly once before anything is written - if any
assertion fails, NOTHING is written, so a mismatch fails safe instead
of silently corrupting the file.
"""

path = "gamecontroller.py"
src = open(path).read()
original_src = src

def apply_edit(name, old, new):
    global src
    count = src.count(old)
    assert count == 1, f"[{name}] expected 1 match, found {count} - aborting, nothing written."
    src = src.replace(old, new)
    print(f"[{name}] OK")

# ---------------------------------------------------------------------------
# EDIT A: add safe_up_pos / rest_pos constants
# ---------------------------------------------------------------------------
apply_edit(
    "A: safe_up_pos/rest_pos constants",
    '''BEND_MIN,   BEND_MAX     =  -5.0,  5.0 # Wrist
ROTATE_MIN, ROTATE_MAX   = -10.0, 10.0 # Wrist
TRIGGER_MIN, TRIGGER_MAX =  0.0, 0.80

# Node5/6 raw safety envelope, measured empirically from a combined''',
    '''BEND_MIN,   BEND_MAX     =  -5.0,  5.0 # Wrist
ROTATE_MIN, ROTATE_MAX   = -10.0, 10.0 # Wrist
TRIGGER_MIN, TRIGGER_MAX =  0.0, 0.80

# ------------------------------------------------------------------------
# Named positions (####_pos convention). rest_pos is the folded stow
# pose the arm physically sits in when powered on/off (target 0.0 for
# every node except the gripper, which rests at GRIPPER_STARTUP_OPEN).
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
GRIPPER_RELEASE_OPEN   = TRIGGER_MAX  # full open - guarantees release of
                                        # anything held before any arm motion
REST_POS_TOLERANCE     = 0.1      # radians - how close counts as "at rest_pos"
                                    # for the arming gate (proxy check only -
                                    # pos_estimate re-zeros at boot regardless
                                    # of true physical position, so this can't
                                    # replace physically verifying the arm is
                                    # at the marked resting pose)

# Node5/6 raw safety envelope, measured empirically from a combined''',
)

# ---------------------------------------------------------------------------
# EDIT B: clean_shutdown - wait for an in-progress sequence instead of
# racing it, and fall back to the same safe sequence if something is
# still armed but the sequence never ran
# ---------------------------------------------------------------------------
apply_edit(
    "B: clean_shutdown",
    '''def clean_shutdown(node_ids, bus, joint_positions, endpoints):
    print("\\nExiting... Setting discovered ODrives to pos=0 => IDLE => shutdown.")
    try:
        move_to_neutral_slowly(bus, node_ids, endpoints)
    except Exception as e:
        print(f"[WARN] Error during slow move to neutral: {e}")
    failed = []
    for nid in node_ids:
        if not disarm_verified(bus, nid, endpoints):
            failed.append(nid)
    if failed:
        print(f"[WARNING] Nodes NOT confirmed disarmed: {failed} - check manually (check_armed.py) before next power-on.")
    else:
        print("[INFO] All nodes confirmed disarmed.")
    if bus:
        bus.shutdown()''',
    '''def clean_shutdown(node_ids, bus, joint_positions, endpoints, safe_return, shoulder_ctrl, wrist_ctrl):
    """
    Exit path. Never runs a second, independent safe-return move
    concurrently with one that's already active - that race (an
    orphaned PStap sequence thread still running while shutdown fired
    its own move) is what sheared the wrist assembly. Instead:
      1. If the sequence is running/paused, wait for it to finish.
      2. If it doesn't finish in time, stop here - Estop and manual
         placement at rest_pos is required, not a second move.
      3. If nothing was ever armed, there's nothing to do.
      4. If something is armed and the sequence never ran (e.g. ESC
         pressed without ever tapping PStap), fall back to the same
         staged sequence rather than the old naive all-at-once move.
    """
    print("\\nExiting...")

    if safe_return.is_running():
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
        bus.shutdown()''',
)

# ---------------------------------------------------------------------------
# EDIT C: replace move_to_neutral_slowly with wait_for_position(),
# SafeReturnSequence, and run_safe_return_sequence()
# ---------------------------------------------------------------------------
apply_edit(
    "C: move_to_neutral_slowly -> staged safe-return sequence",
    '''def move_to_neutral_slowly(bus, node_ids, endpoints, targets=None):
    """
    Temporarily halves each node's trap_traj vel/accel/decel limits (not
    saved to flash - reverts on next reboot or setup.py run), moves every
    node to position 0 (the photographed rest pose), and waits for the
    move to complete before returning. Optional `targets` dict maps
    node_id -> target position; any node not listed defaults to 0.0.

    Both the halve step and the restore step use write_verified() to
    confirm each write actually landed via read-back - CAN writes on
    this bus can silently fail, which previously caused trap_traj
    limits to be repeatedly halved without ever being restored.
    """
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
        if vel is not None and not write_verified(bus, nid, vel_ep['id'], vel_ep['type'], vel / 2, label="trap_vel halve"):
            failed_halve.append((nid, "vel"))
        if accel is not None and not write_verified(bus, nid, accel_ep['id'], accel_ep['type'], accel / 2, label="trap_accel halve"):
            failed_halve.append((nid, "accel"))
        if decel is not None and not write_verified(bus, nid, decel_ep['id'], decel_ep['type'], decel / 2, label="trap_decel halve"):
            failed_halve.append((nid, "decel"))
    if failed_halve:
        print(f"[WARNING] Speed-limit halve not confirmed for: {failed_halve} - proceeding with move anyway.")

    print("[INFO] Moving slowly to neutral (rest) position...")
    for nid in node_ids:
        target = targets[nid] if (targets and nid in targets) else 0.0
        move_odrive_to_position(bus, nid, target)
    time.sleep(4)

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
        print(f"[WARNING] Speed limits NOT confirmed restored for: {failed_restore} - check manually (diag_node_compare2.py) before further motion.")
    else:
        print("[INFO] Neutral position reached. Normal speed limits restored (confirmed).")''',
    '''def wait_for_position(bus, node_ids, endpoints, targets, tolerance=0.05, timeout=15.0,
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
        if vel is not None and not write_verified(bus, nid, vel_ep['id'], vel_ep['type'], vel / 2, label="trap_vel halve"):
            failed_halve.append((nid, "vel"))
        if accel is not None and not write_verified(bus, nid, accel_ep['id'], accel_ep['type'], accel / 2, label="trap_accel halve"):
            failed_halve.append((nid, "accel"))
        if decel is not None and not write_verified(bus, nid, decel_ep['id'], decel_ep['type'], decel / 2, label="trap_decel halve"):
            failed_halve.append((nid, "decel"))
    if failed_halve:
        print(f"[WARNING] Speed-limit halve not confirmed for: {failed_halve} - proceeding anyway.")

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
        if 7 in node_ids:
            rest_targets[7] = GRIPPER_STARTUP_OPEN
        for nid in node_ids:
            move_odrive_to_position(bus, nid, rest_targets[nid])
        if shoulder_ctrl:
            shoulder_ctrl.value = 0.0
            shoulder_ctrl.apply()
        if wrist_ctrl:
            wrist_ctrl.bend_pos   = 0.0
            wrist_ctrl.rotate_pos = 0.0
            wrist_ctrl.apply()
        wait_for_position(bus, node_ids, endpoints, rest_targets,
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

    print("[SAFE_UP] Stage 7: disarming...")
    failed = []
    for nid in node_ids:
        if not disarm_verified(bus, nid, endpoints):
            failed.append(nid)
    if failed:
        print(f"[WARNING] Nodes NOT confirmed disarmed: {failed} - check manually (check_armed.py).")
    else:
        print("[SAFE_UP] Sequence complete - arm at rest_pos, all nodes disarmed.")


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
            self.thread.join(timeout=timeout)''',
)

# ---------------------------------------------------------------------------
# EDIT D: joystick_thread_func signature - accept safe_return
# ---------------------------------------------------------------------------
apply_edit(
    "D: joystick_thread_func signature",
    '''def joystick_thread_func(
    bus, node_ids, joint_positions,
    shoulder_ctrl, wrist_ctrl, endpoints,
    update_rate = UPDATE_RATE
):''',
    '''def joystick_thread_func(
    bus, node_ids, joint_positions,
    shoulder_ctrl, wrist_ctrl, endpoints, safe_return,
    update_rate = UPDATE_RATE
):''',
)

# ---------------------------------------------------------------------------
# EDIT E: arm gesture gains the rest_pos gate; PStap becomes the
# start/pause/resume state machine driving safe_return instead of the
# old unjoined return_thread
# ---------------------------------------------------------------------------
apply_edit(
    "E: arm gesture rest_pos gate + PStap state machine",
    '''    dpad_arm_hold_start   = None
    arm_triggered_this_hold = False
    awaiting_l1_reset     = False
    l1_released_since_arm = False
    ps_prev               = False
    ps_press_time         = None
    return_thread         = None''',
    '''    dpad_arm_hold_start   = None
    arm_triggered_this_hold = False
    awaiting_l1_reset     = False
    l1_released_since_arm = False
    ps_prev               = False
    ps_press_time         = None''',
)

apply_edit(
    "E2: arm gesture body - add rest_pos gate",
    '''            elif (not arm_triggered_this_hold) and (time.time() - dpad_arm_hold_start) >= ARM_HOLD_SECONDS:
                print("\\n[INFO] Arm gesture held - re-syncing and arming all nodes...\\n")
                joystick_states["status"] = "ARMING: re-syncing all nodes..."
                newly_armed = set()
                for nid in node_ids:
                    pos = None
                    for attempt in range(3):
                        pos = read_position(bus, nid, endpoints)
                        if pos is not None:
                            break
                        time.sleep(0.05)
                    if pos is None:
                        print(f"[WARN] Node {nid}: position read failed after 3 attempts - skipping resync/arm for this node this attempt (no guessed target commanded).")
                        continue
                    joint_positions[nid] = pos
                    move_odrive_to_position(bus, nid, pos)
                    if arm_node_verified(bus, nid, endpoints):
                        newly_armed.add(nid)
                        print(f"[INFO] Node {nid} confirmed armed.")
                    else:
                        print(f"[WARN] Node {nid} did NOT arm.")
                if shoulder_ctrl and (1 in node_ids) and (2 in node_ids):''',
    '''            elif (not arm_triggered_this_hold) and (time.time() - dpad_arm_hold_start) >= ARM_HOLD_SECONDS:
                print("\\n[INFO] Arm gesture held - checking rest_pos and arming...\\n")
                joystick_states["status"] = "ARMING: checking rest_pos..."
                newly_armed = set()
                not_at_rest = []
                for nid in node_ids:
                    pos = None
                    for attempt in range(3):
                        pos = read_position(bus, nid, endpoints)
                        if pos is not None:
                            break
                        time.sleep(0.05)
                    if pos is None:
                        print(f"[WARN] Node {nid}: position read failed after 3 attempts - skipping resync/arm for this node this attempt (no guessed target commanded).")
                        continue
                    rest_target = GRIPPER_STARTUP_OPEN if nid == 7 else 0.0
                    if abs(pos - rest_target) > REST_POS_TOLERANCE:
                        not_at_rest.append((nid, round(pos, 4)))
                        continue
                    joint_positions[nid] = pos
                    move_odrive_to_position(bus, nid, pos)
                    if arm_node_verified(bus, nid, endpoints):
                        newly_armed.add(nid)
                        print(f"[INFO] Node {nid} confirmed armed.")
                    else:
                        print(f"[WARN] Node {nid} did NOT arm.")
                if not_at_rest:
                    print(f"[WARNING] Not at rest_pos, arming skipped for: {not_at_rest} - "
                          f"verify the arm is physically at the marked resting pose.")
                if shoulder_ctrl and (1 in node_ids) and (2 in node_ids):''',
)

apply_edit(
    "E3: PS tap/hold -> start/pause/resume state machine, Estop aborts sequence",
    '''        # --- Disarm: PS tap = stop + slow return to rest; PS held 1s = force disarm ---
        if ps and not ps_prev:
            print("\\n>>> PS PRESSED - STOPPING AND RETURNING TO REST <<<\\n")
            joystick_states["status"] = "PS PRESSED: stopping + returning to rest (nodes stay armed)"
            ps_press_time = time.time()
            if return_thread is None or not return_thread.is_alive():
                def _return_to_rest_and_sync():
                    move_to_neutral_slowly(bus, node_ids, endpoints)
                    for nid in node_ids:
                        joint_positions[nid] = 0.0
                    if shoulder_ctrl:
                        shoulder_ctrl.value = 0.0
                    if wrist_ctrl:
                        wrist_ctrl.bend_pos = 0.0
                        wrist_ctrl.rotate_pos = 0.0
                return_thread = threading.Thread(target=_return_to_rest_and_sync, daemon=True)
                return_thread.start()
        if ps:
            if ps_press_time is not None and (time.time() - ps_press_time) >= DISARM_HOLD_SECONDS:
                force_disarm_all(bus, node_ids, endpoints)
                joystick_states["status"] = "FORCED DISARM -- TORQUE CUT IMMEDIATELY"
                ps_press_time = None
        else:
            ps_press_time = None
        ps_prev = ps

        motion_allowed = lb
        if return_thread is not None and return_thread.is_alive():
            motion_allowed = False''',
    '''        # --- PStap: start / pause / resume the safe-return sequence.
        # PShold past DISARM_HOLD_SECONDS is Estop - always available,
        # regardless of sequence state - and aborts the sequence too.
        if ps and not ps_prev:
            ps_press_time = time.time()
            if not safe_return.is_running():
                print("\\n>>> PS TAPPED - starting safe-return sequence <<<\\n")
                joystick_states["status"] = "SAFE RETURN: running (all other input ignored)"
                safe_return.start(bus, node_ids, endpoints, shoulder_ctrl, wrist_ctrl, joint_positions)
            else:
                safe_return.toggle_pause()
                joystick_states["status"] = ("SAFE RETURN: paused - tap PS to resume"
                                              if safe_return.pause_event.is_set()
                                              else "SAFE RETURN: running (all other input ignored)")
        if ps:
            if ps_press_time is not None and (time.time() - ps_press_time) >= DISARM_HOLD_SECONDS:
                safe_return.abort()
                force_disarm_all(bus, node_ids, endpoints)
                joystick_states["status"] = "FORCED DISARM -- TORQUE CUT IMMEDIATELY"
                ps_press_time = None
        else:
            ps_press_time = None
        ps_prev = ps

        motion_allowed = lb
        if safe_return.is_running():
            motion_allowed = False''',
)

# ---------------------------------------------------------------------------
# EDIT F: main() - remove auto-arm at startup, no auto-move to neutral,
# wire up safe_return, pass it through to joystick_thread_func and
# clean_shutdown
# ---------------------------------------------------------------------------
apply_edit(
    "F: startup position sync - drop redundant move-on-unarmed-node",
    '''    print("[INFO] Reading actual current positions...")
    for nid in discovered:
        pos = read_position(bus, nid, endpoints)
        if pos is None:
            print(f"[WARN] Could not read position for node {nid}, assuming 0.0")
            pos = 0.0
        joint_positions[nid] = pos
        move_odrive_to_position(bus, nid, pos)
        print(f"    Node {nid}: {pos:.4f}")

    # Arm with verification - do not trust that the CAN send alone means
    # the node actually entered closed-loop control.
    armed_nodes = set()
    for nid in discovered:
        if arm_node_verified(bus, nid, endpoints):
            armed_nodes.add(nid)
            print(f"[INFO] Node {nid} confirmed armed.")
        else:
            print(f"[WARN] Node {nid} did NOT arm - it will be excluded from control.")

    if not armed_nodes:
        print("[ERROR] No nodes armed successfully. Exiting.")
        bus.shutdown()
        return

    discovered = sorted(armed_nodes)''',
    '''    print("[INFO] Reading actual current positions...")
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
    # and deliberately choose to arm it.''',
)

apply_edit(
    "F2: startup messaging - no auto-move, safe_return instance, wiring",
    '''    print("\\n" + "="*70)
    print("About to move slowly to the neutral rest position (matches the")
    print("robot's marked resting pose). Ensure the arm's range of motion")
    print("is clear.")
    print("="*70)
    input("Press Enter to continue...")
    startup_targets = {7: GRIPPER_STARTUP_OPEN} if 7 in discovered else None
    move_to_neutral_slowly(bus, discovered, endpoints, targets=startup_targets)
    joint_positions = [0.0] * (max_id + 1)
    if 7 in discovered:
        joint_positions[7] = GRIPPER_STARTUP_OPEN
    if shoulder_ctrl: shoulder_ctrl.value = 0.0
    if wrist_ctrl:
        wrist_ctrl.bend_pos   = 0.0
        wrist_ctrl.rotate_pos = 0.0''',
    '''    safe_return = SafeReturnSequence()

    print("\\n" + "="*70)
    print("About to enter live control. Nodes are NOT armed - nothing can")
    print("move until you perform Dpad-Left + Circle (held) with the")
    print("controller in hand. Arming will be refused unless the arm is")
    print("still at rest_pos.")
    print("="*70)
    input("Press Enter to continue...")''',
)

apply_edit(
    "F3: joy_thread args - pass safe_return",
    '''    joy_thread = threading.Thread(
        target = joystick_thread_func,
        args = (bus, discovered, joint_positions, shoulder_ctrl, wrist_ctrl, endpoints),
        daemon = True
    )
    joy_thread.start()''',
    '''    joy_thread = threading.Thread(
        target = joystick_thread_func,
        args = (bus, discovered, joint_positions, shoulder_ctrl, wrist_ctrl, endpoints, safe_return),
        daemon = True
    )
    joy_thread.start()''',
)

apply_edit(
    "F4: clean_shutdown call - pass safe_return, shoulder_ctrl, wrist_ctrl",
    '''        clean_shutdown(discovered, bus, joint_positions, endpoints)
        pygame.quit()''',
    '''        clean_shutdown(discovered, bus, joint_positions, endpoints, safe_return, shoulder_ctrl, wrist_ctrl)
        pygame.quit()''',
)

# All edits applied in memory without error - now write out.
open(path, "w").write(src)
print(f"\\nAll edits applied. {len(original_src.splitlines())} -> {len(src.splitlines())} lines.")
