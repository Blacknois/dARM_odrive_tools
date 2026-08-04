#!/usr/bin/env python3
"""
On top of every previous patch. Three changes, all agreed in chat:

  1. Arming now runs on its own background thread (ArmingSequence,
     same pattern as SafeReturnSequence), so the main joystick loop -
     which must always stay free to check PS-hold every frame - is
     never blocked waiting on it, no matter how long arming takes.
     Each node still gets the same patience as before (5 tries, 0.3s
     apart) - only WHERE that runs changed, not how long it's allowed.
     No motion is sent to anything until the thread reports every node
     armed. PS-hold now also cancels an in-progress arming attempt.

  2. A watchdog, active while driving the arm normally (not during the
     automated return-to-rest sequence, which already has its own
     15s-per-move stall detection): a few times a second, it rechecks
     that every node is still armed. If one unexpectedly drops, ALL
     motion commands stop immediately - the nodes that are still armed
     simply hold their last position, since they stop getting fresh
     targets, and none of them get disarmed. Arming is then locked out
     until gamecontroller.py is restarted, with a warning that stays on
     the status line. One flat response - no retry, no escalation.

  3. `all_armed` is the single source of truth for whether motion is
     allowed, kept in sync at every point nodes go from armed to
     disarmed (successful arm, Estop, sequence completion, or the
     watchdog above) - motion is refused whenever it's not True.

Same safety pattern as every patch before: every block is matched and
asserted to occur exactly once before anything is written.
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
# EDIT A: clean_shutdown - accept arming_seq, abort+wait on it before the
# existing safe_return handling
# ---------------------------------------------------------------------------
apply_edit(
    "A: clean_shutdown signature + arming_seq handling",
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

    if safe_return.is_running():''',
    '''def clean_shutdown(node_ids, bus, joint_positions, endpoints, safe_return, arming_seq, shoulder_ctrl, wrist_ctrl):
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
    print("\\nExiting...")

    if arming_seq.is_running():
        print("[INFO] Arming still in progress - aborting it before exit...")
        arming_seq.abort()
        if arming_seq.thread is not None:
            arming_seq.thread.join(timeout=15)

    if safe_return.is_running():''',
)

# ---------------------------------------------------------------------------
# EDIT B: insert run_arming_sequence() + ArmingSequence class before
# SafeReturnSequence
# ---------------------------------------------------------------------------
apply_edit(
    "B: ArmingSequence class + run_arming_sequence()",
    '''class SafeReturnSequence:''',
    '''def run_arming_sequence(bus, node_ids, endpoints, abort_event):
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
        # Node 7 (gripper) is exempt from the rest_pos check: unlike the
        # arm joints, it has no real mechanical zero to check pos_estimate
        # against (it re-zeros to wherever it physically sits at boot,
        # not a fixed reference), and it doesn't carry the "swept through
        # an unknown pose" risk that motivated this gate for the joints.
        if nid != 7 and abs(pos - 0.0) > REST_POS_TOLERANCE:
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


class SafeReturnSequence:''',
)

# ---------------------------------------------------------------------------
# EDIT C: joystick_thread_func signature - accept arming_seq
# ---------------------------------------------------------------------------
apply_edit(
    "C: joystick_thread_func signature",
    '''def joystick_thread_func(
    bus, node_ids, joint_positions,
    shoulder_ctrl, wrist_ctrl, endpoints, safe_return,
    update_rate = UPDATE_RATE
):''',
    '''def joystick_thread_func(
    bus, node_ids, joint_positions,
    shoulder_ctrl, wrist_ctrl, endpoints, safe_return, arming_seq,
    update_rate = UPDATE_RATE
):''',
)

# ---------------------------------------------------------------------------
# EDIT D: state vars
# ---------------------------------------------------------------------------
apply_edit(
    "D: new state vars",
    '''    ps_prev               = False
    ps_press_time         = None
    arming_locked_out     = False''',
    '''    ps_prev               = False
    ps_press_time         = None
    arming_locked_out     = False
    all_armed             = False
    was_seq_running        = False
    was_arming_running     = False
    disarm_check_counter  = 0''',
)

# ---------------------------------------------------------------------------
# EDIT E: replace the inline arm gesture with a thread-start + a
# separate result-pickup step that runs every frame regardless of
# gesture state (since arming keeps running even if the gesture is
# released early)
# ---------------------------------------------------------------------------
apply_edit(
    "E: threaded arm gesture + result pickup",
    '''        # --- Arm gesture: D-pad left + Circle held 1.5s ---
        if dpad == DPAD_ARM_DIRECTION and circle and not arming_locked_out:
            if dpad_arm_hold_start is None:
                dpad_arm_hold_start = time.time()
            elif (not arm_triggered_this_hold) and (time.time() - dpad_arm_hold_start) >= ARM_HOLD_SECONDS:
                print("\\n[INFO] Arm gesture held - validating all nodes before arming...\\n")
                joystick_states["status"] = "ARMING: validating all nodes..."

                # Pass 1: check every node BEFORE arming any of them. This
                # robot cannot operate safely with even one node left
                # disarmed, so any single failure here aborts the whole
                # attempt with nothing armed - never a partial result.
                positions = {}
                failures = []
                for nid in node_ids:
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
                    # Node 7 (gripper) is exempt from the rest_pos check:
                    # unlike the arm joints, it has no real mechanical zero
                    # to check pos_estimate against (it re-zeros to wherever
                    # it physically sits at boot, not a fixed reference),
                    # and it doesn't carry the "swept through an unknown
                    # pose" risk that motivated this gate for the joints.
                    if nid != 7 and abs(pos - 0.0) > REST_POS_TOLERANCE:
                        failures.append((nid, f"not at rest_pos ({round(pos, 4)})"))

                if failures:
                    print(f"[ERROR] Arming aborted - not every node is ready: {failures}. "
                          f"NO nodes armed. Verify the arm is physically at the marked "
                          f"resting pose and retry.")
                    joystick_states["status"] = f"ARMING FAILED: {failures} -- nothing armed"
                else:
                    # Pass 2: every node validated - arm them one at a time,
                    # verified. If any single node fails to confirm armed,
                    # disarm everything armed so far in this pass - never
                    # leave a partial arm live.
                    newly_armed = []
                    failed_nid = None
                    for nid in node_ids:
                        move_odrive_to_position(bus, nid, positions[nid])
                        if arm_node_verified(bus, nid, endpoints):
                            newly_armed.append(nid)
                            print(f"[INFO] Node {nid} confirmed armed.")
                        else:
                            print(f"[WARN] Node {nid} did NOT arm.")
                            failed_nid = nid
                            break

                    if failed_nid is not None:
                        print(f"[ERROR] Node {failed_nid} failed to arm - disarming the "
                              f"{len(newly_armed)} node(s) that had already armed "
                              f"({newly_armed}) so nothing is left partially armed.")
                        for armed_nid in newly_armed:
                            if not disarm_verified(bus, armed_nid, endpoints):
                                print(f"[WARNING] Node {armed_nid} did NOT confirm disarmed - "
                                      f"check manually (check_armed.py).")

                        # Full sweep of every discovered node, not just the
                        # ones just rolled back - confirm NOTHING is armed
                        # before allowing another attempt. A read failure
                        # counts as "not confirmed disarmed" (fail-safe).
                        armed_ep = endpoints['endpoints']['axis0.is_armed']
                        still_armed = [nid for nid in node_ids
                                       if read_config(bus, nid, armed_ep['id'], armed_ep['type']) is not False]
                        if still_armed:
                            arming_locked_out = True
                            print(f"\\n[CRITICAL] Arming aborted, and node(s) {still_armed} did NOT "
                                  f"confirm disarmed. DO NOT operate the arm. Run check_armed.py and "
                                  f"resolve this manually, then restart gamecontroller.py - the arm "
                                  f"gesture is locked out for the rest of this session.\\n")
                            joystick_states["status"] = (
                                f"!!! DISARM NOT CONFIRMED: {still_armed} - RUN check_armed.py - "
                                f"ARM GESTURE LOCKED OUT, RESTART REQUIRED !!!"
                            )
                        else:
                            joystick_states["status"] = f"ARMING FAILED: node {failed_nid} - all nodes confirmed disarmed"
                    else:
                        for nid in node_ids:
                            joint_positions[nid] = positions[nid]
                        if shoulder_ctrl and (1 in node_ids) and (2 in node_ids):
                            shoulder_ctrl.value = (joint_positions[1] - joint_positions[2]) / 2.0
                        if wrist_ctrl and (5 in node_ids) and (6 in node_ids):
                            wrist_ctrl.rotate_pos = (joint_positions[5] + joint_positions[6]) / 2.0
                            wrist_ctrl.bend_pos   = (joint_positions[5] - joint_positions[6]) / 2.0
                        print(f"[INFO] All nodes armed: {sorted(newly_armed)}\\n")
                        print("[SAFETY] Release and re-press L1 before any motion will be accepted.\\n")
                        joystick_states["status"] = f"ARMED: {sorted(newly_armed)} -- release & re-press L1 to move"
                        awaiting_l1_reset = True
                        l1_released_since_arm = False
                arm_triggered_this_hold = True
        else:
            dpad_arm_hold_start = None
            arm_triggered_this_hold = False''',
    '''        # --- Arm gesture: D-pad left + Circle held 1.5s. Arming itself
        # runs on its own thread (ArmingSequence) so this loop is never
        # blocked waiting on it and keeps checking PS-hold every frame no
        # matter how long arming takes.
        if dpad == DPAD_ARM_DIRECTION and circle and not arming_locked_out:
            if dpad_arm_hold_start is None:
                dpad_arm_hold_start = time.time()
            elif (not arm_triggered_this_hold) and (time.time() - dpad_arm_hold_start) >= ARM_HOLD_SECONDS:
                if not arming_seq.is_running():
                    print("\\n[INFO] Arm gesture held - arming all nodes in the background "
                          "(validate, then arm one at a time, verified)...\\n")
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
                print(f"[INFO] All nodes armed: {sorted(armed_ids)}\\n")
                print("[SAFETY] Release and re-press L1 before any motion will be accepted.\\n")
                joystick_states["status"] = f"ARMED: {sorted(armed_ids)} -- release & re-press L1 to move"
                all_armed = True
                awaiting_l1_reset = True
                l1_released_since_arm = False
            else:
                still_armed = result.get("still_armed", [])
                if still_armed:
                    arming_locked_out = True
                    print(f"\\n[CRITICAL] Arming aborted, and node(s) {still_armed} did NOT "
                          f"confirm disarmed. DO NOT operate the arm. Run check_armed.py and "
                          f"resolve this manually, then restart gamecontroller.py - the arm "
                          f"gesture is locked out for the rest of this session.\\n")
                    joystick_states["status"] = (
                        f"!!! DISARM NOT CONFIRMED: {still_armed} - RUN check_armed.py - "
                        f"ARM GESTURE LOCKED OUT, RESTART REQUIRED !!!"
                    )
                else:
                    print(f"[ERROR] Arming failed ({result.get('reason', '?')}) - all nodes confirmed disarmed.\\n")
                    joystick_states["status"] = f"ARMING FAILED ({result.get('reason', '?')}) - all nodes confirmed disarmed"
        was_arming_running = arming_seq.is_running()''',
)

# ---------------------------------------------------------------------------
# EDIT F: PS-hold also aborts arming_seq; resync all_armed after the
# sequence stops running; watchdog for an unexpected mid-motion disarm;
# motion_allowed gated on all_armed
# ---------------------------------------------------------------------------
apply_edit(
    "F: PS-hold aborts arming + resync + watchdog + motion gate",
    '''        if ps:
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
    '''        if ps:
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
            if disarm_check_counter % 3 == 0:
                armed_ep = endpoints['endpoints']['axis0.is_armed']
                dropped = [nid for nid in node_ids
                           if read_config(bus, nid, armed_ep['id'], armed_ep['type']) is not True]
                if dropped:
                    all_armed = False
                    arming_locked_out = True
                    print(f"\\n[CRITICAL] Node(s) {dropped} unexpectedly disarmed during operation! "
                          f"All motion halted - the rest of the arm holds its last position. "
                          f"Restart gamecontroller.py and check hardware (check_armed.py) before "
                          f"continuing.\\n")
                    joystick_states["status"] = (
                        f"!!! UNEXPECTED DISARM: {dropped} - ALL MOTION HALTED - RESTART REQUIRED !!!"
                    )

        motion_allowed = lb
        if not all_armed:
            motion_allowed = False
        if safe_return.is_running():
            motion_allowed = False
        if arming_locked_out:
            motion_allowed = False''',
)

# ---------------------------------------------------------------------------
# EDIT G: main() - create arming_seq, wire into joy_thread and
# clean_shutdown
# ---------------------------------------------------------------------------
apply_edit(
    "G1: create arming_seq",
    '''    safe_return = SafeReturnSequence()''',
    '''    safe_return = SafeReturnSequence()
    arming_seq  = ArmingSequence()''',
)

apply_edit(
    "G2: joy_thread args",
    '''        args = (bus, discovered, joint_positions, shoulder_ctrl, wrist_ctrl, endpoints, safe_return),''',
    '''        args = (bus, discovered, joint_positions, shoulder_ctrl, wrist_ctrl, endpoints, safe_return, arming_seq),''',
)

apply_edit(
    "G3: clean_shutdown call",
    '''        clean_shutdown(discovered, bus, joint_positions, endpoints, safe_return, shoulder_ctrl, wrist_ctrl)''',
    '''        clean_shutdown(discovered, bus, joint_positions, endpoints, safe_return, arming_seq, shoulder_ctrl, wrist_ctrl)''',
)

open(path, "w").write(src)
print(f"\\nAll edits applied. {len(original_src.splitlines())} -> {len(src.splitlines())} lines.")
