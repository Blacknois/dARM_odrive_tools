#!/usr/bin/env python3
"""
Replaces the per-node arming loop with an all-or-nothing sequence. This
robot cannot operate safely with any single node left disarmed - a
partially-armed arm means at least one joint (or the gripper) has no
holding torque while the rest are live, which is not a safe state to
move in. The previous version armed nodes one at a time and just
printed a warning for whichever ones failed the rest_pos check or
didn't confirm armed, leaving everything that DID succeed live. That's
exactly the partial-arm outcome that must never happen.

New behavior:
  1. Validate every node FIRST, before arming any of them - position
     readable, and (except node 7, the gripper) at rest_pos. If even
     one node fails either check, the whole attempt aborts with
     nothing armed at all.
  2. Only if every node passes validation does it move to actually
     arming them, one at a time, verified. If any single node fails to
     confirm armed, everything armed so far in this pass is
     immediately disarmed again (verified) - so a failure partway
     through can never leave a partial arm live.
  3. After that rollback disarm, every discovered node (not just the
     ones just rolled back) is re-checked via is_armed - if even one
     doesn't confirm disarmed, arming is LOCKED OUT for the rest of
     the process: the arm gesture is refused entirely (not just this
     attempt) until you restart gamecontroller.py, and the status line
     is left showing an unmissable warning telling you to run
     check_armed.py and resolve it manually first. A failed arm that
     also fails to cleanly roll back is a worse state than a normal
     failed attempt and must not be quietly retried.
  4. joint_positions / shoulder_ctrl / wrist_ctrl are only synced to
     the just-armed positions once arming has fully succeeded for
     every node - never on a failed or partial attempt.

Same safety pattern as before: matched and asserted to occur exactly
once before anything is written.
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
# EDIT PRE: add arming_locked_out state, and gate the gesture on it
# ---------------------------------------------------------------------------
apply_edit(
    "PRE: arming_locked_out state var",
    '''    dpad_arm_hold_start   = None
    arm_triggered_this_hold = False
    awaiting_l1_reset     = False
    l1_released_since_arm = False
    ps_prev               = False
    ps_press_time         = None''',
    '''    dpad_arm_hold_start   = None
    arm_triggered_this_hold = False
    awaiting_l1_reset     = False
    l1_released_since_arm = False
    ps_prev               = False
    ps_press_time         = None
    arming_locked_out     = False''',
)

apply_edit(
    "PRE2: gate the arm gesture on arming_locked_out",
    '''        # --- Arm gesture: D-pad left + Circle held 1.5s ---
        if dpad == DPAD_ARM_DIRECTION and circle:''',
    '''        # --- Arm gesture: D-pad left + Circle held 1.5s ---
        if dpad == DPAD_ARM_DIRECTION and circle and not arming_locked_out:''',
)

apply_edit(
    "All-or-nothing arming",
    '''                print("\\n[INFO] Arm gesture held - checking rest_pos and arming...\\n")
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
                    # Node 7 (gripper) is exempt: unlike the arm joints, it has no
                    # real mechanical zero to check pos_estimate against (it
                    # re-zeros to wherever it physically sits at boot, not a
                    # fixed reference), and it doesn't carry the "swept
                    # through an unknown pose" risk that motivated this gate
                    # for the joints - so it always arms, synced to its live
                    # position like every other node.
                    if nid != 7:
                        rest_target = 0.0
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
                if shoulder_ctrl and (1 in node_ids) and (2 in node_ids):
                    shoulder_ctrl.value = (joint_positions[1] - joint_positions[2]) / 2.0
                if wrist_ctrl and (5 in node_ids) and (6 in node_ids):
                    wrist_ctrl.rotate_pos = (joint_positions[5] + joint_positions[6]) / 2.0
                    wrist_ctrl.bend_pos   = (joint_positions[5] - joint_positions[6]) / 2.0
                print(f"[INFO] Re-arm complete: {sorted(newly_armed)}\\n")
                print("[SAFETY] Release and re-press L1 before any motion will be accepted.\\n")
                joystick_states["status"] = f"RE-ARM COMPLETE: {sorted(newly_armed)} -- release & re-press L1 to move"
                awaiting_l1_reset = True
                l1_released_since_arm = False
                arm_triggered_this_hold = True''',
    '''                print("\\n[INFO] Arm gesture held - validating all nodes before arming...\\n")
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
                arm_triggered_this_hold = True''',
)

open(path, "w").write(src)
print(f"\\nAll edits applied. {len(original_src.splitlines())} -> {len(src.splitlines())} lines.")
