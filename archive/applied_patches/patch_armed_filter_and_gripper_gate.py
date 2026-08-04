#!/usr/bin/env python3
"""
Three fixes, on top of patch_safe_return.py and patch_slow_and_skip.py:

  1. PStap was starting run_safe_return_sequence() with the FULL
     discovered node list, not just the nodes actually armed. Any
     unarmed node included that way (e.g. a gripper that failed the
     rest_pos gate) can never confirm reaching a target, so every
     stage that includes it just burns its full 15s wait_for_position
     timeout doing nothing before moving on. clean_shutdown()'s own
     fallback call already filters to only-armed nodes - PStap's live
     trigger never got the same filter. This is very likely what
     looked like "PStap doesn't do anything."

  2. clean_shutdown() waits up to 90s for an in-progress sequence to
     finish before exiting - but if the sequence is PAUSED (not
     actively running) at that moment, it will never finish on its
     own, so the wait was guaranteed to run out uselessly. Now it
     resumes a paused sequence first, so it actually has a chance to
     complete within the wait window.

  3. The rest_pos arming gate applied the same 0.1 tolerance check to
     node 7 (gripper) as every arm joint. The joints have a real
     mechanical zero (resting against the frame); the gripper doesn't
     - its pos_estimate re-zeros to wherever it physically happens to
     be at boot, so gating its arming on "reads near 0.4" is
     unreliable by construction, not just unlucky. The gripper also
     doesn't carry the "swept through an unknown pose" risk that
     motivated the gate for the arm joints in the first place. Node 7
     now always arms (synced to its live position, same as every other
     node), regardless of this check.

Same safety pattern as before: every block is matched and asserted to
occur exactly once before anything is written.
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
# EDIT A: PStap - filter to only currently-armed nodes before starting
# ---------------------------------------------------------------------------
apply_edit(
    "A: PStap filters to armed nodes only",
    '''        if ps and not ps_prev:
            ps_press_time = time.time()
            if not safe_return.is_running():
                print("\\n>>> PS TAPPED - starting safe-return sequence <<<\\n")
                joystick_states["status"] = "SAFE RETURN: running (all other input ignored)"
                safe_return.start(bus, node_ids, endpoints, shoulder_ctrl, wrist_ctrl, joint_positions)
            else:''',
    '''        if ps and not ps_prev:
            ps_press_time = time.time()
            if not safe_return.is_running():
                armed_ep = endpoints['endpoints']['axis0.is_armed']
                armed_ids = [nid for nid in node_ids
                             if read_config(bus, nid, armed_ep['id'], armed_ep['type'])]
                print(f"\\n>>> PS TAPPED - starting safe-return sequence for armed nodes {armed_ids} <<<\\n")
                joystick_states["status"] = "SAFE RETURN: running (all other input ignored)"
                safe_return.start(bus, armed_ids, endpoints, shoulder_ctrl, wrist_ctrl, joint_positions)
            else:''',
)

# ---------------------------------------------------------------------------
# EDIT B: clean_shutdown - resume a paused sequence before waiting on it
# ---------------------------------------------------------------------------
apply_edit(
    "B: resume-before-wait on exit",
    '''    if safe_return.is_running():
        print("[INFO] Safe-return sequence still active - waiting for it to finish before exiting...")
        safe_return.join(timeout=90)''',
    '''    if safe_return.is_running():
        if safe_return.pause_event.is_set():
            print("[INFO] Safe-return sequence is paused - resuming it so it can finish before exiting...")
            safe_return.pause_event.clear()
        print("[INFO] Safe-return sequence still active - waiting for it to finish before exiting...")
        safe_return.join(timeout=90)''',
)

# ---------------------------------------------------------------------------
# EDIT C: arm gesture - drop the rest_pos gate for node 7 (gripper)
# ---------------------------------------------------------------------------
apply_edit(
    "C: node 7 always arms regardless of rest_pos reading",
    '''                    rest_target = GRIPPER_STARTUP_OPEN if nid == 7 else 0.0
                    if abs(pos - rest_target) > REST_POS_TOLERANCE:
                        not_at_rest.append((nid, round(pos, 4)))
                        continue''',
    '''                    # Node 7 (gripper) is exempt: unlike the arm joints, it has no
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
                            continue''',
)

# All edits applied in memory without error - now write out.
open(path, "w").write(src)
print(f"\\nAll edits applied. {len(original_src.splitlines())} -> {len(src.splitlines())} lines.")
