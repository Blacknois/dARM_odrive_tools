#!/usr/bin/env python3
"""
Two fixes to run_safe_return_sequence(), on top of the already-applied
patch_safe_return.py:

  1. Skip-if-already-at-rest: before touching anything, check live
     pos_estimate against the rest_pos targets. If every node is
     already within REST_POS_TOLERANCE, there is nowhere to "return"
     from - skip the whole safe_up_pos staged dance and just disarm.
     This is what should have happened during the incident where PStap
     ran the full sequence even though the arm was already at rest_pos.

  2. Absolute speed cap: halving whatever trap_traj.vel/accel/decel
     happen to be currently configured is not a safe bound by itself -
     if the underlying limit is already fast, half of it is still
     fast. The shoulder covered 2.834 rad (rest_pos -> safe_up) in
     about 1 second even with the halved limit, which is too fast for
     an unsupervised-looking automatic move. This adds an absolute
     cap (SAFE_RETURN_VEL_LIMIT / ACCEL / DECEL) that's used instead of
     half the configured value whenever half the configured value
     would exceed it - so the sequence is never faster than this cap,
     regardless of what the node happens to be configured for.

     NOTE: the cap values below (0.4) are a conservative starting
     guess, not a measured safe speed - I have no visibility into the
     actual trap_traj defaults on your hardware from here. Test the
     first run after this patch with a hand on the power switch and
     PS/Estop, watching closely, and adjust SAFE_RETURN_VEL_LIMIT up
     or down from there once you've seen it move at this pace.

Same safety pattern as before: every block is matched and asserted to
occur exactly once before anything is written - a mismatch aborts with
nothing written, instead of silently corrupting the file.
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
# EDIT A: absolute speed-cap constants, next to REST_POS_TOLERANCE
# ---------------------------------------------------------------------------
apply_edit(
    "A: SAFE_RETURN speed-cap constants",
    '''REST_POS_TOLERANCE     = 0.1      # radians - how close counts as "at rest_pos"
                                    # for the arming gate (proxy check only -
                                    # pos_estimate re-zeros at boot regardless
                                    # of true physical position, so this can't
                                    # replace physically verifying the arm is
                                    # at the marked resting pose)''',
    '''REST_POS_TOLERANCE     = 0.1      # radians - how close counts as "at rest_pos"
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
SAFE_RETURN_DECEL_LIMIT = 0.4''',
)

# ---------------------------------------------------------------------------
# EDIT B: halving loop -> halve-or-cap, whichever is slower
# ---------------------------------------------------------------------------
apply_edit(
    "B: halve-or-cap speed limits",
    '''    orig_values = {}
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
        print(f"[WARNING] Speed-limit halve not confirmed for: {failed_halve} - proceeding anyway.")''',
    '''    orig_values = {}
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
        print(f"[WARNING] Speed-limit cap not confirmed for: {failed_halve} - proceeding anyway.")''',
)

# ---------------------------------------------------------------------------
# EDIT C: already_at_rest() helper + short-circuit at the top of
# run_safe_return_sequence, before the speed limits are even touched
# ---------------------------------------------------------------------------
apply_edit(
    "C: already_at_rest() + short-circuit",
    '''    def aborted():
        return abort_event is not None and abort_event.is_set()

    vel_ep   = endpoints['endpoints']['axis0.trap_traj.config.vel_limit']''',
    '''    def aborted():
        return abort_event is not None and abort_event.is_set()

    pos_ep = endpoints['endpoints']['axis0.pos_estimate']
    already_there = True
    for nid in node_ids:
        pos = read_config(bus, nid, pos_ep['id'], pos_ep['type'])
        target = GRIPPER_STARTUP_OPEN if nid == 7 else 0.0
        if pos is None or abs(pos - target) > REST_POS_TOLERANCE:
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

    vel_ep   = endpoints['endpoints']['axis0.trap_traj.config.vel_limit']''',
)

# All edits applied in memory without error - now write out.
open(path, "w").write(src)
print(f"\\nAll edits applied. {len(original_src.splitlines())} -> {len(src.splitlines())} lines.")
