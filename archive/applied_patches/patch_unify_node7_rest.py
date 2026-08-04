#!/usr/bin/env python3
"""
Node 7 (gripper) used to be special-cased out of the rest_pos checks,
because its pos_estimate wasn't a stable reference across power cycles.
Traced that today to a loose encoder board mount - fixed by tightening
it and recalibrating, then directly confirmed stable across a real
power-off/power-on cycle (0.0 at fully open, before and after).

Since fully open now reads ~0.0 - the same "rest" convention every other
node already uses - the special-casing is no longer needed and is
removed outright here (not patched around): the GRIPPER_STARTUP_OPEN
constant, the separate target in the already-at-rest check, the
gripper-specific rest_targets override in the return sequence, and the
"nid != 7" exemption in arming validation. Node 7 now goes through the
exact same rest_pos logic as every other joint.

Gripper min/max travel limits still need to be properly remeasured
separately - not part of this patch.
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

apply_edit(
    "A: remove GRIPPER_STARTUP_OPEN constant",
    '''ARM_HOLD_SECONDS        = 1.5
DISARM_HOLD_SECONDS     = 1.0
GRIPPER_STARTUP_OPEN    = 0.4

AXIS_LEFT_X       = 0  # Left stick horizontal''',
    '''ARM_HOLD_SECONDS        = 1.5
DISARM_HOLD_SECONDS     = 1.0

AXIS_LEFT_X       = 0  # Left stick horizontal''',
)

apply_edit(
    "B: update rest_pos convention comment",
    '''# ------------------------------------------------------------------------
# Named positions (####_pos convention). rest_pos is the folded stow
# pose the arm physically sits in when powered on/off (target 0.0 for
# every node except the gripper, which rests at GRIPPER_STARTUP_OPEN).
# safe_up_pos is a known-safe vertical/untwisted waypoint used before''',
    '''# ------------------------------------------------------------------------
# Named positions (####_pos convention). rest_pos is the folded stow
# pose the arm physically sits in when powered on/off (target 0.0 for
# every node, including the gripper - now that its encoder mount is
# tightened and recalibrated, 0.0 is a stable, repeatable reference
# for it too, same as every other joint).
# safe_up_pos is a known-safe vertical/untwisted waypoint used before''',
)

apply_edit(
    "C: already-at-rest check uses 0.0 for every node",
    '''    pos_ep = endpoints['endpoints']['axis0.pos_estimate']
    already_there = True
    for nid in node_ids:
        pos = read_config(bus, nid, pos_ep['id'], pos_ep['type'])
        target = GRIPPER_STARTUP_OPEN if nid == 7 else 0.0
        if pos is None or abs(pos - target) > REST_POS_TOLERANCE:
            already_there = False
            break''',
    '''    pos_ep = endpoints['endpoints']['axis0.pos_estimate']
    already_there = True
    for nid in node_ids:
        pos = read_config(bus, nid, pos_ep['id'], pos_ep['type'])
        if pos is None or abs(pos - 0.0) > REST_POS_TOLERANCE:
            already_there = False
            break''',
)

apply_edit(
    "D: Stage 6 fold-down targets every node to 0.0, no gripper override",
    '''    if not aborted():
        print("[SAFE_UP] Stage 6: folding down to rest_pos...")
        rest_targets = {nid: 0.0 for nid in node_ids}
        if 7 in node_ids:
            rest_targets[7] = GRIPPER_STARTUP_OPEN
        for nid in node_ids:
            move_odrive_to_position(bus, nid, rest_targets[nid])''',
    '''    if not aborted():
        print("[SAFE_UP] Stage 6: folding down to rest_pos...")
        rest_targets = {nid: 0.0 for nid in node_ids}
        for nid in node_ids:
            move_odrive_to_position(bus, nid, rest_targets[nid])''',
)

apply_edit(
    "E: remove nid != 7 exemption from arming rest_pos validation",
    '''        positions[nid] = pos
        # Node 7 (gripper) is exempt from the rest_pos check: unlike the
        # arm joints, it has no real mechanical zero to check pos_estimate
        # against (it re-zeros to wherever it physically sits at boot,
        # not a fixed reference), and it doesn't carry the "swept through
        # an unknown pose" risk that motivated this gate for the joints.
        if nid != 7 and abs(pos - 0.0) > REST_POS_TOLERANCE:
            failures.append((nid, f"not at rest_pos ({round(pos, 4)})"))''',
    '''        positions[nid] = pos
        # Node 7 (gripper) used to be exempt here since its pos_estimate
        # wasn't a stable reference across power cycles - traced to a
        # loose encoder board mount, now fixed and recalibrated
        # (2026-07-30), so it gets the same check as every other node.
        if abs(pos - 0.0) > REST_POS_TOLERANCE:
            failures.append((nid, f"not at rest_pos ({round(pos, 4)})"))''',
)

open(path, "w").write(src)
print(f"\nAll edits applied. {len(original_src.splitlines())} -> {len(src.splitlines())} lines.")
