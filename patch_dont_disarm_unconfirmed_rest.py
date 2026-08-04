#!/usr/bin/env python3
"""
Root cause of the "collapsed 1-2 inches above rest_pos" incident: Stage 6
of the return-to-rest sequence commands all joints toward rest_pos, then
calls wait_for_position() to confirm they actually got there - but never
looks at what it returns. wait_for_position() already returns True only
if every node confirmed within tolerance, and False on a timeout (or an
Estop abort) - that return value was just being thrown away, so Stage 7
disarmed unconditionally either way. If the wait ran out before the arm
(now moving slower, from today's earlier speed cap) actually arrived,
Stage 7 disarmed nodes that were still 1-2 inches from rest_pos - instant
loss of holding torque, so that last stretch dropped under gravity. This
is exactly the "disarm while moving = arm falls" case that must never
happen.

Fix: capture wait_for_position()'s return value. If it's False (timeout,
not an Estop abort - that path already returns separately just above),
do NOT disarm. The nodes are still armed and still actively holding /
moving toward the commanded rest position under their own control loop -
nothing needs to be re-sent - so the safe move is to leave them alone and
print a clear warning instead of silently dropping them. No new loop,
just one added check before the existing disarm step.
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
    "A: capture wait_for_position's return value at Stage 6",
    '''        wait_for_position(bus, node_ids, endpoints, rest_targets,
                           pause_event=pause_event, abort_event=abort_event)
        for nid in node_ids:
            joint_positions[nid] = rest_targets[nid]''',
    '''        reached_rest = wait_for_position(bus, node_ids, endpoints, rest_targets,
                           pause_event=pause_event, abort_event=abort_event)
        for nid in node_ids:
            joint_positions[nid] = rest_targets[nid]''',
)

apply_edit(
    "B: refuse to disarm unless rest_pos was actually confirmed",
    '''    if aborted():
        print("[SAFE_UP] Sequence aborted (Estop) - speed limits restored, no further motion sent.")
        return

    print("[SAFE_UP] Stage 7: disarming...")''',
    '''    if aborted():
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

    print("[SAFE_UP] Stage 7: disarming...")''',
)

open(path, "w").write(src)
print(f"\nAll edits applied. {len(original_src.splitlines())} -> {len(src.splitlines())} lines.")
