#!/usr/bin/env python3
"""
GRIPPER_RELEASE_OPEN (used both for the "open on arm" signal and Stage
1 of the return-to-rest sequence) was TRIGGER_MAX (0.80) - and driving
the gripper toward that value has now faulted node 7 twice in a row
with the same signature (disarm_reason=2048, stopping partway). Direct
position tests with check_gripper.py moved it cleanly between roughly
0.5 and 0.66 with no issue, so 0.65 is a value we've actually confirmed
is reachable - this is a conservative placeholder, not a measured true
limit, pending physically checking whether something's blocking it
near full open. Easy to raise again once that's characterized.
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
    "GRIPPER_RELEASE_OPEN -> 0.65 (confirmed-reachable, conservative)",
    '''GRIPPER_RELEASE_OPEN   = TRIGGER_MAX  # full open - guarantees release of
                                        # anything held before any arm motion''',
    '''GRIPPER_RELEASE_OPEN   = 0.65      # NOT full open (TRIGGER_MAX=0.80) - driving
                                    # toward 0.80 faulted node 7 twice in a row
                                    # (disarm_reason=2048, stopping partway).
                                    # 0.65 is a value directly confirmed reachable
                                    # via check_gripper.py - conservative, not a
                                    # measured true limit. Raise again once it's
                                    # been physically checked for an obstruction
                                    # or true end-of-travel near full open.''',
)

open(path, "w").write(src)
print(f"\\nAll edits applied. {len(original_src.splitlines())} -> {len(src.splitlines())} lines.")
