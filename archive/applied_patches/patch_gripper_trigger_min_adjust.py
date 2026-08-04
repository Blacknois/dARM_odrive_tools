#!/usr/bin/env python3
"""
Live-tested: -0.7 as TRIGGER_MIN (closed) left the gripper noticeably still
open. Moving it to -0.85, closer to the observed ~-0.91/-0.94 hand-closed
position but still leaving a small margin below it. Still a placeholder,
not a measured true hard stop - proper torque-based limit-finding is
separate future work.
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
    "TRIGGER_MIN -0.7 -> -0.85",
    '''TRIGGER_MIN, TRIGGER_MAX = -0.7, 0.0  # placeholder range in the NEW reference''',
    '''TRIGGER_MIN, TRIGGER_MAX = -0.85, 0.0  # placeholder range in the NEW reference''',
)

open(path, "w").write(src)
print(f"\nAll edits applied. {len(original_src.splitlines())} -> {len(src.splitlines())} lines.")
