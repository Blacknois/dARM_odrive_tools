#!/usr/bin/env python3
"""
Recalibrating node 7's encoder today didn't just fix the wobble - it also
redefined what pos_estimate=0.0 physically means, since calibration zeros
against wherever the shaft happens to be at that moment (this time, fully
open). The old TRIGGER_MIN/TRIGGER_MAX (0.0 to 0.80) and GRIPPER_RELEASE_OPEN
(0.65) were all tuned against the PREVIOUS reference frame and no longer
correspond to physical reality.

Directly remeasured today, before and after a full power cycle: 0.0 = fully
open (confirmed stable/repeatable), closed is roughly -0.91 to -0.94 by hand
(not a confirmed true hard stop - a torque-based proper limit-finding pass
is separate future work). Setting TRIGGER_MIN to -0.7 as a conservative
pull-back from that, same spirit as the earlier 0.65-vs-0.80 placeholder -
a rough working value, not a measured hard limit.

Trigger mapping direction does NOT need to change: whichever trigger drives
toward TRIGGER_MIN vs TRIGGER_MAX in the existing code keeps doing the same
thing functionally (min = closed, max = open), so this is purely a value
update, not a control-logic change.
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
    "A: TRIGGER_MIN/MAX updated to new reference frame",
    '''TRIGGER_MIN, TRIGGER_MAX =  0.0, 0.80''',
    '''TRIGGER_MIN, TRIGGER_MAX = -0.7, 0.0  # placeholder range in the NEW reference
                                       # frame (post 2026-07-30 recalibration).
                                       # 0.0 = fully open, confirmed stable and
                                       # repeatable across a power cycle. -0.7
                                       # is a conservative pull-back from the
                                       # ~-0.91 to -0.94 hand-closed position -
                                       # NOT a confirmed true hard stop. Proper
                                       # torque-based limit-finding is separate
                                       # future work.''',
)

apply_edit(
    "B: GRIPPER_RELEASE_OPEN updated to new reference frame",
    '''GRIPPER_RELEASE_OPEN   = 0.65      # NOT full open (TRIGGER_MAX=0.80) - driving
                                    # toward 0.80 faulted node 7 twice in a row
                                    # (disarm_reason=2048, stopping partway).
                                    # 0.65 is a value directly confirmed reachable
                                    # via check_gripper.py - conservative, not a
                                    # measured true limit. Raise again once it's
                                    # been physically checked for an obstruction
                                    # or true end-of-travel near full open.''',
    '''GRIPPER_RELEASE_OPEN   = 0.0       # Fully open in the NEW reference frame
                                    # (2026-07-30, after the encoder mount was
                                    # fixed and recalibrated). The old value of
                                    # 0.65 was relative to the pre-recalibration
                                    # frame and no longer corresponds to
                                    # anything physically meaningful.''',
)

open(path, "w").write(src)
print(f"\nAll edits applied. {len(original_src.splitlines())} -> {len(src.splitlines())} lines.")
