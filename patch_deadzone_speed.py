#!/usr/bin/env python3
"""
Two previously-deferred, low-risk improvements:
  - apply_dead_zone() now rescales the remaining stick travel above the
    threshold instead of passing the raw value straight through, so
    there's an actual fine/slow-creep range instead of a jump from 0
    straight to ~DEAD_ZONE the instant the stick leaves center.
  - 10% across-the-board speed reduction (VELOCITY_SCALING,
    FOREARM_VELOCITY_SCALING), since the structure is flexing
    noticeably at current speeds.
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
    "1: 10% speed reduction",
    '''VELOCITY_SCALING         = 1.5
FOREARM_VELOCITY_SCALING = 2.0''',
    '''VELOCITY_SCALING         = 1.35  # was 1.5 - 10% across-the-board reduction,
                                        # structure was flexing noticeably at the
                                        # old speed
FOREARM_VELOCITY_SCALING = 1.8   # was 2.0 - same 10% reduction''',
)

apply_edit(
    "2: dead-zone rescale",
    '''def apply_dead_zone(value):
    return 0.0 if abs(value) < DEAD_ZONE else value''',
    '''def apply_dead_zone(value):
    """
    Below DEAD_ZONE, returns 0. Above it, rescales the remaining travel
    back onto the full -1..1 range, so the first bit of stick motion
    past the dead zone doesn't jump straight to a value of ~DEAD_ZONE -
    previously there was no fine/slow-creep range at all, just a jump
    from 0 to a moderate speed the instant the stick left center.
    """
    if abs(value) < DEAD_ZONE:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - DEAD_ZONE) / (1.0 - DEAD_ZONE)''',
)

open(path, "w").write(src)
print(f"\nAll edits applied. {len(original_src.splitlines())} -> {len(src.splitlines())} lines.")
