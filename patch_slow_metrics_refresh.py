#!/usr/bin/env python3
"""
The metrics panel refreshes 10 times a second, and each refresh calls
get_metrics() for all 8 nodes - 13 individual active-poll CAN reads
per node, per src/metrics.py - so roughly 1,040 individual CAN
round-trips every second, continuously, the whole time gamecontroller.py
runs. This is unrelated to any of today's safety changes (arming,
disarm, the watchdog) and has been running at this rate all along -
but it's very likely the dominant contributor to the CAN bus
congestion behind today's frame stalls and the slower PS-hold response,
since it's constant background load competing for the bus at all
times, including during an emergency stop.

Slowing it to ~3 times a second (0.1s -> 0.3s) cuts that load to
roughly a third, with no functional risk - this thread only ever
reads for display, it never sends anything that moves or arms/disarms
anything, so this cannot affect the safety-relevant behavior. It's a
display value, not a safety value, and does not need to keep pace
with the 30Hz control loop.
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
    "Slow metrics panel refresh from 10Hz to ~3Hz",
    '''        time.sleep(0.1)
        try:
            loop.draw_screen()
        except (urwid.ExitMainLoop, RuntimeError):
            break''',
    '''        # 0.3s (~3Hz) instead of 0.1s (10Hz) - this loop is pure display,
        # calling get_metrics() for all 8 nodes each pass (13 individual
        # CAN reads per node = ~1,040 round-trips/sec at 10Hz). Slowing it
        # cuts that standing CAN load roughly 3x with no functional risk -
        # this thread never sends anything that moves, arms, or disarms
        # anything, so it can't affect safety-relevant behavior.
        time.sleep(0.3)
        try:
            loop.draw_screen()
        except (urwid.ExitMainLoop, RuntimeError):
            break''',
)

open(path, "w").write(src)
print(f"\\nAll edits applied. {len(original_src.splitlines())} -> {len(src.splitlines())} lines.")
