#!/usr/bin/env python3
"""
The watchdog acted on a single failed read - during today's test, a
0.724s frame stall (CAN congestion) caused reads for nodes 2 and 4 to
time out for one check, which the fail-safe "treat unknown as not
armed" logic took as a real drop and froze everything, even though
both nodes were actually still fine (confirmed still armed moments
later). Two changes:

  1. Check half as often (every 6th frame instead of every 3rd, so
     roughly every 200ms instead of 100ms) - less CAN traffic from the
     watchdog itself, one less contributor to the kind of congestion
     that caused the stall in the first place.

  2. A node only counts as dropped if it fails on two checks in a row
     - specifically the INTERSECTION of this check's failures and the
     previous check's failures, not just "failed once." A real drop
     shows the same node failing repeatedly; CAN noise tends to hit
     different nodes at random each time, so requiring the same node
     to fail twice running filters that out while still catching a
     genuine drop within about a quarter second - still a single flat
     check, no nested retry loop.
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
    "A: watchdog state var",
    '''    disarm_check_counter  = 0''',
    '''    disarm_check_counter  = 0
    watchdog_pending_dropped = set()''',
)

apply_edit(
    "B: debounced, half-rate watchdog check",
    '''        if all_armed and not safe_return.is_running():
            disarm_check_counter += 1
            if disarm_check_counter % 3 == 0:
                armed_ep = endpoints['endpoints']['axis0.is_armed']
                dropped = [nid for nid in node_ids
                           if read_config(bus, nid, armed_ep['id'], armed_ep['type']) is not True]
                if dropped:
                    all_armed = False
                    lockout_event.set()
                    print(f"\\n[CRITICAL] Node(s) {dropped} unexpectedly disarmed during operation! "
                          f"All motion halted - the rest of the arm holds its last position. "
                          f"Restart gamecontroller.py and check hardware (check_armed.py) before "
                          f"continuing.\\n")
                    joystick_states["status"] = (
                        f"!!! UNEXPECTED DISARM: {dropped} - ALL MOTION HALTED - RESTART REQUIRED !!!"
                    )''',
    '''        if all_armed and not safe_return.is_running():
            disarm_check_counter += 1
            if disarm_check_counter % 6 == 0:
                armed_ep = endpoints['endpoints']['axis0.is_armed']
                dropped_now = set(nid for nid in node_ids
                                   if read_config(bus, nid, armed_ep['id'], armed_ep['type']) is not True)
                confirmed = dropped_now & watchdog_pending_dropped
                watchdog_pending_dropped = dropped_now
                if confirmed:
                    dropped = sorted(confirmed)
                    all_armed = False
                    lockout_event.set()
                    print(f"\\n[CRITICAL] Node(s) {dropped} unexpectedly disarmed during operation! "
                          f"All motion halted - the rest of the arm holds its last position. "
                          f"Restart gamecontroller.py and check hardware (check_armed.py) before "
                          f"continuing.\\n")
                    joystick_states["status"] = (
                        f"!!! UNEXPECTED DISARM: {dropped} - ALL MOTION HALTED - RESTART REQUIRED !!!"
                    )
        else:
            watchdog_pending_dropped = set()''',
)

open(path, "w").write(src)
print(f"\\nAll edits applied. {len(original_src.splitlines())} -> {len(src.splitlines())} lines.")
