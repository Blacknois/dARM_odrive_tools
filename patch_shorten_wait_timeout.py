#!/usr/bin/env python3
"""
wait_for_position()'s default timeout was 15.0s - how long the
return-to-rest sequence will patiently wait for one move to confirm
before giving up on that stage and continuing. It polls every 0.1s the
whole time and checks Estop every cycle regardless, so it was never a
blind spot - but 15s is still a long time to sit doing nothing if a
stage genuinely stalls, flagged twice now as too long. Cut to 5.0s -
still enough patience for a real, slower-than-usual move (especially
now that speed is intentionally capped), but far less dead time if
something is actually stuck.
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
    "wait_for_position timeout 15.0s -> 5.0s",
    '''def wait_for_position(bus, node_ids, endpoints, targets, tolerance=0.05, timeout=15.0,''',
    '''def wait_for_position(bus, node_ids, endpoints, targets, tolerance=0.05, timeout=5.0,''',
)

open(path, "w").write(src)
print(f"\\nAll edits applied. {len(original_src.splitlines())} -> {len(src.splitlines())} lines.")
