#!/usr/bin/env python3
"""
read_config() waits up to 0.1s per attempt, up to 3 attempts, in a
tight loop with no sleep - so a single call can legitimately take up
to 0.3s in the worst case before giving up and returning None. That's
fine for one call, but several places call it in a row for every node
(the watchdog: up to 8 nodes; the metrics panel: 13 fields x 8 nodes) -
if a handful of those hit their worst case back to back, the
individual waits just stack up. 8 nodes x 0.3s = 2.4s, which is almost
exactly the 2.331s frame stall just observed - a very close match for
"several reads each quietly waiting their full timeout in a row," not
one single freeze.

Fix: shorten the per-attempt wait from 0.1s to 0.03s. A healthy CAN
response comes back in single-digit milliseconds, so 30ms is generous
headroom, not a hair trigger. Retries stay at 3 - that resilience to a
genuinely dropped CAN frame (real electrical noise from motor
operation does occasionally cost a frame) is kept; only how long any
one attempt is allowed to drag on before trying again gets shorter.
This is applied to src/configure.py, not gamecontroller.py - a
different file, but the same assert-exact-match safety pattern.
"""

path = "configure.py"
src = open(path).read()
original_src = src

def apply_edit(name, old, new):
    global src
    count = src.count(old)
    assert count == 1, f"[{name}] expected 1 match, found {count} - aborting, nothing written."
    src = src.replace(old, new)
    print(f"[{name}] OK")

apply_edit(
    "read_config per-attempt timeout 0.1s -> 0.03s",
    '''def read_config(bus, node_id, endpoint_id, endpoint_type, timeout=0.1, retries=3):''',
    '''def read_config(bus, node_id, endpoint_id, endpoint_type, timeout=0.03, retries=3):''',
)

open(path, "w").write(src)
print(f"\\nAll edits applied. {len(original_src.splitlines())} -> {len(src.splitlines())} lines.")
