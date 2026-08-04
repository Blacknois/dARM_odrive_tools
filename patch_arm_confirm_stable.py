#!/usr/bin/env python3
"""
arm_node_verified() called a node armed the instant is_armed read True
even once, with no check that it actually stayed armed. If a node
briefly reports armed and then drops back out during some internal
settling process, this would have already declared success and moved
on - a plausible explanation for nodes reporting armed at arming time
and then showing up as an unexpected mid-operation disarm shortly
after (the [5, 6] dropout during today's test).

Now, once is_armed first reads True, it has to stay True for a short
confirmation window (0.2s, checked every 0.05s) before this returns
success. If it drops back out during that window, the attempt counts
as failed and retries, same as any other failure - a small, fixed
amount of extra time per node, not an open-ended wait.
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
    "arm_node_verified requires a stable confirmation window",
    '''def arm_node_verified(bus, node_id, endpoints, retries=5, settle_timeout=0.3):
    """
    Requests CLOSED_LOOP_CONTROL and confirms via axis0.is_armed that the
    node actually latched into it, instead of trusting that the CAN send
    alone succeeded.
    """
    armed_ep  = endpoints['endpoints']['axis0.is_armed']
    disarm_ep = endpoints['endpoints']['axis0.disarm_reason']
    errors_ep = endpoints['endpoints']['axis0.active_errors']
    for attempt in range(retries):
        set_closed_loop_control(bus, node_id)
        start = time.time()
        while time.time() - start < settle_timeout:
            armed = read_config(bus, node_id, armed_ep['id'], armed_ep['type'])
            if armed:
                return True
            time.sleep(0.05)
    disarm_reason = read_config(bus, node_id, disarm_ep['id'], disarm_ep['type'])
    active_errors = read_config(bus, node_id, errors_ep['id'], errors_ep['type'])
    print(f"[ERROR] Node {node_id} did not confirm armed after {retries} attempts "
          f"(disarm_reason={disarm_reason}, active_errors={active_errors})")
    return False''',
    '''def arm_node_verified(bus, node_id, endpoints, retries=5, settle_timeout=0.3,
                       confirm_window=0.2, confirm_interval=0.05):
    """
    Requests CLOSED_LOOP_CONTROL and confirms via axis0.is_armed that the
    node actually latched into it, instead of trusting that the CAN send
    alone succeeded. A single True read isn't enough on its own - once
    is_armed first reads True, it has to stay True for confirm_window
    seconds (checked every confirm_interval) before this counts as a
    real success. Catches a node that briefly reports armed and then
    drops back out during some internal settling process, rather than
    only finding out later once motion is already underway.
    """
    armed_ep  = endpoints['endpoints']['axis0.is_armed']
    disarm_ep = endpoints['endpoints']['axis0.disarm_reason']
    errors_ep = endpoints['endpoints']['axis0.active_errors']
    for attempt in range(retries):
        set_closed_loop_control(bus, node_id)
        start = time.time()
        while time.time() - start < settle_timeout:
            armed = read_config(bus, node_id, armed_ep['id'], armed_ep['type'])
            if armed:
                confirm_start = time.time()
                stayed_armed = True
                while time.time() - confirm_start < confirm_window:
                    time.sleep(confirm_interval)
                    still = read_config(bus, node_id, armed_ep['id'], armed_ep['type'])
                    if not still:
                        stayed_armed = False
                        break
                if stayed_armed:
                    return True
                print(f"[WARN] Node {node_id} reported armed but did not stay armed - retrying.")
                break
            time.sleep(0.05)
    disarm_reason = read_config(bus, node_id, disarm_ep['id'], disarm_ep['type'])
    active_errors = read_config(bus, node_id, errors_ep['id'], errors_ep['type'])
    print(f"[ERROR] Node {node_id} did not confirm armed after {retries} attempts "
          f"(disarm_reason={disarm_reason}, active_errors={active_errors})")
    return False''',
)

open(path, "w").write(src)
print(f"\\nAll edits applied. {len(original_src.splitlines())} -> {len(src.splitlines())} lines.")
