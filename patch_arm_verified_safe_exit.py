"""
Fix: arm_node_verified() can give up and return False after a node
"reported armed but did not stay armed" on every retry - but that check
can be triggered by a flaky read, not a real drop (we've already seen
CAN reads glitch elsewhere in this project). The retry loop never sends
an explicit disarm command, so if the node actually WAS armed the whole
time and only the READS were unreliable, the function returns False
having never disarmed it. The caller only disarms nodes it believes
succeeded, so a node in this state gets skipped from cleanup entirely -
this is exactly what happened to node 7 on 2026-07-31.

This patch makes arm_node_verified() force a verified disarm before
giving up, so its contract actually holds: returning False always means
the node is confirmed safe, not just "we're not sure."

Run on the Pi from ~/robot/dARM/odrive_tools/.
"""

path = "gamecontroller.py"

with open(path, "r") as f:
    content = f.read()

old = '''    disarm_reason = read_config(bus, node_id, disarm_ep['id'], disarm_ep['type'])
    active_errors = read_config(bus, node_id, errors_ep['id'], errors_ep['type'])
    print(f"[ERROR] Node {node_id} did not confirm armed after {retries} attempts "
          f"(disarm_reason={disarm_reason}, active_errors={active_errors})")
    return False'''

new = '''    disarm_reason = read_config(bus, node_id, disarm_ep['id'], disarm_ep['type'])
    active_errors = read_config(bus, node_id, errors_ep['id'], errors_ep['type'])
    print(f"[ERROR] Node {node_id} did not confirm armed after {retries} attempts "
          f"(disarm_reason={disarm_reason}, active_errors={active_errors})")
    # Giving up here does NOT guarantee the node is actually disarmed - a
    # "did not stay armed" retry above can be triggered by a flaky read
    # rather than a real drop, meaning the node could still be genuinely
    # armed even though every retry looked like a failure. Force a
    # verified disarm before returning, so this function's contract
    # holds: returning False always means the node is confirmed safe,
    # never just "we're not sure." (2026-07-31: node 7 was left
    # physically armed after this function gave up, because nothing
    # here ever actually asked it to disarm.)
    if not disarm_verified(bus, node_id, endpoints):
        print(f"[CRITICAL] Node {node_id} could not be confirmed disarmed after "
              f"giving up on arming - DO NOT operate the arm. Check manually "
              f"(check_armed.py).")
    return False'''

if old not in content:
    raise SystemExit("Could not find the expected arm_node_verified ending - aborting, no changes made.")

content = content.replace(old, new)

with open(path, "w") as f:
    f.write(content)

print("Patched: arm_node_verified() now forces a verified disarm before giving up.")
