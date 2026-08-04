import re

FILE = "gamecontroller.py"
BACKUP = "gamecontroller.py.bak_distinct_retry_timeouts"

with open(FILE) as f:
    content = f.read()

replacements = [
    (
        'def arm_node_verified(bus, node_id, endpoints, retries=5, settle_timeout=0.3,\n'
        '                       confirm_window=0.2, confirm_interval=0.05):',
        '# 2026-08-01: settle_timeout changed from 0.3 to 0.22 (distinct from\n'
        '# disarm_verified/write_verified below) so each function\'s worst-case\n'
        '# blocking time is numerically distinguishable in "Frame stall detected"\n'
        '# warnings - lets us tell which retry loop actually caused a given stall.\n'
        'def arm_node_verified(bus, node_id, endpoints, retries=5, settle_timeout=0.22,\n'
        '                       confirm_window=0.2, confirm_interval=0.05):'
    ),
    (
        'def disarm_verified(bus, node_id, endpoints, retries=5, settle_timeout=0.3):',
        '# 2026-08-01: settle_timeout changed from 0.3 to 0.34 (distinct from\n'
        '# arm_node_verified/write_verified) - same reasoning, see arm_node_verified.\n'
        'def disarm_verified(bus, node_id, endpoints, retries=5, settle_timeout=0.34):'
    ),
    (
        'def write_verified(bus, node_id, endpoint_id, endpoint_type, value, label="",\n'
        '                    retries=5, settle_timeout=0.3, tolerance=1e-3):',
        '# 2026-08-01: settle_timeout changed from 0.3 to 0.46 (distinct from\n'
        '# arm_node_verified/disarm_verified) - same reasoning, see arm_node_verified.\n'
        'def write_verified(bus, node_id, endpoint_id, endpoint_type, value, label="",\n'
        '                    retries=5, settle_timeout=0.46, tolerance=1e-3):'
    ),
]

for old, new in replacements:
    count = content.count(old)
    if count != 1:
        raise SystemExit(f"[ERROR] Expected exactly 1 match, found {count}, for:\n{old[:80]}...")
    content = content.replace(old, new)

with open(BACKUP, "w") as f:
    f.write(open(FILE).read())

with open(FILE, "w") as f:
    f.write(content)

print(f"[OK] Patched successfully. Backup saved to {BACKUP}")
