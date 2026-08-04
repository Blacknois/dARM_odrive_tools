"""
Temporarily widen the elbow-roll (node 3) soft-clamp so the real
wire/mechanical limit can be explored, same approach used for the wrist
clamps earlier. Run on the Pi from ~/robot/dARM/odrive_tools/.

Old: JOINT2_MIN, JOINT2_MAX = -8.38,  12.89
New: JOINT2_MIN, JOINT2_MAX = -15.0,  20.0   # TEMP WIDENED for limit testing 2026-07-31
"""

path = "gamecontroller.py"

with open(path, "r") as f:
    content = f.read()

old = "JOINT2_MIN, JOINT2_MAX = -8.38,  12.89"
new = "JOINT2_MIN, JOINT2_MAX = -15.0,  20.0  # TEMP WIDENED for limit testing 2026-07-31 (was -8.38, 12.89)"

if old not in content:
    raise SystemExit("Could not find the expected JOINT2_MIN/MAX line - aborting, no changes made.")

content = content.replace(old, new)

with open(path, "w") as f:
    f.write(content)

print("Patched: JOINT2 (elbow roll, node 3) clamp widened to -15.0 / 20.0 (temporary).")
