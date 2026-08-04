"""
Set node 3 (elbow roll) to its final measured clamp, replacing the
temporary widened-for-testing value.

Measured 2026-07-31 (after widening past the old too-tight clamp):
  wire limit: min=-10.14, max=12.72
  Final clamp pulls in 0.14 on each side as a margin (wire wasn't
  pulled fully taut during measurement): min=-10.00, max=12.58

Run on the Pi from ~/robot/dARM/odrive_tools/.
"""

path = "gamecontroller.py"

with open(path, "r") as f:
    content = f.read()

old = "JOINT2_MIN, JOINT2_MAX = -15.0,  20.0  # TEMP WIDENED for limit testing 2026-07-31 (was -8.38, 12.89)"
new = "JOINT2_MIN, JOINT2_MAX = -10.00,  12.58  # Elbow roll - measured 2026-07-31 (wire limit -10.14/12.72, 0.14 margin each side)"

if old not in content:
    raise SystemExit("Could not find the expected temp-widened JOINT2_MIN/MAX line - aborting, no changes made.")

content = content.replace(old, new)

with open(path, "w") as f:
    f.write(content)

print("Patched: JOINT2 (elbow roll, node 3) set to final clamp -10.00 / 12.58.")
