"""
Confirm the wrist (nodes 5/6) clamps as final, replacing the "temporary"
label. The widened values (bend +-8, rotate +-15) were exercised
throughout 2026-07-31 testing without incident, and bend/rotate share
the same two motors so their raw min/max can't be cleanly separated
into independent tight bounds - keeping the tested-safe values rather
than guessing a tighter number from ambiguous combined data.

Run on the Pi from ~/robot/dARM/odrive_tools/.
"""

path = "gamecontroller.py"

with open(path, "r") as f:
    content = f.read()

old_bend = "BEND_MIN,   BEND_MAX     =  -8.0,  8.0 # Wrist - TEMP WIDENED for limit testing 2026-07-31"
new_bend = "BEND_MIN,   BEND_MAX     =  -8.0,  8.0 # Wrist - confirmed final 2026-07-31 (tested throughout, no incident)"

old_rotate = "ROTATE_MIN, ROTATE_MAX   = -15.0, 15.0 # Wrist - TEMP WIDENED for limit testing 2026-07-31"
new_rotate = "ROTATE_MIN, ROTATE_MAX   = -15.0, 15.0 # Wrist - confirmed final 2026-07-31 (tested throughout, no incident)"

if old_bend not in content:
    raise SystemExit("Could not find the expected temp-widened BEND_MIN/MAX line - aborting, no changes made.")
if old_rotate not in content:
    raise SystemExit("Could not find the expected temp-widened ROTATE_MIN/MAX line - aborting, no changes made.")

content = content.replace(old_bend, new_bend)
content = content.replace(old_rotate, new_rotate)

with open(path, "w") as f:
    f.write(content)

print("Patched: wrist clamps (bend +-8, rotate +-15) confirmed as final.")
