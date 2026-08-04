"""
Temporarily widen the wrist soft-clamps so the real wire/mechanical limit
can be explored without the placeholder BEND/ROTATE clamps cutting motion
off first. Run on the Pi from ~/robot/dARM/odrive_tools/.

Old:
BEND_MIN,   BEND_MAX     =  -5.0,  5.0 # Wrist
ROTATE_MIN, ROTATE_MAX   = -10.0, 10.0 # Wrist

New (temporary, for limit-finding only):
BEND_MIN,   BEND_MAX     =  -8.0,  8.0 # Wrist - TEMP WIDENED for limit testing 2026-07-31
ROTATE_MIN, ROTATE_MAX   = -15.0, 15.0 # Wrist - TEMP WIDENED for limit testing 2026-07-31
"""
import re

path = "gamecontroller.py"

with open(path, "r") as f:
    content = f.read()

old_bend = "BEND_MIN,   BEND_MAX     =  -5.0,  5.0 # Wrist"
new_bend = "BEND_MIN,   BEND_MAX     =  -8.0,  8.0 # Wrist - TEMP WIDENED for limit testing 2026-07-31"

old_rotate = "ROTATE_MIN, ROTATE_MAX   = -10.0, 10.0 # Wrist"
new_rotate = "ROTATE_MIN, ROTATE_MAX   = -15.0, 15.0 # Wrist - TEMP WIDENED for limit testing 2026-07-31"

if old_bend not in content:
    raise SystemExit("Could not find the BEND_MIN/MAX line - aborting, no changes made.")
if old_rotate not in content:
    raise SystemExit("Could not find the ROTATE_MIN/MAX line - aborting, no changes made.")

content = content.replace(old_bend, new_bend)
content = content.replace(old_rotate, new_rotate)

with open(path, "w") as f:
    f.write(content)

print("Patched: wrist clamps widened to bend +-8, rotate +-15 (temporary).")
