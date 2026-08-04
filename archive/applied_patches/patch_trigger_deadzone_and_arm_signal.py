#!/usr/bin/env python3
"""
Two independent, small fixes:

  1. Trigger dead zone. apply_dead_zone() was built for the sticks,
     which rest at 0 and move symmetrically to +-1. The triggers don't
     rest at 0 - they rest at -1 (released) and move to +1 (fully
     pressed). Running the stick dead-zone function on a trigger meant
     the first half of the pull, from released to the halfway point,
     produced zero output - the gripper only responded once a trigger
     was pressed more than halfway. A new apply_trigger_dead_zone()
     normalizes -1..1 to 0..1 first (0 = released, 1 = fully pressed),
     then applies a small dead zone near the released end only, so
     response starts almost immediately off rest instead of requiring
     a 50%+ pull. The gripper's own increment math is unchanged - only
     which function computes the trigger's value changes.

  2. Gripper-open-on-arm signal, restored. Once every node finishes
     arming, the gripper is commanded fully open - a clear, visible
     "armed and ready" confirmation, same as before this whole
     redesign. A closed gripper can't grab anything, so this doubles
     as a sane starting state for whatever comes next.
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

# ---------------------------------------------------------------------------
# EDIT A: add apply_trigger_dead_zone() next to apply_dead_zone()
# ---------------------------------------------------------------------------
apply_edit(
    "A: apply_trigger_dead_zone()",
    '''    if abs(value) < DEAD_ZONE:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - DEAD_ZONE) / (1.0 - DEAD_ZONE)

def signal_handler(sig, frame):''',
    '''    if abs(value) < DEAD_ZONE:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - DEAD_ZONE) / (1.0 - DEAD_ZONE)

TRIGGER_DEAD_ZONE = 0.05  # small - just enough to ignore rest-position noise

def apply_trigger_dead_zone(value):
    """
    Triggers rest at -1.0 (released) and move toward +1.0 (fully
    pressed) - not at 0 like the sticks, so apply_dead_zone() (built
    for sticks) doesn't fit them: it was silently requiring a trigger
    to be pressed more than halfway before producing any output at
    all. This normalizes -1..1 to 0..1 (0 = released, 1 = fully
    pressed) first, then applies a small dead zone near the released
    end only, so response starts almost immediately off rest.
    """
    normalized = (value + 1.0) / 2.0
    if normalized < TRIGGER_DEAD_ZONE:
        return 0.0
    return (normalized - TRIGGER_DEAD_ZONE) / (1.0 - TRIGGER_DEAD_ZONE)

def signal_handler(sig, frame):''',
)

# ---------------------------------------------------------------------------
# EDIT B: use the new function for both trigger reads
# ---------------------------------------------------------------------------
apply_edit(
    "B: trigger reads use apply_trigger_dead_zone",
    '''        raw_lt = apply_dead_zone(joystick.get_axis(AXIS_LEFT_TRIGGER))
        raw_rt = apply_dead_zone(joystick.get_axis(AXIS_RIGHT_TRIGGER))''',
    '''        raw_lt = apply_trigger_dead_zone(joystick.get_axis(AXIS_LEFT_TRIGGER))
        raw_rt = apply_trigger_dead_zone(joystick.get_axis(AXIS_RIGHT_TRIGGER))''',
)

# ---------------------------------------------------------------------------
# EDIT C: open the gripper once all nodes finish arming
# ---------------------------------------------------------------------------
apply_edit(
    "C: gripper-open-on-arm signal",
    '''                if wrist_ctrl and (5 in armed_ids) and (6 in armed_ids):
                    wrist_ctrl.rotate_pos = (joint_positions[5] + joint_positions[6]) / 2.0
                    wrist_ctrl.bend_pos   = (joint_positions[5] - joint_positions[6]) / 2.0
                print(f"[INFO] All nodes armed: {sorted(armed_ids)}\\n")''',
    '''                if wrist_ctrl and (5 in armed_ids) and (6 in armed_ids):
                    wrist_ctrl.rotate_pos = (joint_positions[5] + joint_positions[6]) / 2.0
                    wrist_ctrl.bend_pos   = (joint_positions[5] - joint_positions[6]) / 2.0
                if 7 in armed_ids:
                    # Visible "armed and ready" confirmation - a closed
                    # gripper can't grab anything anyway, so full open is
                    # also a sane starting state.
                    move_odrive_to_position(bus, 7, GRIPPER_RELEASE_OPEN)
                    joint_positions[7] = GRIPPER_RELEASE_OPEN
                print(f"[INFO] All nodes armed: {sorted(armed_ids)}\\n")''',
)

open(path, "w").write(src)
print(f"\\nAll edits applied. {len(original_src.splitlines())} -> {len(src.splitlines())} lines.")
