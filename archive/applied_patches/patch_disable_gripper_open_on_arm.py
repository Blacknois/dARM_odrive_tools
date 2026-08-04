#!/usr/bin/env python3
"""
Node 7 keeps faulting (disarm_reason=2048) partway through this move,
almost every single arm attempt, even after the encoder fix and the
ferrite rings. This was only ever a visual "armed and ready" convenience
signal (closed gripper can't grab anything anyway) - not a safety
requirement. Commenting it out (not deleting) so arming stops forcing
this move while the underlying fault is investigated further. Easy to
re-enable later by uncommenting.
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
    "disable gripper-open-on-arm trigger",
    '''                if 7 in armed_ids:
                    # Visible "armed and ready" confirmation - a closed
                    # gripper can't grab anything anyway, so full open is
                    # also a sane starting state.
                    move_odrive_to_position(bus, 7, GRIPPER_RELEASE_OPEN)
                    joint_positions[7] = GRIPPER_RELEASE_OPEN''',
    '''                # TEMPORARILY DISABLED: node 7 was faulting (disarm_reason=2048)
                # partway through this auto-open move on nearly every arm
                # attempt. Only ever a visual "armed and ready" convenience
                # signal, not a safety requirement - re-enable once the
                # underlying fault is root-caused.
                # if 7 in armed_ids:
                #     move_odrive_to_position(bus, 7, GRIPPER_RELEASE_OPEN)
                #     joint_positions[7] = GRIPPER_RELEASE_OPEN''',
)

open(path, "w").write(src)
print(f"\nAll edits applied. {len(original_src.splitlines())} -> {len(src.splitlines())} lines.")
