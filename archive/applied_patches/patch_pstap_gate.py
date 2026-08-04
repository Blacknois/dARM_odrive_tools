#!/usr/bin/env python3
"""
PS-tap could start the return-to-rest sequence using whatever nodes
happened to read as armed at that moment, with no check on whether the
system was in a locked-out or frozen state, or whether arming was
still actively in progress in the background. That's wrong two ways:

  1. If the mid-motion watchdog had already frozen everything because
     one node dropped, the other nodes are still genuinely armed - so
     a PS-tap would start automated motion on them, defeating the
     freeze entirely.
  2. If arming was still running in the background, PS-tap could start
     the return sequence on whatever had armed so far, running at the
     same time arming was still trying to arm the rest - two things
     issuing motion commands at once, the same kind of collision that
     caused the original wrist damage this whole redesign exists to
     prevent.

Fix: starting a NEW return-to-rest run now requires the single
all_armed flag - confirmed fully armed, nothing in progress, nothing
locked out. Pausing/resuming a sequence that's already running is
unaffected, since that never starts anything new. Otherwise, PS-tap is
simply ignored - it does not touch or clear whatever alert is already
on the status line.
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
    "PS-tap gated on all_armed",
    '''        if ps and not ps_prev:
            ps_press_time = time.time()
            if not safe_return.is_running():
                armed_ep = endpoints['endpoints']['axis0.is_armed']
                armed_ids = [nid for nid in node_ids
                             if read_config(bus, nid, armed_ep['id'], armed_ep['type'])]
                print(f"\\n>>> PS TAPPED - starting safe-return sequence for armed nodes {armed_ids} <<<\\n")
                joystick_states["status"] = "SAFE RETURN: running (all other input ignored)"
                safe_return.start(bus, armed_ids, endpoints, shoulder_ctrl, wrist_ctrl, joint_positions)
            else:
                safe_return.toggle_pause()
                joystick_states["status"] = ("SAFE RETURN: paused - tap PS to resume"
                                              if safe_return.pause_event.is_set()
                                              else "SAFE RETURN: running (all other input ignored)")''',
    '''        if ps and not ps_prev:
            ps_press_time = time.time()
            if safe_return.is_running():
                safe_return.toggle_pause()
                joystick_states["status"] = ("SAFE RETURN: paused - tap PS to resume"
                                              if safe_return.pause_event.is_set()
                                              else "SAFE RETURN: running (all other input ignored)")
            elif all_armed:
                armed_ep = endpoints['endpoints']['axis0.is_armed']
                armed_ids = [nid for nid in node_ids
                             if read_config(bus, nid, armed_ep['id'], armed_ep['type'])]
                print(f"\\n>>> PS TAPPED - starting safe-return sequence for armed nodes {armed_ids} <<<\\n")
                joystick_states["status"] = "SAFE RETURN: running (all other input ignored)"
                safe_return.start(bus, armed_ids, endpoints, shoulder_ctrl, wrist_ctrl, joint_positions)
            else:
                # Not fully armed, arming still in progress, or locked
                # out - PS-tap does nothing. Deliberately does not touch
                # joystick_states["status"], so whatever alert is already
                # showing (frozen/locked-out/arming) stays visible.
                print("\\n[INFO] PS tapped but ignored - not in a fully-armed, ready state.\\n")''',
)

open(path, "w").write(src)
print(f"\\nAll edits applied. {len(original_src.splitlines())} -> {len(src.splitlines())} lines.")
