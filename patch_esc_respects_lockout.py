#!/usr/bin/env python3
"""
Found while checking what ESC does in a locked-out/frozen state: it
was NOT respected. clean_shutdown()'s exit-time fallback sweeps for
any node that still reads armed and, if it finds one, runs the full
automated return-to-rest sequence on it - with no check for whether
the system was locked out. If the watchdog had frozen things because
one node dropped, the OTHER nodes are still genuinely armed, so ESC
would have started automated coordinated motion on them anyway -
exactly the same class of bug as the PS-tap gap just fixed, and
exactly what "the only way out is PS-hold or the power switch" is
supposed to prevent.

Fix requires `arming_locked_out` to be visible from clean_shutdown(),
which runs in a different scope than the joystick thread that sets it
- so it's converted from a local bool to a threading.Event, the same
pattern already used for every other piece of state shared across
threads in this file (safe_return's and arming_seq's abort_event).
clean_shutdown() now checks it first and, if set, does NOT run the
automated sequence at all - it just leaves whatever's still armed
exactly as it is (an ODrive in closed-loop control holds its last
position on its own, with no further Python involvement needed) and
tells you to check hardware manually before next power-on.
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
# EDIT A: clean_shutdown signature + early lockout check
# ---------------------------------------------------------------------------
apply_edit(
    "A: clean_shutdown checks lockout_event first",
    '''def clean_shutdown(node_ids, bus, joint_positions, endpoints, safe_return, arming_seq, shoulder_ctrl, wrist_ctrl):''',
    '''def clean_shutdown(node_ids, bus, joint_positions, endpoints, safe_return, arming_seq, lockout_event, shoulder_ctrl, wrist_ctrl):''',
)

apply_edit(
    "A2: lockout check body",
    '''    if arming_seq.is_running():
        print("[INFO] Arming still in progress - aborting it before exit...")
        arming_seq.abort()
        if arming_seq.thread is not None:
            arming_seq.thread.join(timeout=15)

    if safe_return.is_running():''',
    '''    if arming_seq.is_running():
        print("[INFO] Arming still in progress - aborting it before exit...")
        arming_seq.abort()
        if arming_seq.thread is not None:
            arming_seq.thread.join(timeout=15)

    if lockout_event.is_set():
        print("[CRITICAL] Exiting while locked out (an unconfirmed disarm or an unexpected "
              "node dropout was detected and never resolved). NOT running the automated "
              "return sequence - it is not safe to assume coordinated motion is OK when we "
              "don't know why a node dropped. Whatever is still armed will hold its last "
              "position on its own until power is cut - it does not need Python running to "
              "do that. Use check_armed.py to see exactly what's armed, then resolve manually "
              "before next power-on.")
        if bus:
            bus.shutdown()
        return

    if safe_return.is_running():''',
)

# ---------------------------------------------------------------------------
# EDIT B: joystick_thread_func signature - accept lockout_event
# ---------------------------------------------------------------------------
apply_edit(
    "B: joystick_thread_func signature",
    '''def joystick_thread_func(
    bus, node_ids, joint_positions,
    shoulder_ctrl, wrist_ctrl, endpoints, safe_return, arming_seq,
    update_rate = UPDATE_RATE
):''',
    '''def joystick_thread_func(
    bus, node_ids, joint_positions,
    shoulder_ctrl, wrist_ctrl, endpoints, safe_return, arming_seq, lockout_event,
    update_rate = UPDATE_RATE
):''',
)

# ---------------------------------------------------------------------------
# EDIT C: drop the local arming_locked_out state var - lockout_event
# (passed in) replaces it
# ---------------------------------------------------------------------------
apply_edit(
    "C: remove local arming_locked_out var",
    '''    ps_prev               = False
    ps_press_time         = None
    arming_locked_out     = False
    all_armed             = False''',
    '''    ps_prev               = False
    ps_press_time         = None
    all_armed             = False''',
)

# ---------------------------------------------------------------------------
# EDIT D: the 4 remaining uses of arming_locked_out -> lockout_event
# ---------------------------------------------------------------------------
apply_edit(
    "D1: gesture gate",
    '''        if dpad == DPAD_ARM_DIRECTION and circle and not arming_locked_out:''',
    '''        if dpad == DPAD_ARM_DIRECTION and circle and not lockout_event.is_set():''',
)

apply_edit(
    "D2: arming-result lockout set",
    '''                still_armed = result.get("still_armed", [])
                if still_armed:
                    arming_locked_out = True
                    print(f"\\n[CRITICAL] Arming aborted, and node(s) {still_armed} did NOT ''',
    '''                still_armed = result.get("still_armed", [])
                if still_armed:
                    lockout_event.set()
                    print(f"\\n[CRITICAL] Arming aborted, and node(s) {still_armed} did NOT ''',
)

apply_edit(
    "D3: watchdog lockout set",
    '''                if dropped:
                    all_armed = False
                    arming_locked_out = True
                    print(f"\\n[CRITICAL] Node(s) {dropped} unexpectedly disarmed during operation! ''',
    '''                if dropped:
                    all_armed = False
                    lockout_event.set()
                    print(f"\\n[CRITICAL] Node(s) {dropped} unexpectedly disarmed during operation! ''',
)

apply_edit(
    "D4: motion_allowed gate",
    '''        if arming_locked_out:
            motion_allowed = False''',
    '''        if lockout_event.is_set():
            motion_allowed = False''',
)

# ---------------------------------------------------------------------------
# EDIT E: main() - create lockout_event, wire into joy_thread and
# clean_shutdown
# ---------------------------------------------------------------------------
apply_edit(
    "E1: create lockout_event",
    '''    arming_seq  = ArmingSequence()''',
    '''    arming_seq  = ArmingSequence()
    lockout_event = threading.Event()''',
)

apply_edit(
    "E2: joy_thread args",
    '''        args = (bus, discovered, joint_positions, shoulder_ctrl, wrist_ctrl, endpoints, safe_return, arming_seq),''',
    '''        args = (bus, discovered, joint_positions, shoulder_ctrl, wrist_ctrl, endpoints, safe_return, arming_seq, lockout_event),''',
)

apply_edit(
    "E3: clean_shutdown call",
    '''        clean_shutdown(discovered, bus, joint_positions, endpoints, safe_return, arming_seq, shoulder_ctrl, wrist_ctrl)''',
    '''        clean_shutdown(discovered, bus, joint_positions, endpoints, safe_return, arming_seq, lockout_event, shoulder_ctrl, wrist_ctrl)''',
)

open(path, "w").write(src)
print(f"\\nAll edits applied. {len(original_src.splitlines())} -> {len(src.splitlines())} lines.")
