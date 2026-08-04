#!/usr/bin/env python3
"""
Patches TEST_gamecontroller_noBT.py (a full copy of gamecontroller.py) so it
can be run with no physical joystick/Bluetooth controller at all:

  - pygame.joystick is monkeypatched globally to a harmless no-op stub, so
    every call site in the file (including the reconnect logic inside
    joystick_thread_func) gets a safe stand-in instead of touching real
    hardware/Bluetooth.
  - handle_input() gains two keyboard triggers: SPACEBAR arms all nodes
    directly (calls the same arming_seq.start(...) the real Dpad+Circle
    gesture uses), and 'q' triggers PS-tap (calls the same
    safe_return.start(...)/toggle_pause() the real PS button uses).
  - run_safe_return_sequence()'s "already at rest_pos, skip staged sequence"
    shortcut is forced off, so the full staged safe_up_pos -> rest_pos
    sequence always runs for this test, regardless of current position.

Does NOT touch gamecontroller.py, startdARM, or preflight_startdARM.sh.
"""
import sys
import py_compile

TARGET = "TEST_gamecontroller_noBT.py"

replacements = [
    (
        '    pygame.init()\n'
        '    pygame.joystick.init()\n'
        '    if pygame.joystick.get_count() == 0:\n'
        '        print("[ERROR] No joystick found.")\n'
        '        pygame.quit()\n'
        '        return\n'
        '\n'
        '    stick = pygame.joystick.Joystick(0)\n'
        '    stick.init()\n'
        '    print(f"Joystick: {stick.get_name()}")\n'
        '    print(f"# Axes: {stick.get_numaxes()}")\n',

        '    # TEST MODE (noBT): no physical joystick/Bluetooth controller is\n'
        '    # used. pygame.joystick is monkeypatched globally so every call\n'
        '    # site in this file (including the reconnect logic further up in\n'
        '    # joystick_thread_func) gets a harmless no-op stick instead of\n'
        '    # touching real hardware/Bluetooth.\n'
        '    class _NoOpJoystick:\n'
        '        def init(self): pass\n'
        '        def get_hat(self, idx): return (0, 0)\n'
        '        def get_button(self, idx): return False\n'
        '        def get_axis(self, idx): return 0.0\n'
        '        def get_name(self): return "TEST MODE - no physical joystick (noBT)"\n'
        '        def get_numaxes(self): return 6\n'
        '\n'
        '    pygame.joystick.Joystick = lambda idx: _NoOpJoystick()\n'
        '    pygame.joystick.get_count = lambda: 1\n'
        '\n'
        '    pygame.init()\n'
        '    pygame.joystick.init()\n'
        '    if pygame.joystick.get_count() == 0:\n'
        '        print("[ERROR] No joystick found.")\n'
        '        pygame.quit()\n'
        '        return\n'
        '\n'
        '    stick = pygame.joystick.Joystick(0)\n'
        '    stick.init()\n'
        '    print(f"Joystick: {stick.get_name()}")\n'
        '    print(f"# Axes: {stick.get_numaxes()}")\n'
        '    print("\\n[TEST MODE] SPACEBAR = arm all nodes directly. '
        '\'q\' = PS-tap (safe-return).\\n")\n'
    ),
    (
        "def handle_input(key, loop, node_ids, bus, joint_positions):\n"
        "    if key == 'esc':\n"
        "        stop_event.set()\n"
        "        raise urwid.ExitMainLoop()\n",

        "def handle_input(key, loop, node_ids, bus, joint_positions,\n"
        "                  endpoints=None, arming_seq=None, safe_return=None,\n"
        "                  shoulder_ctrl=None, wrist_ctrl=None):\n"
        "    if key == 'esc':\n"
        "        stop_event.set()\n"
        "        raise urwid.ExitMainLoop()\n"
        "    # TEST MODE (noBT) keyboard triggers - call the exact same\n"
        "    # underlying safety-critical functions the real Dpad+Circle\n"
        "    # gesture and PS-tap use, just triggered by keyboard instead of\n"
        "    # a physical controller.\n"
        "    if key == ' ':\n"
        "        if arming_seq is not None and endpoints is not None and not arming_seq.is_running():\n"
        "            print(\"\\n[TEST MODE] SPACEBAR pressed - arming all nodes directly \"\n"
        "                  \"(validate, then arm one at a time, verified)...\\n\")\n"
        "            arming_seq.start(bus, node_ids, endpoints)\n"
        "    if key == 'q':\n"
        "        if safe_return is not None and endpoints is not None:\n"
        "            if safe_return.is_running():\n"
        "                safe_return.toggle_pause()\n"
        "            else:\n"
        "                armed_ep = endpoints['endpoints']['axis0.is_armed']\n"
        "                armed_ids = [nid for nid in node_ids\n"
        "                             if read_config(bus, nid, armed_ep['id'], armed_ep['type'])]\n"
        "                if armed_ids:\n"
        "                    print(f\"\\n[TEST MODE] 'q' PRESSED - starting safe-return sequence \"\n"
        "                          f\"for armed nodes {armed_ids} <<<\\n\")\n"
        "                    safe_return.start(bus, armed_ids, endpoints, shoulder_ctrl, wrist_ctrl, joint_positions)\n"
        "                else:\n"
        "                    print(\"[TEST MODE] 'q' pressed but no nodes are armed - nothing to do.\")\n"
    ),
    (
        "        unhandled_input = lambda k: handle_input(k, loop, discovered, bus, joint_positions)\n",

        "        unhandled_input = lambda k: handle_input(k, loop, discovered, bus, joint_positions,\n"
        "                                                  endpoints, arming_seq, safe_return,\n"
        "                                                  shoulder_ctrl, wrist_ctrl)\n"
    ),
    (
        "    if already_there:\n"
        '        print("[SAFE_UP] Already at rest_pos (within tolerance) - skipping safe_up_pos waypoint, disarming directly.")\n',

        "    # TEST MODE (noBT): always run the full staged sequence, even if\n"
        "    # already at rest_pos - real gamecontroller.py/run_safe_return_sequence\n"
        "    # is unaffected since this is a separate copy of the file.\n"
        "    already_there = False\n"
        "    if already_there:\n"
        '        print("[SAFE_UP] Already at rest_pos (within tolerance) - skipping safe_up_pos waypoint, disarming directly.")\n'
    ),
    (
        '    print("\\n" + "="*70)\n'
        '    print("About to enter live control. Nodes are NOT armed - nothing can")\n'
        '    print("move until you perform Dpad-Left + Circle (held) with the")\n'
        '    print("controller in hand. Arming will be refused unless the arm is")\n'
        '    print("still at rest_pos.")\n'
        '    print("="*70)\n',

        '    print("\\n" + "="*70)\n'
        '    print("[TEST MODE - NO BLUETOOTH] Nodes are NOT armed - nothing can move")\n'
        '    print("until you press SPACEBAR (arms all nodes directly, no hold needed).")\n'
        '    print("Press \'q\' for PS-tap (safe-return). Arming will be refused unless")\n'
        '    print("the arm is still at rest_pos. Press ESC to exit.")\n'
        '    print("="*70)\n'
    ),
]

def main():
    with open(TARGET, "r") as f:
        content = f.read()

    for i, (old, new) in enumerate(replacements, 1):
        count = content.count(old)
        if count == 0:
            print(f"[ERROR] Replacement {i}: exact text not found in {TARGET}. Aborting - no changes written.")
            sys.exit(1)
        if count > 1:
            print(f"[ERROR] Replacement {i}: text matched {count} times (expected 1) - ambiguous. Aborting.")
            sys.exit(1)
        content = content.replace(old, new)

    with open(TARGET, "w") as f:
        f.write(content)

    print(f"Applied {len(replacements)} replacements to {TARGET}.")

    try:
        py_compile.compile(TARGET, doraise=True)
        print("Syntax check: OK")
    except py_compile.PyCompileError as e:
        print(f"[ERROR] Syntax check FAILED: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
