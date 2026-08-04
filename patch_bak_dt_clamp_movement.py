#!/usr/bin/env python3
"""
Patches TEST_gamecontroller_bak_dt_clamp.py (July 30 code-bisection copy) to
inject a small, time-boxed synthetic wrist-bend command, so real CAN
movement traffic happens without a physical controller.

This file already auto-arms all discovered nodes inside main() before the
joystick is ever touched -- no arming patch needed.

Injection design (see _NoOpJoystick below):
  - t = 0-3s   : LB+RB held, axis = 0.0 (neutral -- confirms armed+idle first)
  - t = 3-6s   : LB+RB held, axis = 0.35 on AXIS_LEFT_Y (> DEAD_ZONE=0.25) ->
                 wrist_mode active, drives nodes 5/6 (bend) a small, clamped
                 amount via the file's own BEND_MIN/MAX + taper_increment/
                 DECEL_ZONE_WRIST logic.
  - t > 6s     : back to neutral (0.0) -- holds new position, stops moving,
                 so CAN health can be observed after the move too.

Does NOT touch gamecontroller.py, TEST_gamecontroller_bak_dt_clamp.py's
un-patched original behavior beyond this one block, startdARM, or
preflight_startdARM.sh.
"""
import sys
import py_compile

TARGET = "TEST_gamecontroller_bak_dt_clamp.py"

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

        '    # TEST MODE (movement bisection): no physical joystick. Injects a\n'
        '    # small, time-boxed synthetic wrist-bend command (nodes 5/6) so\n'
        '    # real CAN movement traffic occurs. Neutral for first 3s (confirm\n'
        '    # armed+idle), small bend command 3-6s, back to neutral after.\n'
        '    import time as _t\n'
        '    class _NoOpJoystick:\n'
        '        _t0 = _t.time()\n'
        '        def init(self): pass\n'
        '        def get_hat(self, idx): return (0, 0)\n'
        '        def get_button(self, idx):\n'
        '            if idx == DEAD_MAN_BUTTON_INDEX: return True\n'
        '            if idx == MODE_TOGGLE_BUTTON_INDEX: return True\n'
        '            return False\n'
        '        def get_axis(self, idx):\n'
        '            elapsed = _t.time() - _NoOpJoystick._t0\n'
        '            if idx == AXIS_LEFT_Y and 3.0 < elapsed < 6.0:\n'
        '                return 0.35\n'
        '            return 0.0\n'
        '        def get_name(self): return "TEST MODE - synthetic axis injection (bak_dt_clamp movement test)"\n'
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
        '    print("\\n[TEST MODE] Auto-armed by main(). At t=3s a small wrist-bend\\n"\n'
        '          "command (nodes 5/6) will run for ~3s, then return to neutral.\\n")\n'
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
