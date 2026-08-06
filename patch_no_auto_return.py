import py_compile

TARGET = "TEST_gamecontroller_no_auto_return.py"

OLD = """            elif all_armed:
                armed_ep = endpoints['endpoints']['axis0.is_armed']
                armed_ids = [nid for nid in node_ids
                             if read_config(bus, nid, armed_ep['id'], armed_ep['type'])]
                print(f"\\n>>> PS TAPPED - starting safe-return sequence for armed nodes {armed_ids} <<<\\n")
                joystick_states["status"] = "SAFE RETURN: running (all other input ignored)"
                safe_return.start(bus, armed_ids, endpoints, shoulder_ctrl, wrist_ctrl, joint_positions)
"""

NEW = """            elif all_armed:
                # TEST: automated safe-return sequence temporarily disabled
                # while alignment/calibration confidence is being rebuilt.
                # PS-hold force-disarm (below, DISARM_HOLD_SECONDS) is
                # completely untouched and still fully active - this only
                # removes the automated multi-joint return motion.
                print("\\n[INFO] PS tapped - auto safe-return is DISABLED in this "
                      "test build. Hold PS for forced disarm if needed.\\n")
                joystick_states["status"] = "AUTO SAFE-RETURN DISABLED (test build) - PS-hold still force-disarms"
"""

with open(TARGET) as f:
    content = f.read()

if OLD not in content:
    print("ABORT: exact text not found, no changes made.")
    raise SystemExit(1)

if content.count(OLD) != 1:
    print(f"ABORT: found {content.count(OLD)} matches, expected exactly 1. No changes made.")
    raise SystemExit(1)

content = content.replace(OLD, NEW)
with open(TARGET, "w") as f:
    f.write(content)

py_compile.compile(TARGET, doraise=True)
print("Patched and compiled OK.")
