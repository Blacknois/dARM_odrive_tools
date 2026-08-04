FILE = "gamecontroller.py"
BACKUP = "gamecontroller.py.bak_watchdog_timeout_fix"

with open(FILE, "r") as f:
    content = f.read()

OLD = '''        if all_armed and not safe_return.is_running():
            disarm_check_counter += 1
            if disarm_check_counter % 6 == 0:
                armed_ep = endpoints['endpoints']['axis0.is_armed']
                dropped_now = set(nid for nid in node_ids
                                   if read_config(bus, nid, armed_ep['id'], armed_ep['type']) is not True)
                confirmed = dropped_now & watchdog_pending_dropped
                watchdog_pending_dropped = dropped_now
                if confirmed:
                    dropped = sorted(confirmed)
                    all_armed = False
                    lockout_event.set()
                    print(f"\\n[CRITICAL] Node(s) {dropped} unexpectedly disarmed during operation! "
                          f"All motion halted - the rest of the arm holds its last position. "
                          f"Restart gamecontroller.py and check hardware (check_armed.py) before "
                          f"continuing.\\n")
                    joystick_states["status"] = (
                        f"!!! UNEXPECTED DISARM: {dropped} - ALL MOTION HALTED - RESTART REQUIRED !!!"
                    )
        else:
            watchdog_pending_dropped = set()'''

NEW = '''        if all_armed and not safe_return.is_running():
            disarm_check_counter += 1
            if disarm_check_counter % 6 == 0:
                armed_ep = endpoints['endpoints']['axis0.is_armed']
                readings = {nid: read_config(bus, nid, armed_ep['id'], armed_ep['type'])
                            for nid in node_ids}
                # 2026-08-01: An explicit False is unambiguous - the node itself
                # reported not armed - so it now acts immediately (faster than
                # before, which waited for a 2nd confirmation even on a clear
                # False). A timeout (None) is ambiguous - real-world testing
                # tonight proved a single timed-out read can happen on a
                # provably healthy, error-free bus - so it still needs to
                # repeat on the next check before being trusted, same as the
                # original debounce behavior.
                confirmed_false = set(nid for nid, v in readings.items() if v is False)
                timed_out = set(nid for nid, v in readings.items() if v is None)
                confirmed_timeout = timed_out & watchdog_pending_dropped
                watchdog_pending_dropped = timed_out
                confirmed = confirmed_false | confirmed_timeout
                if confirmed:
                    dropped = sorted(confirmed)
                    all_armed = False
                    lockout_event.set()
                    print(f"\\n[CRITICAL] Node(s) {dropped} unexpectedly disarmed during operation! "
                          f"All motion halted - the rest of the arm holds its last position. "
                          f"Restart gamecontroller.py and check hardware (check_armed.py) before "
                          f"continuing.\\n")
                    joystick_states["status"] = (
                        f"!!! UNEXPECTED DISARM: {dropped} - ALL MOTION HALTED - RESTART REQUIRED !!!"
                    )
        else:
            watchdog_pending_dropped = set()'''

if OLD not in content:
    print("[ERROR] Could not find the expected watchdog block - file may have changed. No changes made.")
    raise SystemExit(1)

count = content.count(OLD)
if count != 1:
    print(f"[ERROR] Expected exactly 1 match, found {count}. No changes made.")
    raise SystemExit(1)

with open(BACKUP, "w") as f:
    f.write(content)

new_content = content.replace(OLD, NEW)

with open(FILE, "w") as f:
    f.write(new_content)

print(f"[OK] Patched successfully. Backup saved to {BACKUP}")
