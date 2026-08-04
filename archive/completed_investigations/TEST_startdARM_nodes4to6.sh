#!/bin/bash
# TEST-ONLY wrapper. Node 7 physically pulled off the CAN daisy chain
# for this test - only nodes 4-6 should be present/discovered. Launches
# TEST_gamecontroller_nodes4to6.py, an untouched byte-identical copy of
# gamecontroller.py (verified via diff at creation time). Does NOT touch
# startdARM, preflight_startdARM.sh, or gamecontroller.py.
# Uses the real Bluetooth controller - same as normal operation.

set -u
cd ~/robot/dARM/odrive_tools || { echo "[TEST preflight 4-6] Could not cd to odrive_tools"; exit 1; }
source .venv/bin/activate

MAX_RETRIES=5
RETRY_DELAY=5
LOGFILE="/tmp/dARM_TEST_nodes4to6_last_run.log"

attempt=1
while [ "$attempt" -le "$MAX_RETRIES" ]; do
    echo "[TEST preflight 4-6] Checking can0 state..."
    if ! ip -details link show can0 2>/dev/null | grep -q "ERROR-ACTIVE"; then
        echo "[TEST preflight 4-6] can0 not healthy - resetting interfaces..."
        bash reset_interfaces.sh
        sleep 2
    fi

    echo "[TEST preflight 4-6] Launching TEST_gamecontroller_nodes4to6.py (attempt $attempt/$MAX_RETRIES)..."
    script -q -c "python3 TEST_gamecontroller_nodes4to6.py" "$LOGFILE"

    # Same HARD SAFETY OVERRIDE as the real preflight script.
    if grep -qE "confirmed armed|ARMED:" "$LOGFILE"; then
        echo "[TEST preflight 4-6] Arming occurred during this run - handing back to you. No auto-retry."
        exit 0
    fi

    if grep -q "No ODrives found on the CAN bus" "$LOGFILE"; then
        echo "[TEST preflight 4-6] No ODrives found - will reset and retry ($attempt/$MAX_RETRIES)."
    elif grep -q "No joystick found" "$LOGFILE"; then
        echo "[TEST preflight 4-6] No joystick found - waiting a moment and retrying ($attempt/$MAX_RETRIES)."
    else
        echo "[TEST preflight 4-6] Exited - handing back to you (not an auto-retry case)."
        exit 0
    fi

    attempt=$((attempt + 1))
    sleep "$RETRY_DELAY"
done

echo "[TEST preflight 4-6] Gave up after $MAX_RETRIES attempts - check hardware manually."
exit 1
