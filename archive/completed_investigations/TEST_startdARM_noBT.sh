#!/bin/bash
# TEST-ONLY wrapper. Isolates Bluetooth entirely (rfkill block) to test
# whether the CAN bus faults are related to the shared BT/WiFi radio chip.
# Launches TEST_gamecontroller_noBT.py (keyboard-controlled: SPACEBAR=arm,
# q=PS-tap) instead of the real gamecontroller.py. Does NOT touch startdARM,
# preflight_startdARM.sh, or gamecontroller.py.

set -u
cd ~/robot/dARM/odrive_tools || { echo "[TEST preflight] Could not cd to odrive_tools"; exit 1; }
source .venv/bin/activate

MAX_RETRIES=5
RETRY_DELAY=5
LOGFILE="/tmp/dARM_TEST_noBT_last_run.log"

echo "[TEST preflight] Blocking Bluetooth radio (rfkill) for isolation..."
sudo systemctl stop bluetooth

attempt=1
while [ "$attempt" -le "$MAX_RETRIES" ]; do
    echo "[TEST preflight] Checking can0 state..."
    if ! ip -details link show can0 2>/dev/null | grep -q "ERROR-ACTIVE"; then
        echo "[TEST preflight] can0 not healthy - resetting interfaces..."
        bash reset_interfaces.sh
        sleep 2
    fi

    echo "[TEST preflight] Launching TEST_gamecontroller_noBT.py (attempt $attempt/$MAX_RETRIES)..."
    script -q -c "python3 TEST_gamecontroller_noBT.py" "$LOGFILE"

    # Same HARD SAFETY OVERRIDE as the real preflight script.
    if grep -qE "confirmed armed|ARMED:" "$LOGFILE"; then
        echo "[TEST preflight] Arming occurred during this run - handing back to you. No auto-retry."
        break
    fi

    if grep -q "No ODrives found on the CAN bus" "$LOGFILE"; then
        echo "[TEST preflight] No ODrives found - will reset and retry ($attempt/$MAX_RETRIES)."
    else
        echo "[TEST preflight] Exited - handing back to you (not an auto-retry case)."
        break
    fi

    attempt=$((attempt + 1))
    sleep "$RETRY_DELAY"
done

echo ""
echo "[TEST preflight] Done. Bluetooth is still blocked. To restore normal Bluetooth/controller use:"
echo "    sudo systemctl start bluetooth"
