#!/bin/bash
# Pre-flight wrapper for startdARM. Checks/self-heals known "not ready yet"
# conditions, then hands off to gamecontroller.py unmodified. Never retries
# once a real control session has actually started or armed anything.

set -u
cd ~/dARM/odrive_tools || { echo "[preflight] Could not cd to odrive_tools"; exit 1; }
source .venv/bin/activate

MAX_RETRIES=5
RETRY_DELAY=5
LOGFILE="/tmp/dARM_last_run.log"

echo "[preflight] Checking controller battery..."
for cap_file in /sys/class/power_supply/*/capacity; do
    [ -e "$cap_file" ] || continue
    level=$(cat "$cap_file" 2>/dev/null)
    if [ -n "${level:-}" ] && [ "$level" -lt 10 ] 2>/dev/null; then
        echo "[preflight] WARNING: controller battery at ${level}% - consider charging before use."
    fi
done

attempt=1
while [ "$attempt" -le "$MAX_RETRIES" ]; do
    echo "[preflight] Checking can0 state..."
    if ! ip -details link show can0 2>/dev/null | grep -q "ERROR-ACTIVE"; then
        echo "[preflight] can0 not healthy - resetting interfaces..."
        bash reset_interfaces.sh
        sleep 2
    fi

    echo "[preflight] Launching gamecontroller.py (attempt $attempt/$MAX_RETRIES)..."
    script -q -c "python3 gamecontroller.py" "$LOGFILE"

    # HARD SAFETY OVERRIDE: if arming ever happened this run, never retry,
    # regardless of anything else in the log. The arm may still be holding
    # position and a fresh relaunch on top of that is not safe to automate.
    if grep -qE "confirmed armed|ARMED:" "$LOGFILE"; then
        echo "[preflight] Arming occurred during this run - handing back to you. No auto-retry."
        exit 0
    fi

    if grep -q "No ODrives found on the CAN bus" "$LOGFILE"; then
        echo "[preflight] No ODrives found - will reset and retry ($attempt/$MAX_RETRIES)."
    elif grep -q "No joystick found" "$LOGFILE"; then
        echo "[preflight] No joystick found - waiting a moment and retrying ($attempt/$MAX_RETRIES)."
    else
        echo "[preflight] Exited - handing back to you (not an auto-retry case)."
        exit 0
    fi

    attempt=$((attempt + 1))
    sleep "$RETRY_DELAY"
done

echo "[preflight] Gave up after $MAX_RETRIES attempts - check hardware manually (check_armed.py, ip -details link show can0)."
exit 1
