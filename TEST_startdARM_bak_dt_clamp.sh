#!/bin/bash
# TEST-ONLY wrapper for the code-bisection test. Launches the July 30
# (pre-big-code-jump) version of gamecontroller.py to test whether the
# CAN freeze bug predates or postdates that point in the code history.
# Does NOT touch gamecontroller.py, startdARM, or preflight_startdARM.sh.

set -u
cd ~/robot/dARM/odrive_tools || { echo "[TEST preflight] Could not cd to odrive_tools"; exit 1; }
source .venv/bin/activate

LOGFILE="/tmp/dARM_TEST_bak_dt_clamp_last_run.log"

echo "[TEST preflight] Checking can0 state..."
if ! ip -details link show can0 2>/dev/null | grep -q "ERROR-ACTIVE"; then
    echo "[TEST preflight] can0 not healthy - resetting interfaces..."
    bash reset_interfaces.sh
    sleep 2
fi

echo "[TEST preflight] Launching TEST_gamecontroller_bak_dt_clamp.py (July 30 version)..."
script -q -c "python3 TEST_gamecontroller_bak_dt_clamp.py" "$LOGFILE"

echo "[TEST preflight] Done."
