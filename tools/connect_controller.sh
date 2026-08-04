#!/bin/bash
MAC="48:18:8D:F9:24:50"

is_connected() {
    bluetoothctl info "$MAC" | grep -q "Connected: yes"
}

echo "Checking controller connection..."
if is_connected; then
    echo "Already connected."
    exit 0
fi

echo "Not connected. Attempting connect..."
bluetoothctl connect "$MAC" > /dev/null 2>&1
sleep 2

if is_connected; then
    echo "Connected."
    exit 0
fi

echo "First attempt failed. Restarting bluetooth service and retrying..."
sudo systemctl restart bluetooth
sleep 3
bluetoothctl connect "$MAC" > /dev/null 2>&1
sleep 2

if is_connected; then
    echo "Connected after bluetooth restart."
    exit 0
fi

echo ""
echo "Still not connected - this usually means it needs a full re-pair"
echo "(e.g. if it was recently used with another computer)."
echo "Put it in pairing mode (hold PS + Create until light flashes white), then run:"
echo "  bluetoothctl remove $MAC"
echo "  bluetoothctl"
echo "    scan on"
echo "    pair $MAC"
echo "    trust $MAC"
echo "    exit"
exit 1
