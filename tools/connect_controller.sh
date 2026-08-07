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
echo "Still not connected. Attempting a full automated re-pair of this"
echo "specific controller ($MAC) only - no other device will be touched."
echo "Put it in pairing mode now (hold PS + Create/Share until the light"
echo "flashes rapidly). You have ~10 seconds before scanning starts."
sleep 10

bluetoothctl remove "$MAC" > /dev/null 2>&1

# Trust BEFORE pair: BlueZ auto-authorizes the HID service for devices
# already marked trusted. Trusting after pairing instead means the HID
# authorization request goes to the agent mid-pair with nothing there to
# answer it, and the controller gives up and disconnects on its own.
LOG=$(mktemp)
{
    echo "power on"
    echo "agent on"
    echo "default-agent"
    echo "scan on"
    sleep 20
    echo "scan off"
    echo "trust $MAC"
    sleep 2
    echo "pair $MAC"
    sleep 6
    echo "connect $MAC"
    sleep 3
    echo "quit"
} | bluetoothctl > "$LOG" 2>&1
rm -f "$LOG"

if is_connected; then
    echo "Connected after full re-pair."
    exit 0
fi

echo ""
echo "Automated re-pair failed. Manual steps (trust BEFORE pair - trusting"
echo "first avoids an HID-authorization prompt this script can't answer):"
echo "  bluetoothctl remove $MAC"
echo "  bluetoothctl"
echo "    agent on"
echo "    default-agent"
echo "    scan on"
echo "    trust $MAC"
echo "    pair $MAC"
echo "    connect $MAC"
echo "    quit"
exit 1
