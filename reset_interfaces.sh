#!/bin/bash

echo "Resetting CAN interface can0..."
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
echo ""

echo "Restarting bluetooth service..."
sudo systemctl restart bluetooth
echo ""

echo "Reinitialize udev..."
sudo udevadm trigger
echo ""