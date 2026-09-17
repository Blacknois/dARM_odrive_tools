#!/usr/bin/env python3
"""odrive_temps.py - FET and motor temperature of every node, read-only.
ODrive CAN 'Get_Temperature' (cmd 0x15) via an RTR request; the reply is two
float32: fet_temperature, motor_temperature (deg C). Sends no commands."""
import can, struct, sys, time
NODES = range(8); CMD = 0x15
bus = can.interface.Bus("can0", interface="socketcan")
try:
    for nid in NODES:
        bus.send(can.Message(arbitration_id=(nid << 5) | CMD, is_remote_frame=True, is_extended_id=False, dlc=8))
        fet = mot = None; t0 = time.time()
        while time.time() - t0 < 0.3:
            m = bus.recv(0.05)
            if m is None or m.is_remote_frame: continue
            if m.arbitration_id == ((nid << 5) | CMD) and len(m.data) >= 8:
                fet, mot = struct.unpack("<ff", m.data[:8]); break
        print("node %d  FET %s  motor %s" % (nid, "%.1f C" % fet if fet is not None else "no reply", "%.1f C" % mot if mot is not None else "no reply"))
finally:
    bus.shutdown()
