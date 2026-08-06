#!/usr/bin/env python3
"""
Read-only CAN health diagnostic: per-node message broadcast rates and
bus fault counters (n_restarts, error, n_rx, effective baudrate). No
writes, no arming, no motion.
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # so this can find src/ when run from tools/
import can
from src.configure import load_endpoints, read_config

NODE_IDS = [0, 1, 2, 3, 4, 5, 6, 7]

RATE_FIELDS = [
    ("axis0.config.can.heartbeat_msg_rate_ms",   "heartbeat_ms"),
    ("axis0.config.can.encoder_msg_rate_ms",     "encoder_ms"),
    ("axis0.config.can.iq_msg_rate_ms",          "iq_ms"),
    ("axis0.config.can.torques_msg_rate_ms",     "torques_ms"),
    ("axis0.config.can.powers_msg_rate_ms",      "powers_ms"),
    ("axis0.config.can.bus_voltage_msg_rate_ms", "busv_ms"),
    ("axis0.config.can.temperature_msg_rate_ms", "temp_ms"),
    ("axis0.config.can.version_msg_rate_ms",     "version_ms"),
    ("axis0.config.can.error_msg_rate_ms",       "error_ms"),
]

FAULT_FIELDS = [
    ("can.n_restarts",         "n_restarts"),
    ("can.error",              "can_error"),
    ("can.n_rx",               "n_rx"),
    ("can.effective_baudrate", "baudrate"),
]

def main():
    bus = can.interface.Bus("can0", interface="socketcan")
    endpoints = load_endpoints()['endpoints']

    print("=== Per-node configured CAN broadcast rates (ms; lower = more traffic) ===")
    header = f"{'Node':<6}" + "".join(f"{label:<13}" for _, label in RATE_FIELDS)
    print(header)
    print("-" * len(header))
    for nid in NODE_IDS:
        row = f"{nid:<6}"
        for path, label in RATE_FIELDS:
            ep = endpoints.get(path)
            val = read_config(bus, nid, ep['id'], ep['type']) if ep else None
            row += f"{str(val):<13}"
        print(row)

    print("\n=== Per-node CAN bus fault counters ===")
    header2 = f"{'Node':<6}" + "".join(f"{label:<14}" for _, label in FAULT_FIELDS)
    print(header2)
    print("-" * len(header2))
    for nid in NODE_IDS:
        row = f"{nid:<6}"
        for path, label in FAULT_FIELDS:
            ep = endpoints.get(path)
            val = read_config(bus, nid, ep['id'], ep['type']) if ep else None
            row += f"{str(val):<14}"
        print(row)

    print("\nDone. No commands were sent to any node.")
    bus.shutdown()

if __name__ == "__main__":
    main()
