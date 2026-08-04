#!/usr/bin/env python3
"""
Read-only. Captures a full health snapshot - per-node ODrive electrical
readings, fault codes, firmware/hardware versions, CAN bus interface
stats, Pi system load, and disk space - and appends it (timestamped) to
baseline_log.txt. Doesn't arm or move anything. Meant to be re-run
anytime (idle, at rest, robot powered on) to build a history for
comparing "is anything drifting/degrading over time" against today's
known-good baseline.

Deliberately NOT included: CPU temperature (no evidence of thermal
throttling, not worth the added complexity right now) and encoder
calibration-validity flags (calibration gets redone periodically anyway,
so a snapshot of it isn't a meaningful drift indicator the way voltage,
errors, or fault codes are).
"""
import subprocess
import time
import can
from src.configure import load_endpoints, read_config
from src.can_utils import discover_node_ids
from src.metrics import get_metrics

def run(cmd):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5).stdout.strip()
    except Exception as e:
        return f"[error running {cmd}: {e}]"

def main():
    lines = []
    lines.append(f"\n{'='*70}")
    lines.append(f"BASELINE SNAPSHOT - {time.strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"{'='*70}")

    bus = can.interface.Bus("can0", interface="socketcan")
    try:
        node_ids = sorted(discover_node_ids(bus))
        endpoints = load_endpoints()
        ep = endpoints["endpoints"]
        active_err_ep = ep["axis0.active_errors"]
        disarm_ep = ep["axis0.disarm_reason"]
        fw_maj_ep, fw_min_ep, fw_rev_ep = ep["fw_version_major"], ep["fw_version_minor"], ep["fw_version_revision"]
        hw_maj_ep, hw_min_ep, hw_var_ep = ep["hw_version_major"], ep["hw_version_minor"], ep["hw_version_variant"]

        lines.append("\n--- Per-node ODrive readings ---")
        lines.append(f"{'node':>4} {'volts':>7} {'amps':>7} {'bus_amps':>9} {'armed':>6} "
                      f"{'active_err':>10} {'disarm_reason':>13} {'fw':>8} {'hw':>8}")
        for nid in node_ids:
            m = get_metrics(bus, nid, endpoints)
            active_err = read_config(bus, nid, active_err_ep['id'], active_err_ep['type'])
            disarm = read_config(bus, nid, disarm_ep['id'], disarm_ep['type'])
            fw = (read_config(bus, nid, fw_maj_ep['id'], fw_maj_ep['type']),
                  read_config(bus, nid, fw_min_ep['id'], fw_min_ep['type']),
                  read_config(bus, nid, fw_rev_ep['id'], fw_rev_ep['type']))
            hw = (read_config(bus, nid, hw_maj_ep['id'], hw_maj_ep['type']),
                  read_config(bus, nid, hw_min_ep['id'], hw_min_ep['type']),
                  read_config(bus, nid, hw_var_ep['id'], hw_var_ep['type']))
            fw_str = ".".join(str(x) for x in fw)
            hw_str = ".".join(str(x) for x in hw)
            lines.append(
                f"{nid:>4} {m.get('volts'):>7} {m.get('amps'):>7} {m.get('bus_amps'):>9} "
                f"{m.get('armed'):>6} {active_err:>10} {disarm:>13} {fw_str:>8} {hw_str:>8}"
            )
    finally:
        bus.shutdown()

    lines.append("\n--- CAN interface (can0) stats ---")
    lines.append(run("ip -details -statistics link show can0"))

    lines.append("\n--- Pi system load ---")
    lines.append(run("uptime"))
    lines.append(run("free -h"))

    lines.append("\n--- Disk space ---")
    lines.append(run("df -h /"))

    output = "\n".join(str(l) for l in lines)
    print(output)
    with open("baseline_log.txt", "a") as f:
        f.write(output + "\n")

if __name__ == "__main__":
    main()
