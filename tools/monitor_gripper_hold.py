#!/usr/bin/env python3
"""
Watches node7 (gripper) through a real hold test, hands-free.

Designed so the operator NEVER has to read the terminal while gripping -
Carla's own feedback 2026-09-01: "i cant monitor and operate at the same
time, too many decimal points to watch and i miss the critical numbers".
So this waits for the arm, waits for a REAL grip, times its own hold,
prints one short line every 5s, and gives a plain-English verdict.

Signals recorded:
  pos_estimate / pos_setpoint  - the gap between them IS the position
                                 error driving the proportional term.
                                 With compliant fingers this error never
                                 closes, which is why the integrator
                                 winds continuously under any real grip.
  Iq_measured                  - real current, vs the 1.5A hard trip
  vel_integrator_torque        - integral term; pins at the cap
  active_errors / current_state - 0 and 8 mean clean and still closed-loop

Deliberately does NOT judge fault state from axis0.disarm_reason: that is a
persistent record of the LAST disarm and never clears on a successful
re-arm, so testing it gives false "still faulted" reports forever (real
false alarm, 2026-08-28). Read once at startup as history only.

Read-only: sends no motor commands, writes no config.

GRIP DETECTION (fixed 2026-09-01): the first version triggered on current
alone, which fired on a transient blip before Carla had actually squeezed
and then happily timed 60s of a fully-open gripper and called it "CLEAN".
A hold is now only recognised when the fingers have actually TRAVELLED
away from their armed resting position AND are drawing current - and the
verdict refuses to pass a hold that never took any real load.

Usage: python3 monitor_gripper_hold.py [hold_seconds] [out_file]
"""
import sys, os, time, json
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import can
from src.configure import load_endpoints, read_config

NODE = 7
HARD_TRIP_A = 1.5      # node7 current_hard_max
GRIP_TRAVEL = 0.05     # fingers must move at least this far from rest
GRIP_START_A = 0.25    # ...AND be drawing at least this much current
LOAD_FLOOR_A = 0.20    # a hold quieter than this never really took load
SETTLE_DELTA = 0.004   # pos spread over SETTLE_WINDOW that counts as stopped
SETTLE_WINDOW = 1.0
ARM_TIMEOUT = 300.0
GRIP_TIMEOUT = 300.0
POLL_HZ = 5.0
STATE_NAMES = {1: "IDLE", 3: "CALIBRATION", 8: "CLOSED_LOOP"}

FIELDS = ["axis0.pos_estimate", "axis0.controller.pos_setpoint",
          "axis0.vel_estimate", "axis0.motor.foc.Iq_measured",
          "axis0.controller.vel_integrator_torque",
          "axis0.active_errors", "axis0.current_state"]


def main():
    hold_secs = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    out_file = sys.argv[2] if len(sys.argv) > 2 else "gripper_hold.json"
    period = 1.0 / POLL_HZ

    bus = can.interface.Bus("can0", interface="socketcan")
    ep = load_endpoints()['endpoints']

    def rd(name):
        e = ep[name]
        return read_config(bus, NODE, e['id'], e['type'])

    def snapshot():
        v = [rd(f) for f in FIELDS]
        return None if None in v else v

    # ---- confirm the fix is actually on the node before testing it ----
    pos_gain = rd('axis0.controller.config.pos_gain')
    integ_lim = rd('axis0.controller.config.vel_integrator_limit')
    disarm_hist = rd('axis0.disarm_reason')

    if pos_gain is None or integ_lim is None:
        print("Cannot read node7 - is the arm powered and CAN up? Aborting.")
        bus.shutdown()
        sys.exit(1)

    print("=" * 60)
    print(f"  pos_gain             = {pos_gain}     (must be 13)")
    print(f"  vel_integrator_limit = {integ_lim:.3f}  (must be 0.18)")
    print(f"  disarm_reason        = {disarm_hist}       (history only, ignore)")
    print("=" * 60)
    if abs(pos_gain - 13.0) > 0.01 or abs(integ_lim - 0.18) > 0.001:
        print("\n  *** STOP - config is NOT the validated pair. ***")
        print("  *** Both were flash-saved 2026-09-01, so if either has moved,")
        print("  *** something reset it - find out what before testing. ***\n")
        bus.shutdown()
        sys.exit(2)
    cap = abs(integ_lim)

    # ---- phase 1: wait to be armed ----
    print("\nWaiting for you to arm (L1+Circle)...")
    t0 = time.time()
    while True:
        if rd('axis0.current_state') == 8:
            break
        if time.time() - t0 > ARM_TIMEOUT:
            print("Gave up waiting to be armed.")
            bus.shutdown(); sys.exit(1)
        time.sleep(0.5)

    rest_pos = rd('axis0.pos_estimate')
    print(f"Armed. Fingers resting at {rest_pos:.4f}.")
    print("Squeeze onto the object and hold - don't watch this screen,")
    print("I'll do the watching and tell you when to let go.\n")

    # ---- phase 2: wait for a REAL grip (travel AND current) ----
    t0 = time.time()
    while True:
        snap = snapshot()
        if snap is None:
            time.sleep(period); continue
        pos, sp, vel, iq, integ, errs, st = snap
        if errs != 0 or st != 8:
            print(f"Faulted before the grip started: errors={errs} state={st}")
            bus.shutdown(); sys.exit(1)
        if abs(pos - rest_pos) > GRIP_TRAVEL and abs(iq) > GRIP_START_A:
            break
        if time.time() - t0 > GRIP_TIMEOUT:
            print("No real grip detected - giving up.")
            bus.shutdown(); sys.exit(1)
        time.sleep(period)
    print(f"Grip detected (fingers moved {abs(pos-rest_pos):.3f}) - "
          f"waiting for them to stop moving...")

    # ---- phase 3: wait for the fingers to settle ----
    samples = []
    faulted_at = None
    window = []
    settle_start = time.time()
    while True:
        snap = snapshot()
        if snap is None:
            time.sleep(period); continue
        pos, sp, vel, iq, integ, errs, st = snap
        if errs != 0 or st != 8:
            faulted_at = -1.0
            print("\n" + "!" * 60)
            print(f"  FAULTED WHILE STILL CLOSING - errors={errs} "
                  f"state={STATE_NAMES.get(st, st)}")
            print("!" * 60)
            break
        window.append((time.time(), pos))
        window = [(t, p) for (t, p) in window if time.time() - t <= SETTLE_WINDOW]
        if len(window) >= 4 and (time.time() - window[0][0]) >= SETTLE_WINDOW * 0.9:
            if max(p for _, p in window) - min(p for _, p in window) < SETTLE_DELTA:
                break
        if time.time() - settle_start > 30:
            print("Fingers never settled in 30s - starting the clock anyway.")
            break
        time.sleep(period)

    # ---- phase 4: the timed hold ----
    if faulted_at is None:
        print(f"Holding. Watching for {hold_secs:.0f}s.\n")
        print("     t      err       Iq     integ")
        start = time.time()
        next_report = 0.0
        while True:
            now = time.time()
            elapsed = now - start
            if elapsed > hold_secs:
                break
            snap = snapshot()
            if snap is None:
                time.sleep(period); continue
            pos, sp, vel, iq, integ, errs, st = snap
            samples.append({"t": elapsed, "pos": pos, "sp": sp, "vel": vel,
                            "iq": iq, "integ": integ, "errs": errs, "state": st})
            if errs != 0 or st != 8:
                faulted_at = elapsed
                print("\n" + "!" * 60)
                print(f"  FAULTED at {elapsed:.0f}s - errors={errs} "
                      f"state={STATE_NAMES.get(st, st)}")
                print("!" * 60)
                break
            if elapsed >= next_report:
                at_cap = "  [integ at cap]" if abs(integ) >= cap * 0.98 else ""
                warn = "  <-- CURRENT HIGH" if abs(iq) > 1.2 else ""
                print(f"  {elapsed:5.0f}s {sp-pos:8.4f} {abs(iq):7.2f}A "
                      f"{abs(integ):8.4f}{at_cap}{warn}")
                next_report += 5.0
            sleep_left = period - (time.time() - now)
            if sleep_left > 0:
                time.sleep(sleep_left)

    bus.shutdown()
    print("\n>>> You can let go now. <<<")

    with open(out_file, "w") as f:
        json.dump({"pos_gain": pos_gain, "vel_integrator_limit": integ_lim,
                   "hold_secs": hold_secs, "rest_pos": rest_pos,
                   "faulted_at": faulted_at, "samples": samples}, f, indent=2)

    # ---- verdict ----
    print("\n" + "=" * 60)
    if not samples:
        print("  NO HOLD RECORDED - it faulted before the hold began.")
        print("  That points at the approach into the object, not the hold.")
        print("=" * 60)
        print("\n  NOTE: a fault resets node7. pos_gain is flash-saved as of")
        print("  2026-09-01, so it should come back as 13 - but check.")
        print(f"\n(raw data: {out_file})")
        return

    iqs = [abs(s["iq"]) for s in samples]
    integs = [abs(s["integ"]) for s in samples]
    poss = [s["pos"] for s in samples]
    errsz = [abs(s["sp"] - s["pos"]) for s in samples]
    held = samples[-1]["t"]
    peak_iq = max(iqs)

    if faulted_at is not None:
        print(f"  FAULTED at {faulted_at:.0f}s.")
    elif peak_iq < LOAD_FLOOR_A:
        print(f"  INCONCLUSIVE - held {held:.0f}s without faulting, but peak")
        print(f"  current was only {peak_iq:.2f}A. The gripper never took real")
        print("  load, so this says nothing about whether the fix holds.")
    else:
        print(f"  CLEAN - held {held:.0f}s under real load, no errors.")
    print("=" * 60)
    print(f"  current     : {min(iqs):.2f}A to {peak_iq:.2f}A  "
          f"(trips at {HARD_TRIP_A}A, headroom {HARD_TRIP_A - peak_iq:.2f}A)")
    print(f"  integrator  : {max(integs):.4f} peak   (cap {cap:.2f})"
          f"{'  - pinned at cap' if max(integs) >= cap*0.98 else ''}")
    print(f"  pos error   : {min(errsz):.4f} to {max(errsz):.4f}  "
          "(never reaches 0 with compliant fingers - that's expected)")
    print(f"  finger creep: {poss[-1] - poss[0]:+.4f} over the hold")

    if faulted_at is not None:
        if max(integs) < cap * 0.98:
            print("\n  Integrator never reached its cap, so this was NOT windup -")
            print("  it's the proportional term. pos_gain needs to go below 13.")
        else:
            print("\n  Integrator was pinned at the cap, so the cap held - but")
            print("  total demand still reached the trip. Next lever is pos_gain")
            print("  below 13.")
        print("\n  NOTE: the fault reset node7 - re-check pos_gain reads 13.")
    elif peak_iq >= LOAD_FLOOR_A:
        print("\n  Good result. The integrator marching to its cap under a")
        print("  sustained grip is normal here, not a warning sign: compliant")
        print("  fingers keep flexing, so position error never closes and the")
        print("  integral term winds until the cap stops it. That cap is doing")
        print("  real work every single grip - it is not an edge-case guard.")


if __name__ == "__main__":
    main()
