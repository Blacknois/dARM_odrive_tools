# dARM ODrive Tools

Control software for the dARM robot arm — an 8-node ODrive S1 arm (base,
shoulder x2, elbow-roll, elbow, wrist x2, gripper) driven over CAN from a
Raspberry Pi, controlled with a DualSense (PS5) controller. Originally
forked from [JesseDarr/dARM_odrive_tools](https://github.com/JesseDarr/dARM_odrive_tools).

## Running the real robot

```
cd odrive_tools
source .venv/bin/activate
bash preflight_startdARM.sh
```

This checks the CAN interface is healthy, launches `gamecontroller.py`, and
retries automatically on a few known "not ready yet" conditions (no
ODrives found yet, no joystick found yet) - but it never retries once
arming has actually happened, since the arm may be holding a position and
an automated relaunch on top of that isn't safe.

Nodes stay disarmed until you perform the Dpad-Left + Circle gesture with
the controller in hand, and arming is refused unless the arm is physically
at `rest_pos`. See `gamecontroller.py`'s own safety logic (all-or-nothing
arming, coordinated safe-return sequence, watchdog-based lockout on an
unexpected disarm) for the full behavior.

**The real robot must only ever be driven through `gamecontroller.py`'s own
safety system - never a second, parallel one.**

## Project structure

```
odrive_tools/
├── gamecontroller.py           - the real control loop (never edit directly -
│                                  copy to a TEST_-prefixed file first)
├── preflight_startdARM.sh      - the real launcher (same rule as above)
├── reset_interfaces.sh         - resets can0 + bluetooth (software-only, safe)
├── start_dashboard.sh          - launches the live position-stream dashboard
├── console.py                  - accepted direct diagnostic/calibration TUI
│                                  (separate from gamecontroller.py, fine for
│                                  isolated CAN/arm testing)
├── setup.py                    - applies data/config.py to all ODrives
│                                  (broad commissioning script - prefer the
│                                  narrower tools/ scripts for one-off changes)
├── backup_darm.sh              - snapshots this folder to ../backups/
├── patch_watchdog_ambiguous_timeout.py
│                                - a real, NOT-YET-APPLIED fix (see Known
│                                  issues below) - not historical, don't
│                                  archive this one
├── src/                         - real library code, imported by
│                                  gamecontroller.py etc.
├── data/                        - config.py + flat_endpoints.json, the
│                                  real config used by the running system
├── tools/                       - diagnostic/maintenance scripts for
│                                  inspecting or fixing individual ODrives
│                                  (check_*, diag_*, calibrate.py,
│                                  pos_stream_server.py, watch_limits.py, etc.)
└── archive/
    ├── applied_patches/         - one-time patch scripts already merged
    │                              into gamecontroller.py/src - historical
    │                              record, not meant to be re-run
    ├── completed_investigations/ - TEST_-prefixed scripts from finished
    │                              investigations (BT interference ruled
    │                              out, CAN node-isolation bisection, a
    │                              code-history bisection)
    └── abandoned/                - dead/superseded files kept for reference
```

`../backups/` (one level up, alongside `odrive_tools/`) holds full
timestamped snapshots from `backup_darm.sh`, plus loose `.bak` files moved
out of the working tree.

## History: the August 2026 CAN bus-off investigation

The arm had a real, intermittent CAN bus fault that caused it to freeze and
lock out mid-operation - not a code bug, a genuine electrical fault. Several
days of investigation (node isolation bisection, wire swaps, firmware
version audits, USB-direct diagnostics) consistently implicated node 4
(elbow). The actual root cause turned out to be **stale/drifted motor and
encoder calibration on node 4** - not a bad board, not the firmware version,
not the cable. A forced recalibration (triggered incidentally by reflashing
nodes 2/4/7 to standardize firmware versions) resolved it, confirmed by a
sustained real armed-motion test with zero CAN errors.

Along the way, node 7 (gripper) was also found misconfigured to
hall-encoder mode instead of the correct onboard-encoder mode used by the
rest of the fleet - a real config mistake introduced by the same reflash,
unrelated to the main fault but worth knowing about if it ever recurs.

Also discovered: this hardware has no absolute multi-turn encoders, so
`pos_estimate`'s "zero" reference is just wherever the arm physically is at
power-on, not a persistent calibrated value. **The arm must be powered on
while physically at the correct `rest_pos` posture every time**, or
joint-limit safety logic (which assumes a correct zero) could let a joint
drive past its real mechanical limit without knowing it.

## Known issues

- **`patch_watchdog_ambiguous_timeout.py` is a real fix, not yet applied.**
  The watchdog block in `joystick_thread_func` (`gamecontroller.py`, the
  `is not True` / `is not False` checks) treats a CAN read timeout (`None`)
  identically to a confirmed disarm (`False`). This is the correct/expected
  reaction to a node genuinely not responding, not an independent bug - but
  it's worth applying properly at some point.
- Wrist bend/rotate limits are still being characterized - the wrist
  differential appears to be a swashplate/coupled mechanism (tilt magnitude
  + direction from both node 5 and node 6 together), not independent
  bend/twist axes, which the current `BEND_MIN/MAX`/`ROTATE_MIN/MAX` clamps
  may not correctly model.
- Bluetooth controller has an intermittent `Reason.Local` disconnect
  (Pi-side, not the controller) - low priority, not CAN-fault-related.

## Setup

Requires Python 3.6+. Dependencies are pinned in the `.venv` in this repo
(recreate with `python3 -m venv .venv && pip install -r requirements.txt`
if needed - key packages: `python-can`, `pygame`, `urwid`, `odrive`).
