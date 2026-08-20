# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Control software for the dARM robot arm: an 8-node ODrive S1 arm (base,
shoulder x2, elbow-roll, elbow, wrist x2, gripper) driven over CAN from a
Raspberry Pi, controlled with a DualSense (PS5) controller. This is real
hardware control code for a physical robot with no safety-net
simulator - there is no test suite, and mistakes can break hardware or
cause unintended motion. Originally forked from
[JesseDarr/dARM_odrive_tools](https://github.com/JesseDarr/dARM_odrive_tools);
this fork is `Blacknois/dARM_odrive_tools`, developed independently since.

This repo (`odrive_tools/`) is a git submodule of a larger project at
`~/dARM/` (also containing CAD, BOM, ROS2 package, pictures/videos - not
relevant to this software).

## Running the real robot

```
cd odrive_tools
source .venv/bin/activate
bash preflight_startdARM.sh    # or the shell alias: startdARM
```

`preflight_startdARM.sh` checks CAN health, launches `gamecontroller.py`,
and retries on a few known "not ready yet" conditions (no ODrives found,
no joystick found) - but never retries once arming has actually occurred,
since the arm may be holding a position.

Nodes stay disarmed until the Dpad-Left + Circle gesture is held on the
controller, and arming is refused unless the arm is physically at
`rest_pos`.

## Critical rules for this codebase

- **Never edit `gamecontroller.py`, `preflight_startdARM.sh`, or
  `reset_interfaces.sh` directly.** Always copy to a `TEST_`-prefixed
  filename first, edit the copy, verify with `python3 -m py_compile`. This
  is a hard rule from the project owner, not a style preference.
- **The real robot must only ever be driven through `gamecontroller.py`'s
  own safety system** - never a second, parallel control path.
  `console.py` is a separate, pre-existing, already-accepted direct
  diagnostic/calibration TUI (fine for isolated CAN/arm testing).
  `darm_visualizer.html` + `pos_stream_server.py` are read-only/visualization
  only and never send commands to the real robot.
- **No absolute multi-turn encoders on this hardware.** `pos_estimate`'s
  "zero" reference is just wherever the arm physically is at power-on, not
  a persistent calibrated value. The arm must be powered on while
  physically at the correct `rest_pos` posture every time, or joint-limit
  logic (which assumes a correct zero) could let a joint drive past its
  real mechanical limit without knowing it.
- **Physical LED color is ground truth for arm state, not console text**:
  blue = idle/disarmed, green (flashing) = armed/closed-loop. The codebase's
  own caution (refusing to assume disarmed when a CAN read can't confirm
  it) is intentional and correct - don't "fix" it into being more
  optimistic.
- Never let the arm disarm away from `rest_pos` if avoidable - it can fall
  or damage itself. Use the existing coordinated safe-return sequence
  (`SafeReturnSequence` / PStap gesture in `gamecontroller.py`), don't
  invent a new independent return-to-rest path.
- `patch_watchdog_ambiguous_timeout.py` (top-level, not archived) is a
  real, confirmed, **not-yet-applied** fix - the `is not True`/`is not
  False` ambiguity it addresses is still live in `gamecontroller.py`'s
  watchdog block. Don't mistake it for historical just because it lives
  alongside applied patches conceptually.

## Architecture

- **`gamecontroller.py`** - the real control loop. All-or-nothing arming
  (`run_arming_sequence`/`ArmingSequence`), per-node verified arm/disarm
  (`arm_node_verified`, `disarm_verified` - confirm state actually latched,
  not just that the CAN send succeeded), a watchdog that detects an
  unexpected disarm mid-operation and locks out further motion rather than
  guessing it's safe to continue, and a staged coordinated
  `SafeReturnSequence` (always via a known-safe `safe_up_pos` waypoint
  before folding to `rest_pos` - a direct all-at-once return previously
  swept the arm through the table and sheared the wrist assembly).
  Everything is written to gate correctly on whichever nodes are actually
  discovered on the bus - no node ID is hardcoded as required, so testing
  with a subset of nodes physically present works without code changes.
- **`src/`** - the real library code: `can_utils.py` (raw CAN send/recv,
  node discovery), `configure.py` (`read_config`/`write_config`/
  `save_config`, endpoint-based get/set over CAN), `control.py`,
  `metrics.py`. Imported by `gamecontroller.py` and everything in `tools/`.
- **`data/`** - `config.py` (per-node tuning) and `flat_endpoints.json`
  (the CAN endpoint name -> numeric ID table). This table is generated
  per ODrive firmware build - if a node's firmware version diverges from
  what this file was generated against, CAN reads/writes could silently
  target the wrong endpoint.
- **`tools/`** - diagnostic/maintenance scripts (one node or one concern
  each: `check_*`, `diag_*`, `calibrate.py`, `pos_stream_server.py`,
  `watch_limits.py`, etc.). These live one directory deeper than `src/`,
  so each has a `sys.path` bootstrap inserted right after its docstring/
  shebang to find `src/` regardless of how it's invoked - don't remove
  that when editing one of these files.
- **`archive/`** - `applied_patches/` (one-time patches already merged
  into `gamecontroller.py`/`src/`, historical record, not meant to be
  re-run), `completed_investigations/` (`TEST_`-prefixed scripts from
  finished investigations), `abandoned/` (dead/superseded files).

## Web research standards

When researching hardware/library behavior (e.g. controller/SDL quirks) for
this project, prefer primary sources - manufacturer docs, or an upstream
open-source project's own issue tracker/source (e.g. libsdl-org/SDL's own
issues for an SDL bug) - over third-party blogs/forum posts repeating a
claim secondhand. Multiple sources only count as real corroboration if
they're independently reporting the same thing - check whether "multiple
sources" actually all trace back to one original (possibly wrong) claim
before treating it as verified.

## Git workflow note

`origin/main` on GitHub still reflects an older history line (predates
this fork's independent development) and this repo's local history was
deliberately squashed/migrated away from it at some point. Recent work has
been pushed as **separate feature branches**, not force-pushed to `main` -
follow that pattern (new branch, push, don't touch `main`) unless
explicitly asked to reconcile the two histories.

## Known open issues

- Wrist bend/rotate: the mechanism appears to be a coupled
  swashplate-style differential (tilt magnitude + direction from node 5
  and node 6 together), not independent bend/twist axes - the current
  `BEND_MIN/MAX`/`ROTATE_MIN/MAX` clamps in `gamecontroller.py` may not
  correctly model the real constraint shape in (node5, node6) space.
- Bluetooth controller has an intermittent `Reason.Local` disconnect
  (Pi-side, not the controller) - low priority.
