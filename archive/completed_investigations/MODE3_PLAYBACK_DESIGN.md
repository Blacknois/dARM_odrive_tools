# Mode 3: File-driven robot playback — design draft

Status: design only, not implemented. Drafted 2026-08-05/06 after finding
the visualizer's live/file render-interlacing bug and the real 1.84 rad
outlier sample in an actual recording. Not yet reviewed/approved by Carla
against her own independent thinking on the same problem.

## Core principle (already decided, non-negotiable)

This is a **separate, dedicated program**, not a feature bolted onto
`darm_visualizer.html`/`pos_stream_server.py`. Those stay strictly
read-only, matching their existing documented invariant. Mode 3 reuses
the same *primitives* `gamecontroller.py` already has verified
(`arm_node_verified`, `disarm_verified`, `move_odrive_to_position`, the
watchdog logic) rather than reinventing them, but is its own top-level
entry point - the same category of thing as `gamecontroller.py` itself:
the one deliberate gateway to real motion, just file-driven instead of
joystick-driven.

## Validation - runs once, up front, before Play is even enabled

All-or-nothing, same philosophy as the arming sequence: one bad sample
blocks the whole file from being played, not just that sample skipped
silently.

Two checks, because neither alone is sufficient - confirmed by real data
from the 2026-08-01 Halloween-adjacent... no, unrelated, the
`darm_recording_1785578977364.json` test file reviewed 2026-08-05:

1. **Kinematic plausibility** - for each consecutive sample pair, is the
   position delta physically achievable given that node's real
   `trap_traj.vel_limit` and the actual recorded time gap
   (`max_travel = vel_limit * dt`)? Catches genuinely impossible jumps
   (the "...4858..." hypothetical case).
2. **Statistical outlier check** - is this jump way outside the *local*
   trend even if kinematically possible? Necessary because the real
   node 5 jump found in the test file (1.84 rad in ~150ms) was actually
   *within* what 30 rad/s could kinematically achieve (~4.5 rad max) -
   a pure kinematic check would NOT have caught it. It was ~23x the
   local average per-sample delta, which a relative/rolling-median-based
   outlier check would catch. **Both checks are required, one does not
   substitute for the other.**

Also validate every sample's absolute position against the real,
authoritative per-node limits from `gamecontroller.py`
(`JOINT0-3_MIN/MAX`, `MOTOR5/6_MIN/MAX`, `TRIGGER_MIN/MAX`) - not the
visualizer's own generic wide slider ranges, which are unrelated to real
safety limits (see separate open item: visualizer editor sliders should
be bounded by these same real constants, so an out-of-range edit can't
even be created in the first place - not yet built, noted 2026-08-05).

## Arming

Same rest_pos-gated arming `gamecontroller.py` already uses
(`arm_node_verified`, must be physically at rest_pos within tolerance).
Never automatic - explicit action only.

## Controls (VCR-style, Carla's initial instinct, refined)

- **Arm / Disarm** - always available regardless of playback state,
  independent buttons, not implicit in play/stop.
- **Step Forward / Step Back** - single-sample advance, sent as a
  normal verified position command. Pauses after each step rather than
  auto-continuing - lets a human verify every single position before
  advancing, maximum-caution mode.
- **Play/Pause** - advances sample-by-sample, but each step *confirms*
  the arm actually reached position (`wait_for_position`-style
  confirmation, tied to real state) before sending the next target -
  NOT the visualizer's blind timer-based approach, which just fires
  commands on a wall-clock schedule regardless of whether the arm
  actually got there.
- **Stop** - deliberately NOT the same as Disarm. Stop halts playback
  advancement but leaves the arm armed, holding its current position
  (same "freeze in place" philosophy as the watchdog) - so stopping
  playback can never itself cause a drop. Disarm remains a separate,
  always-available, deliberate action (same spirit as PS-hold force
  disarm being independent of PS-tap).

## Repeat/looping

Explicit opt-in, not default. Bounded repeat count, not indefinite.
**Directly blocked on an open gap**: see
`darm_speed_load_automation_assumptions.md` - all current thermal data
is from single isolated ~60s holds with real cooldown between tests,
never sustained back-to-back cycling. Looping should not be trusted
until a real multi-cycle back-to-back thermal test exists.

## Failsafe

Reuse the exact same watchdog (freeze-in-place, lockout, manual restart
required) rather than build a parallel mechanism. Any CAN read failure
or bus instability mid-playback pauses immediately rather than retrying
blindly or sending stale data.

## External trigger hook - deferred, not blocking, but captured

Carla's concrete use case (2026-08-06): a "give Halloween candy" motion
gets recorded ahead of time, validated, and then triggered by a remote
button (wired or Bluetooth) each time a trick-or-treater arrives at the
door - not driven from the interactive VCR UI in the moment.

Implies mode 3 eventually needs a trigger layer *on top of* the core
playback engine: something that can fire a specific pre-selected,
pre-validated file's playback from an external signal, separate from
someone sitting at the controls. Not designed yet - explicitly parked
until the rest of mode 3 exists and is trusted. Whatever this becomes,
it should trigger playback of an already-validated file, never skip
validation, and should have the same Stop/Disarm-independent-of-trigger
property as everything else here (a stuck/repeated button press should
not be able to force repeated unbounded motion without the same repeat
bounding and thermal caveats as manual looping).

## Open items / not yet resolved

- Exact numeric thresholds for the statistical outlier check (how many
  local-average-multiples counts as "reject")
- Whether validation failures should hard-block the whole file or offer
  a "skip/interpolate over this sample" repair option
- Visualizer editor slider bounds should be pulled from the same real
  limit constants (separate, smaller, non-blocking task)
- Not yet reviewed against Carla's own independent overnight thinking
  on the same problem
