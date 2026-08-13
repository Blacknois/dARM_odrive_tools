"""
forward_kinematics.py

Pure Python (stdlib only, no numpy) forward-kinematics for the dARM arm,
ported directly from darm_visualizer.html's CHAIN/applyPose() - same
link offsets, fixed mounting rpy, and raw-position -> joint-angle
conversions, which were validated against a real photo on 2026-07-31.

This is the math foundation for "Ground Clamp" - computing where the
arm's key points actually are in 3D space for a given set of raw node
positions, so a floor-height (or other geometric) safety check can be
computed for ANY configuration without having measured that specific
combination before.

Units: meters, radians internally; raw node positions are whatever
units axis0.pos_estimate uses (matches gamecontroller.py/visualizer).

Coordinate frame: URDF-native, Z-up, origin at the base_link joint
(node 0's rotation axis). This is NOT the same frame Three.js renders
in (the visualizer additionally rotates -90deg about X for Y-up
rendering) - that rotation is a display-only concern and is
deliberately NOT replicated here, since this module only cares about
real-world "up" for a floor check.
"""
import math

DEG = math.pi / 180.0

# Link offsets (meters) and fixed URDF mounting rpy (radians) - not
# user-tunable, from the real CAD/URDF, same source as the visualizer.
LINK_OFFSETS = {
    "link_1":       (0.0, 0.0, 0.14444),
    "link_2":       (0.0, 0.0, 0.07556),
    "link_3":       (0.0, 0.0, 0.18708),
    "forearm":      (0.0, 0.0, 0.06292),
    "differential": (0.0, 0.0, 0.34),
    "gripper":      (0.0, 0.0, 0.02),
}
LINK_RPY = {
    "link_1":       (0.0, 0.0, 0.0),
    "link_2":       (-113 * DEG, 0.0, 0.0),
    "link_3":       (0.0, 0.0, 0.0),
    "forearm":      (122.9 * DEG, 0.0, 0.0),
    "differential": (0.0, 0.0, 0.0),
    "gripper":      (0.0, 0.0, 0.0),
}
FINGER_OFFSET = 0.091365  # meters, gripper origin -> finger tips along local Z

SHOULDER_CAD_MAX_DEG = 226.0
ELBOW_CAD_MAX_DEG = 245.8

# Same calibrated raw<->degree ratios as the visualizer panel defaults.
SCALE_8308 = 40.06         # base (node0), elbow-roll (node3), deg/raw - unified value, average of THREE independent powered mark-alignment tests 2026-08-10 (elbow-roll single 360deg: 40.07; elbow-roll +/-360deg two-mark: 40.06; base +/-360deg two-mark: 40.04). Was briefly split into two separate constants mid-session based on an old ~210deg node0 harness-limit reference that turned out to likely be circular (computed from the old 25.9 value, not independently measured) - re-unified once real node0 mark-alignment data confirmed it matches node3 after all, consistent with the two joints genuinely sharing identical parts/gear ratio/motor.
SCALE_SHOULDER = 41.6       # deg/raw
SCALE_ELBOW = 40.73         # deg/raw (was 37.93, corrected via powered 90deg forearm parallel/perpendicular-to-floor test 2026-08-10)
SCALE_WRIST_BEND = 29.1     # deg/raw - FIXED 2026-08-09: was still using the stale 13.75 value that gamecontrollers own comment already flagged as ~2x wrong back on 2026-08-06 - never got propagated here, causing finger_tip height to be dramatically overstated for any bent-wrist pose
SCALE_WRIST_ROTATE = 27.7   # deg/raw (was 12.0, corrected via powered 180deg + 360deg mark-alignment tests 2026-08-10)


# --- minimal 3x3 matrix / 3-vector helpers (no numpy) -----------------

def mat_mult(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )

def mat_vec(m, v):
    return tuple(sum(m[i][k] * v[k] for k in range(3)) for i in range(3))

def vec_add(a, b):
    return tuple(a[i] + b[i] for i in range(3))

IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))

def rot_axis(axis, theta):
    """Rodrigues' rotation formula - rotation matrix for `theta` radians
    about a unit vector `axis`. Works for any axis, so link_1/link_3's
    Z-axis and link_2/forearm's X-axis are both just this one function."""
    x, y, z = axis
    c, s = math.cos(theta), math.sin(theta)
    C = 1 - c
    return (
        (x*x*C + c,   x*y*C - z*s, x*z*C + y*s),
        (y*x*C + z*s, y*y*C + c,   y*z*C - x*s),
        (z*x*C - y*s, z*y*C + x*s, z*z*C + c),
    )

def rot_rpy(rpy):
    """Fixed URDF mounting rotation, applied as separate X then Y then Z
    rotations composed in that order - matches THREE.Euler's default
    'XYZ' order, which is what off.rotation.set(rx,ry,rz) uses."""
    rx, ry, rz = rpy
    Rx = rot_axis((1, 0, 0), rx)
    Ry = rot_axis((0, 1, 0), ry)
    Rz = rot_axis((0, 0, 1), rz)
    return mat_mult(mat_mult(Rx, Ry), Rz)


def joint_angles(n0, n12, n3, n4, n5, n6):
    """Raw node positions -> actual joint rotation (axis, angle) per
    link, replicating applyPose() exactly (same scales, same CAD-max
    offsets, same sign conventions)."""
    bend_pos = (n5 - n6) / 2.0
    rotate_pos = (n5 + n6) / 2.0
    return {
        "link_1":       ((0, 0, 1), n0 * SCALE_8308 * DEG),
        "link_2":       ((1, 0, 0), (SHOULDER_CAD_MAX_DEG + n12 * SCALE_SHOULDER) * DEG),
        "link_3":       ((0, 0, 1), n3 * SCALE_8308 * DEG),
        "forearm":      ((1, 0, 0), -(ELBOW_CAD_MAX_DEG - n4 * SCALE_ELBOW) * DEG),
        "differential": ((1, 0, 0), bend_pos * SCALE_WRIST_BEND * DEG),
        "gripper":      ((0, 0, 1), -rotate_pos * SCALE_WRIST_ROTATE * DEG),
    }


CHAIN_ORDER = ["link_1", "link_2", "link_3", "forearm", "differential", "gripper"]


def forward_kinematics(n0, n12, n3, n4, n5, n6):
    """
    Returns a dict of link name -> (x,y,z) position of that link's own
    origin, in the base_link frame (meters, Z-up, URDF-native - NOT the
    visualizer's Y-up rendering frame). Also returns 'finger_tip' as the
    lowest-reaching real point past the gripper.

    n0/n3/n4 = raw pos_estimate for nodes 0/3/4. n12 = node 1's raw
    value (shoulder pair, node 1 stands in for both, same convention as
    the recorder/visualizer). n5/n6 = raw pos_estimate for nodes 5/6.
    """
    angles = joint_angles(n0, n12, n3, n4, n5, n6)

    R = IDENTITY
    t = (0.0, 0.0, 0.0)
    positions = {}

    for name in CHAIN_ORDER:
        offset = LINK_OFFSETS[name]
        rpy = LINK_RPY[name]
        axis, theta = angles[name]

        R_local = mat_mult(rot_rpy(rpy), rot_axis(axis, theta))
        # this link's rot-frame origin, in the previous link's rot-frame,
        # is just `offset` (translation happens before entering the
        # rotated frame) - so in the BASE frame it's t + R @ offset
        t = vec_add(t, mat_vec(R, offset))
        R = mat_mult(R, R_local)
        positions[name] = t

    # finger tip: a point FINGER_OFFSET further along the gripper's own
    # local Z, mapped into the base frame the same way as any other point
    finger_tip = vec_add(t, mat_vec(R, (0.0, 0.0, FINGER_OFFSET)))
    positions["finger_tip"] = finger_tip

    return positions


# --- Ground Clamp: floor-height safety check ---------------------------
#
# Calibrated 2026-08-06 against a real captured snapshot of the wrist
# physically touching the floor (computed finger_tip z = +0.0039m in
# the base frame at that exact configuration). The floor itself isn't a
# precise reference (wood floor, not perfectly flat/clean), so this is
# deliberately a rough calibration, not a precision one - the margin
# below matters far more than nailing the constant to the millimeter.
FLOOR_Z_BASE_FRAME = 0.004  # meters, real measured floor contact point
FLOOR_MARGIN_DEFAULT = 0.05  # meters - fallback for any point without a
                              # real-measurement-backed margin below.
                              # Accounts for real floor unevenness/debris,
                              # not for FK math uncertainty.

# Per-point margins (2026-08-09): link_3 and forearm both sit on the
# same modular gearbox housing shared across nodes 0-4 (confirmed by
# DrJones - interchangeable layers, identical outer diameter). Real
# caliper measurement of that housing: 125.79mm max width (across the
# ridges that hold the assembly bolts, not the smooth valleys between -
# using the smaller valley measurement would underestimate the real
# bulge). Using half that (radius, assuming the skeleton line runs
# through the housing's center) as the real margin for those two points,
# replacing the old flat 5cm guess with something grounded in a real
# measurement. differential/gripper/finger_tip are a different,
# unmeasured component - left at the old default until they get their
# own real measurement, not changed just because link_3/forearm changed.
FLOOR_MARGIN_BY_POINT = {
    "link_3":  0.063,   # 125.79mm housing / 2, real caliper measurement
    "forearm": 0.063,   # same shared gearbox housing as link_3
}

def _safe_z_for(name):
    return FLOOR_Z_BASE_FRAME + FLOOR_MARGIN_BY_POINT.get(name, FLOOR_MARGIN_DEFAULT)

# Which computed points actually get checked against the floor.
# CORRECTED 2026-08-06: originally excluded link_3/forearm on the
# assumption they "stay high in every real configuration" - a real
# elbow-on-the-floor test proved that wrong (elbow/forearm can clearly
# reach the floor too, independent of wrist state). Now checks every
# link past the shoulder, since any of them can plausibly swing low.
FLOOR_CHECK_POINTS = ("link_3", "forearm", "differential", "gripper", "finger_tip")

# Kept for anything that still imports a single scalar (e.g. old scripts) -
# reflects the default/fallback margin, NOT the real per-point values above.
FLOOR_MARGIN = FLOOR_MARGIN_DEFAULT
FLOOR_SAFE_Z = FLOOR_Z_BASE_FRAME + FLOOR_MARGIN_DEFAULT


def check_floor_clamp(n0, n12, n3, n4, n5, n6, safe_z=None):
    """
    Returns (is_safe, positions, violations) for the given candidate
    raw node positions. is_safe is False if ANY of FLOOR_CHECK_POINTS
    would be at or below its own real per-point safe_z (see
    FLOOR_MARGIN_BY_POINT). Passing an explicit `safe_z` overrides this
    and applies that one flat value to every point instead (kept for
    backward compatibility / manual what-if checks).
    violations is a dict of {point_name: z} for just the points that
    failed, for logging.
    """
    positions = forward_kinematics(n0, n12, n3, n4, n5, n6)
    violations = {
        name: positions[name][2]
        for name in FLOOR_CHECK_POINTS
        if positions[name][2] <= (safe_z if safe_z is not None else _safe_z_for(name))
    }
    return (len(violations) == 0, positions, violations)


if __name__ == "__main__":
    # Sanity check against the real captured snapshot (2026-08-06,
    # wrist physically touching the floor):
    # node0=0.018, node1=-1.466, node3=0.042, node4=6.136,
    # node5=-3.349, node6=3.481
    pos = forward_kinematics(
        n0=0.018018066883087158,
        n12=-1.4660530090332031,
        n3=0.041766587644815445,
        n4=6.136322975158691,
        n5=-3.3491427898406982,
        n6=3.480604648590088,
    )
    for name, p in pos.items():
        print(f"{name:14s} x={p[0]:+.4f} y={p[1]:+.4f} z={p[2]:+.4f}")

    print(f"\nFLOOR_SAFE_Z = {FLOOR_SAFE_Z:.4f}m (measured {FLOOR_Z_BASE_FRAME:.4f} + margin {FLOOR_MARGIN:.4f})")
    safe, _, violations = check_floor_clamp(
        0.018018066883087158, -1.4660530090332031, 0.041766587644815445,
        6.136322975158691, -3.3491427898406982, 3.480604648590088,
    )
    print(f"check_floor_clamp at the real floor-touch snapshot: safe={safe}, violations={violations}")
    print("(expected: safe=False - this exact real config is the whole reason for the margin)")
