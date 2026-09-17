#!/usr/bin/env python3
"""
build_calibration.py - board calibration from the four measured corners.

The board was physically moved on 2026-09-06 (now below the base). The
previous calibration, chess_board_calibration_2026-08-20_48mmRoll.json,
describes the OLD position and is wrong by 616-1163 mm per square. Anything
reading it would drive the arm about a metre from the real board.

This builds a replacement by bilinear interpolation from four corner
squares measured POWERED and holding position, read back through forward
kinematics (see darm_board_geometry_measured in Claude's memory).

WHY MEASURED-IN-MODEL-SPACE COORDINATES ARE THE RIGHT THING TO STORE:
the arm sags ~34mm at full extension and the kinematic model does not know
it. So the model's idea of where the tip is, when the tip is physically on
a square, is NOT the square's true position - and that is exactly what
should be stored. Command that value and the same sag applies again, so
the arm lands back on the square. This is why the old catalogue also had a
52mm z-spread across a level board.

LIMITATION, STATED PLAINLY: four real points, not the 46 of the previous
calibration. Interpolation assumes the sag varies smoothly across the
board. Good enough to move safely and refine from; not a finished
calibration. Squares far from a corner are the least trustworthy.

Writes a NEW dated file. Never modifies the existing one.
"""
import json, math, os, sys, datetime

CORNERS = {
    "a1": [0.0295, -0.2865, -0.1329],
    "h1": [-0.2346, -0.1117, -0.1366],
    "a8": [-0.1681, -0.5703, -0.0995],
    "h8": [-0.4455, -0.3841, -0.1026],
}
FILES = "abcdefgh"
OUT_DIR = "/home/darm/dARM/odrive_tools"


def square(fi, ri):
    """fi = file index a..h (0..7), ri = rank index 1..8 (0..7)."""
    u, v = fi / 7.0, ri / 7.0
    return [round(CORNERS["a1"][k] * (1 - u) * (1 - v)
                + CORNERS["h1"][k] * u * (1 - v)
                + CORNERS["a8"][k] * (1 - u) * v
                + CORNERS["h8"][k] * u * v, 4) for k in range(3)]


def main():
    squares = {}
    for ri in range(8):
        for fi in range(8):
            squares["%s%d" % (FILES[fi], ri + 1)] = square(fi, ri)

    # sanity: interpolated corners must reproduce the measurements exactly
    for name in CORNERS:
        got = squares[name]
        gap = math.dist(got, CORNERS[name]) * 1000
        assert gap < 0.2, "%s off by %.2f mm" % (name, gap)

    # derived board vectors, for anything that expects them
    a1, h1, a8 = squares["a1"], squares["h1"], squares["a8"]
    fvec = [h1[k] - a1[k] for k in range(3)]
    rvec = [a8[k] - a1[k] for k in range(3)]
    n = lambda v: [x / math.sqrt(sum(c * c for c in v)) for x in v]

    data = {
        "calibrated": ("%s (build_calibration.py - bilinear interpolation from "
                       "FOUR corner squares measured powered and holding "
                       "position on 2026-09-06, read back through forward "
                       "kinematics; the board was moved below the base and the "
                       "previous calibration was wrong by 616-1163 mm)"
                       % datetime.date.today().isoformat()),
        "reference_frame": "world (robot base_link-anchored, Z-up)",
        "touch_reference": "finger_tip TF frame",
        "square_size_m": 0.048,
        "reference_squares": ["a1", "h1", "a8", "h8"],
        "file_unit_vector": [round(x, 6) for x in n(fvec)],
        "rank_unit_vector": [round(x, 6) for x in n(rvec)],
        "method": "bilinear interpolation from 4 measured corners",
        "confidence": ("APPROXIMATE. Four real points only. Interpolation "
                       "assumes drivetrain sag varies smoothly across the "
                       "board. Squares far from a corner are least reliable. "
                       "Verify a square by touching it before trusting it."),
        "squares": squares,
    }

    stamp = datetime.date.today().isoformat()
    path = os.path.join(OUT_DIR, "chess_board_calibration_%s_4corner.json" % stamp)
    if os.path.exists(path):
        sys.exit("refusing to overwrite existing %s" % path)
    with open(path, "w") as f:
        json.dump(data, f, indent=2)

    print("wrote %s\n" % path)
    print("  edge lengths (should each be 7 x 48 = 336 mm):")
    for a, b, lab in (("a1", "h1", "near"), ("a8", "h8", "far"),
                      ("a1", "a8", "a-file"), ("h1", "h8", "h-file")):
        mm = math.dist(squares[a][:2], squares[b][:2]) * 1000
        print("    %-8s %s-%s  %6.1f mm  %+.1f" % (lab, a, b, mm, mm - 336))
    print("\n  %-6s %-9s %-9s %-9s %s" % ("sq", "x", "y", "z", "radial"))
    for k in ("a1", "d4", "e5", "h1", "a8", "e8", "h8"):
        x, y, z = squares[k]
        print("  %-6s %+9.4f %+9.4f %+9.4f %8.4f" % (k, x, y, z, math.hypot(x, y)))


if __name__ == "__main__":
    main()
