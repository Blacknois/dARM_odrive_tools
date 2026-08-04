#!/usr/bin/env python3

import time
import threading
import signal
import urwid
import pygame
import can

from src.can_utils import discover_node_ids
from src.control import move_odrive_to_position, set_closed_loop_control, set_idle_mode
from src.metrics import get_metrics, METRIC_ENDPOINTS
from src.configure import load_endpoints, read_config, write_config

# ------------------------------------------------------------------------------
# 1) Button and Axis Definitions - https://www.pygame.org/docs/ref/joystick.html
# ------------------------------------------------------------------------------
DEAD_MAN_BUTTON_INDEX   = 4  # LB
MODE_TOGGLE_BUTTON_INDEX = 5 # RB
ARM_BUTTON_INDEX        = 1     # Circle
PS_BUTTON_INDEX         = 10    # PS button
DPAD_ARM_DIRECTION      = (-1, 0)  # D-pad left
ARM_HOLD_SECONDS        = 1.5
DISARM_HOLD_SECONDS     = 1.0
GRIPPER_STARTUP_OPEN    = 0.4

AXIS_LEFT_X       = 0  # Left stick horizontal
AXIS_LEFT_Y       = 1  # Left stick vertical
AXIS_LEFT_TRIGGER = 2  # L2 (physical left trigger)  
AXIS_RIGHT_X      = 3  # Right stick horizontal
AXIS_RIGHT_Y      = 4  # Right stick vertical
AXIS_RIGHT_TRIGGER= 5  # R2 (physical right trigger)  

UPDATE_RATE              = 30.0
DEAD_ZONE                = 0.25
VELOCITY_SCALING         = 1.5
FOREARM_VELOCITY_SCALING = 2.0
GRIPPER_SCALING          = 0.5

# ------------------------------------------------------------------------------
# 2) Joint Range Definitions
#    Node0     => Joint 0
#    Node(1,2) => Joint 1 (Shoulder)
#    Node3     => Joint 2
#    Node4     => Joint 3
#    Node(5,6) => Wrist (bend + rotate)
#    Node(7)   => Gripper squeeze/release
# ------------------------------------------------------------------------------
JOINT0_MIN, JOINT0_MAX = -8.22,  7.91
JOINT1_MIN, JOINT1_MAX = -5.43,  0.0
JOINT2_MIN, JOINT2_MAX = -8.38,  12.89
JOINT3_MIN, JOINT3_MAX =  0.0,   5.76

BEND_MIN,   BEND_MAX     =  -5.0,  5.0 # Wrist
ROTATE_MIN, ROTATE_MAX   = -10.0, 10.0 # Wrist
TRIGGER_MIN, TRIGGER_MAX =  0.0, 0.80

# Node5/6 raw safety envelope, measured empirically from a combined
# tilt+rotation test, plus a large safety margin. This is the
# authoritative clamp applied in WriteController.apply() - BEND/ROTATE
# above are only soft bounds on the internal accumulator and are NOT
# sufficient on their own to guarantee this range.
MOTOR5_MIN, MOTOR5_MAX = -28.41, 6.06
MOTOR6_MIN, MOTOR6_MAX = -27.36, 4.57

# Soft-limit deceleration: commanded speed scales down within this
# distance (same units as the joint ranges above) of a min/max limit.
DECEL_ZONE         = 1.0
DECEL_ZONE_WRIST   = 3.0
DECEL_ZONE_GRIPPER = 0.2

def taper_increment(current_val, increment, min_val, max_val, decel_zone):
    """
    Scales down `increment` as `current_val` approaches min_val/max_val,
    so movement eases into a limit instead of arriving at full speed.
    """
    if increment > 0:
        remaining = max_val - current_val
    elif increment < 0:
        remaining = current_val - min_val
    else:
        return increment
    if decel_zone > 0 and remaining < decel_zone:
        scale = max(0.05, remaining / decel_zone)
        increment *= scale
    return increment

stop_event = threading.Event()

# Joystick states for UI display
joystick_states = {
    "LB": False,
    "Dpad": False,
    "Circle": False,
    "PS": False,
    "status": "",
    "axes": {
        AXIS_LEFT_X:  0.0,
        AXIS_LEFT_Y:  0.0,
        AXIS_RIGHT_X: 0.0,
        AXIS_RIGHT_Y: 0.0,
        AXIS_LEFT_TRIGGER:  0.0,  # <--- For debugging display
        AXIS_RIGHT_TRIGGER: 0.0   # <--- For debugging display
    }
}

# ------------------------------------------------------------------------------
# 3) Helper Functions
# ------------------------------------------------------------------------------
def apply_dead_zone(value):
    return 0.0 if abs(value) < DEAD_ZONE else value

def signal_handler(sig, frame):
    raise KeyboardInterrupt

def handle_input(key, loop, node_ids, bus, joint_positions):
    if key == 'esc':
        stop_event.set()
        raise urwid.ExitMainLoop()

def clean_shutdown(node_ids, bus, joint_positions, endpoints):
    print("\nExiting... Setting discovered ODrives to pos=0 => IDLE => shutdown.")
    try:
        move_to_neutral_slowly(bus, node_ids, endpoints)
    except Exception as e:
        print(f"[WARN] Error during slow move to neutral: {e}")
    failed = []
    for nid in node_ids:
        if not disarm_verified(bus, nid, endpoints):
            failed.append(nid)
    if failed:
        print(f"[WARNING] Nodes NOT confirmed disarmed: {failed} - check manually (check_armed.py) before next power-on.")
    else:
        print("[INFO] All nodes confirmed disarmed.")
    if bus:
        bus.shutdown()

def force_disarm_all(bus, node_ids, endpoints):
    """
    Immediately cuts torque on every listed node (IDLE), with no slow
    move first. Used when PS is held continuously past
    DISARM_HOLD_SECONDS, overriding the gentler stop-and-return-to-rest.
    Verifies each node actually confirms disarmed via is_armed read-back
    rather than trusting the CAN send alone.
    """
    print("\n>>> FORCED DISARM - TORQUE CUT IMMEDIATELY <<<\n")
    failed = []
    for nid in node_ids:
        if not disarm_verified(bus, nid, endpoints):
            failed.append(nid)
    if failed:
        print(f"[WARNING] Nodes NOT confirmed disarmed: {failed} - check manually (check_armed.py).")

# ------------------------------------------------------------------------------
# 4) Shoulder & Gripper Classes
# ------------------------------------------------------------------------------
def read_position(bus, node_id, endpoints):
    """
    Reads a node's actual current position (axis0.pos_estimate).
    """
    ep = endpoints['endpoints']['axis0.pos_estimate']
    return read_config(bus, node_id, ep['id'], ep['type'])

def arm_node_verified(bus, node_id, endpoints, retries=5, settle_timeout=0.3):
    """
    Requests CLOSED_LOOP_CONTROL and confirms via axis0.is_armed that the
    node actually latched into it, instead of trusting that the CAN send
    alone succeeded.
    """
    armed_ep  = endpoints['endpoints']['axis0.is_armed']
    disarm_ep = endpoints['endpoints']['axis0.disarm_reason']
    errors_ep = endpoints['endpoints']['axis0.active_errors']
    for attempt in range(retries):
        set_closed_loop_control(bus, node_id)
        start = time.time()
        while time.time() - start < settle_timeout:
            armed = read_config(bus, node_id, armed_ep['id'], armed_ep['type'])
            if armed:
                return True
            time.sleep(0.05)
    disarm_reason = read_config(bus, node_id, disarm_ep['id'], disarm_ep['type'])
    active_errors = read_config(bus, node_id, errors_ep['id'], errors_ep['type'])
    print(f"[ERROR] Node {node_id} did not confirm armed after {retries} attempts "
          f"(disarm_reason={disarm_reason}, active_errors={active_errors})")
    return False

def disarm_verified(bus, node_id, endpoints, retries=5, settle_timeout=0.3):
    """
    Requests IDLE and confirms via axis0.is_armed that the node actually
    latched into it, instead of trusting that the CAN send alone
    succeeded. Mirrors arm_node_verified() for the opposite transition -
    a fire-and-forget IDLE command can silently fail to take effect on a
    lossy CAN bus, leaving a node armed with no indication of failure.
    """
    armed_ep = endpoints['endpoints']['axis0.is_armed']
    for attempt in range(retries):
        set_idle_mode(bus, node_id)
        start = time.time()
        while time.time() - start < settle_timeout:
            armed = read_config(bus, node_id, armed_ep['id'], armed_ep['type'])
            if armed is not None and not armed:
                return True
            time.sleep(0.05)
    print(f"[ERROR] Node {node_id} did not confirm disarmed after {retries} attempts.")
    return False

def write_verified(bus, node_id, endpoint_id, endpoint_type, value, label="",
                    retries=5, settle_timeout=0.3, tolerance=1e-3):
    """
    Writes a value and confirms via read-back that it actually landed,
    instead of trusting a single fire-and-forget CAN write. Mirrors
    disarm_verified()/arm_node_verified() - CAN writes on this bus are
    unreliable and can silently fail with no error raised. This is what
    was missing from move_to_neutral_slowly()'s halve/restore steps,
    which caused trap_traj limits to be repeatedly halved over time.
    """
    for attempt in range(retries):
        write_config(bus, node_id, endpoint_id, endpoint_type, value)
        start = time.time()
        while time.time() - start < settle_timeout:
            readback = read_config(bus, node_id, endpoint_id, endpoint_type)
            if readback is not None and abs(readback - value) <= tolerance:
                return True
            time.sleep(0.05)
    print(f"[ERROR] Node {node_id} {label} did not confirm write of {value} after {retries} attempts.")
    return False

def move_to_neutral_slowly(bus, node_ids, endpoints, targets=None):
    """
    Temporarily halves each node's trap_traj vel/accel/decel limits (not
    saved to flash - reverts on next reboot or setup.py run), moves every
    node to position 0 (the photographed rest pose), and waits for the
    move to complete before returning. Optional `targets` dict maps
    node_id -> target position; any node not listed defaults to 0.0.

    Both the halve step and the restore step use write_verified() to
    confirm each write actually landed via read-back - CAN writes on
    this bus can silently fail, which previously caused trap_traj
    limits to be repeatedly halved without ever being restored.
    """
    vel_ep   = endpoints['endpoints']['axis0.trap_traj.config.vel_limit']
    accel_ep = endpoints['endpoints']['axis0.trap_traj.config.accel_limit']
    decel_ep = endpoints['endpoints']['axis0.trap_traj.config.decel_limit']

    orig_values = {}
    failed_halve = []
    for nid in node_ids:
        vel   = read_config(bus, nid, vel_ep['id'], vel_ep['type'])
        accel = read_config(bus, nid, accel_ep['id'], accel_ep['type'])
        decel = read_config(bus, nid, decel_ep['id'], decel_ep['type'])
        orig_values[nid] = (vel, accel, decel)
        if vel is not None and not write_verified(bus, nid, vel_ep['id'], vel_ep['type'], vel / 2, label="trap_vel halve"):
            failed_halve.append((nid, "vel"))
        if accel is not None and not write_verified(bus, nid, accel_ep['id'], accel_ep['type'], accel / 2, label="trap_accel halve"):
            failed_halve.append((nid, "accel"))
        if decel is not None and not write_verified(bus, nid, decel_ep['id'], decel_ep['type'], decel / 2, label="trap_decel halve"):
            failed_halve.append((nid, "decel"))
    if failed_halve:
        print(f"[WARNING] Speed-limit halve not confirmed for: {failed_halve} - proceeding with move anyway.")

    print("[INFO] Moving slowly to neutral (rest) position...")
    for nid in node_ids:
        target = targets[nid] if (targets and nid in targets) else 0.0
        move_odrive_to_position(bus, nid, target)
    time.sleep(4)

    failed_restore = []
    for nid in node_ids:
        vel, accel, decel = orig_values[nid]
        if vel is not None and not write_verified(bus, nid, vel_ep['id'], vel_ep['type'], vel, label="trap_vel restore"):
            failed_restore.append((nid, "vel"))
        if accel is not None and not write_verified(bus, nid, accel_ep['id'], accel_ep['type'], accel, label="trap_accel restore"):
            failed_restore.append((nid, "accel"))
        if decel is not None and not write_verified(bus, nid, decel_ep['id'], decel_ep['type'], decel, label="trap_decel restore"):
            failed_restore.append((nid, "decel"))

    if failed_restore:
        print(f"[WARNING] Speed limits NOT confirmed restored for: {failed_restore} - check manually (diag_node_compare2.py) before further motion.")
    else:
        print("[INFO] Neutral position reached. Normal speed limits restored (confirmed).")

class ShoulderController:
    """
    Node(1,2) => single shoulder joint with two motors in opposite directions.
    We track 'value' => motorA = +value, motorB = -value.
    """
    def __init__(self, bus, node_ids):
        self.bus      = bus
        self.node_ids = node_ids
        self.value    = 0.0

    def apply(self):
        nA, nB = self.node_ids
        motorA = self.value
        motorB = -self.value
        move_odrive_to_position(self.bus, nA, motorA)
        move_odrive_to_position(self.bus, nB, motorB)

class WriteController:
    """
    Node(5,6) => Wrist with bend_pos and rotate_pos
    """
    def __init__(self, bus, node_ids):
        self.bus        = bus
        self.node_ids   = node_ids  # [5,6]
        self.bend_pos   = 0.0
        self.rotate_pos = 0.0

    def clamp(self, val, min_val, max_val):
        return max(min_val, min(max_val, val))

    def apply(self):
        nA, nB = self.node_ids
        motorA = self.rotate_pos + self.bend_pos
        motorB = self.rotate_pos - self.bend_pos
        # Authoritative raw safety clamp - see MOTOR5/6_MIN/MAX above.
        motorA = self.clamp(motorA, MOTOR5_MIN, MOTOR5_MAX)
        motorB = self.clamp(motorB, MOTOR6_MIN, MOTOR6_MAX)
        move_odrive_to_position(self.bus, nA, motorA)
        move_odrive_to_position(self.bus, nB, motorB)

# ------------------------------------------------------------------------------
# 5) UI Update Thread
# ------------------------------------------------------------------------------
def update_ui_thread(bus, node_ids, endpoints, metrics_text, joystick_text, loop):
    col_widths = {}
    for metric in METRIC_ENDPOINTS:
        col_widths[metric] = max(len(metric), 4) + 3

    node_col_w = 6
    header = f"{'Node':<{node_col_w}}" + "".join(
        f"{m:<{col_widths[m]}}" for m in METRIC_ENDPOINTS
    )

    axis_names = ["LeftX", "LeftY", "RightX", "RightY", "LTrig", "RTrig"]
    axis_col_w = max(len(n) for n in axis_names) + 3
    joy_header_line = "LB".ljust(8) + "".join(x.ljust(axis_col_w) for x in axis_names)

    while not stop_event.is_set():
        # ODrive metrics
        lines = [header]
        for nid in node_ids:
            data = get_metrics(bus, nid, endpoints)
            row = f"{nid:<{node_col_w}}"
            for metric in METRIC_ENDPOINTS:
                val = data.get(metric, None)
                if isinstance(val, (int, float)):
                    sign_space = ' ' if val >= 0 else ''
                    row += f"{sign_space}{val:.2f}".ljust(col_widths[metric])
                else:
                    row += f"{'None':<{col_widths[metric]}}"
            lines.append(row)
        metrics_text.set_text("\n".join(lines))

        # Joystick line
        lb_str = "Pressed" if joystick_states["LB"] else "NotPress"
        joy_line = lb_str.ljust(8)

        # We display the 6 axes we track
        axis_values = [
            joystick_states["axes"].get(AXIS_LEFT_X, 0.0),
            joystick_states["axes"].get(AXIS_LEFT_Y, 0.0),
            joystick_states["axes"].get(AXIS_RIGHT_X, 0.0),
            joystick_states["axes"].get(AXIS_RIGHT_Y, 0.0),
            joystick_states["axes"].get(AXIS_LEFT_TRIGGER, 0.0),
            joystick_states["axes"].get(AXIS_RIGHT_TRIGGER, 0.0),
        ]
        for val in axis_values:
            joy_line += f"{val:>6.2f}".ljust(axis_col_w)

        dpad_str = "HELD" if joystick_states["Dpad"] else "-"
        circle_str = "HELD" if joystick_states["Circle"] else "-"
        ps_str = "HELD" if joystick_states["PS"] else "-"
        gesture_line = f"DpadLeft:{dpad_str}  Circle:{circle_str}  PS:{ps_str}"
        status_line = f"Status: {joystick_states['status']}"
        joystick_text.set_text(joy_header_line + "\n" + joy_line + "\n\n" + gesture_line + "\n" + status_line)

        time.sleep(0.1)
        try:
            loop.draw_screen()
        except (urwid.ExitMainLoop, RuntimeError):
            break

# ------------------------------------------------------------------------------
# 6) Main Joystick Logic Thread
# ------------------------------------------------------------------------------
def joystick_thread_func(
    bus, node_ids, joint_positions,
    shoulder_ctrl, wrist_ctrl, endpoints,
    update_rate = UPDATE_RATE
):
    """
    - If LB pressed         => normal mode
    - If LB and RB pressed  => wrist mode
    """
    clock = pygame.time.Clock()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    connected = True

    dpad_arm_hold_start   = None
    arm_triggered_this_hold = False
    awaiting_l1_reset     = False
    l1_released_since_arm = False
    ps_prev               = False
    ps_press_time         = None
    return_thread         = None

    while not stop_event.is_set():
        dt = clock.tick(update_rate) / 1000.0

        # Watchdog: detect controller disconnect via the actual SDL event
        # (more reliable than hoping a stale read errors out on its own).
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEREMOVED:
                if connected:
                    print("\n[SAFETY] Controller disconnected! Freezing all motion - no further commands will be sent until it reconnects.\n")
                connected = False
            elif event.type == pygame.JOYDEVICEADDED:
                try:
                    joystick = pygame.joystick.Joystick(0)
                    joystick.init()
                    if not connected:
                        print("\n[INFO] Controller reconnected. Resuming control.\n")
                    connected = True
                except pygame.error:
                    connected = False

        if not connected:
            # No new commands sent at all while disconnected - ODrives hold
            # their last commanded position in closed loop and do not move.
            time.sleep(0.05)
            continue

        try:
            lb = joystick.get_button(DEAD_MAN_BUTTON_INDEX)
            rb = joystick.get_button(MODE_TOGGLE_BUTTON_INDEX)
            dpad = joystick.get_hat(0)
            circle = joystick.get_button(ARM_BUTTON_INDEX)
            ps = joystick.get_button(PS_BUTTON_INDEX)
        except pygame.error:
            if connected:
                print("\n[SAFETY] Lost contact with controller! Freezing all motion.\n")
            connected = False
            time.sleep(0.05)
            continue

        joystick_states["LB"] = bool(lb)
        joystick_states["Dpad"] = (dpad == DPAD_ARM_DIRECTION)
        joystick_states["Circle"] = bool(circle)
        joystick_states["PS"] = bool(ps)

        # Left stick: X (horizontal) => rotate, Y (vertical) => bend
        raw_bend   = apply_dead_zone(joystick.get_axis(AXIS_LEFT_Y))
        raw_rotate = apply_dead_zone(joystick.get_axis(AXIS_LEFT_X))

        # Right stick
        rx = apply_dead_zone(joystick.get_axis(AXIS_RIGHT_X))
        ry = apply_dead_zone(joystick.get_axis(AXIS_RIGHT_Y))

        # New triggers
        raw_lt = apply_dead_zone(joystick.get_axis(AXIS_LEFT_TRIGGER))
        raw_rt = apply_dead_zone(joystick.get_axis(AXIS_RIGHT_TRIGGER))

        joystick_states["axes"][AXIS_LEFT_X]        = raw_rotate
        joystick_states["axes"][AXIS_LEFT_Y]        = raw_bend
        joystick_states["axes"][AXIS_RIGHT_X]       = rx
        joystick_states["axes"][AXIS_RIGHT_Y]       = ry
        joystick_states["axes"][AXIS_LEFT_TRIGGER]  = raw_lt
        joystick_states["axes"][AXIS_RIGHT_TRIGGER] = raw_rt

        # --- Arm gesture: D-pad left + Circle held 1.5s ---
        if dpad == DPAD_ARM_DIRECTION and circle:
            if dpad_arm_hold_start is None:
                dpad_arm_hold_start = time.time()
            elif (not arm_triggered_this_hold) and (time.time() - dpad_arm_hold_start) >= ARM_HOLD_SECONDS:
                print("\n[INFO] Arm gesture held - re-syncing and arming all nodes...\n")
                joystick_states["status"] = "ARMING: re-syncing all nodes..."
                newly_armed = set()
                for nid in node_ids:
                    pos = None
                    for attempt in range(3):
                        pos = read_position(bus, nid, endpoints)
                        if pos is not None:
                            break
                        time.sleep(0.05)
                    if pos is None:
                        print(f"[WARN] Node {nid}: position read failed after 3 attempts - skipping resync/arm for this node this attempt (no guessed target commanded).")
                        continue
                    joint_positions[nid] = pos
                    move_odrive_to_position(bus, nid, pos)
                    if arm_node_verified(bus, nid, endpoints):
                        newly_armed.add(nid)
                        print(f"[INFO] Node {nid} confirmed armed.")
                    else:
                        print(f"[WARN] Node {nid} did NOT arm.")
                if shoulder_ctrl and (1 in node_ids) and (2 in node_ids):
                    shoulder_ctrl.value = (joint_positions[1] - joint_positions[2]) / 2.0
                if wrist_ctrl and (5 in node_ids) and (6 in node_ids):
                    wrist_ctrl.rotate_pos = (joint_positions[5] + joint_positions[6]) / 2.0
                    wrist_ctrl.bend_pos   = (joint_positions[5] - joint_positions[6]) / 2.0
                print(f"[INFO] Re-arm complete: {sorted(newly_armed)}\n")
                print("[SAFETY] Release and re-press L1 before any motion will be accepted.\n")
                joystick_states["status"] = f"RE-ARM COMPLETE: {sorted(newly_armed)} -- release & re-press L1 to move"
                awaiting_l1_reset = True
                l1_released_since_arm = False
                arm_triggered_this_hold = True
        else:
            dpad_arm_hold_start = None
            arm_triggered_this_hold = False

        # --- Disarm: PS tap = stop + slow return to rest; PS held 1s = force disarm ---
        if ps and not ps_prev:
            print("\n>>> PS PRESSED - STOPPING AND RETURNING TO REST <<<\n")
            joystick_states["status"] = "PS PRESSED: stopping + returning to rest (nodes stay armed)"
            ps_press_time = time.time()
            if return_thread is None or not return_thread.is_alive():
                def _return_to_rest_and_sync():
                    move_to_neutral_slowly(bus, node_ids, endpoints)
                    for nid in node_ids:
                        joint_positions[nid] = 0.0
                    if shoulder_ctrl:
                        shoulder_ctrl.value = 0.0
                    if wrist_ctrl:
                        wrist_ctrl.bend_pos = 0.0
                        wrist_ctrl.rotate_pos = 0.0
                return_thread = threading.Thread(target=_return_to_rest_and_sync, daemon=True)
                return_thread.start()
        if ps:
            if ps_press_time is not None and (time.time() - ps_press_time) >= DISARM_HOLD_SECONDS:
                force_disarm_all(bus, node_ids, endpoints)
                joystick_states["status"] = "FORCED DISARM -- TORQUE CUT IMMEDIATELY"
                ps_press_time = None
        else:
            ps_press_time = None
        ps_prev = ps

        motion_allowed = lb
        if return_thread is not None and return_thread.is_alive():
            motion_allowed = False
        if awaiting_l1_reset:
            if not lb:
                l1_released_since_arm = True
            if l1_released_since_arm and lb:
                awaiting_l1_reset = False
                joystick_states["status"] = "L1 RE-PRESSED: motion enabled"
            else:
                motion_allowed = False

        if motion_allowed:
            # Possibly controlling the normal joints or the gripper
            wrist_mode = (rb == 1)

            # Joint 2 => node3 => Right Stick X
            if 3 in node_ids:
                joint_positions[3] += taper_increment(joint_positions[3], rx * VELOCITY_SCALING * dt, JOINT2_MIN, JOINT2_MAX, DECEL_ZONE)
                if joint_positions[3] < JOINT2_MIN: joint_positions[3] = JOINT2_MIN 
                if joint_positions[3] > JOINT2_MAX: joint_positions[3] = JOINT2_MAX 
                move_odrive_to_position(bus, 3, joint_positions[3])  

            # Joint 3 => node4 => Right Stick Y
            if 4 in node_ids:
                joint_positions[4] += taper_increment(joint_positions[4], -ry * VELOCITY_SCALING * dt, JOINT3_MIN, JOINT3_MAX, DECEL_ZONE)
                if joint_positions[4] < JOINT3_MIN: joint_positions[4] = JOINT3_MIN  
                if joint_positions[4] > JOINT3_MAX: joint_positions[4] = JOINT3_MAX  
                move_odrive_to_position(bus, 4, joint_positions[4])  

            if not wrist_mode:
                # Normal left-stick => Joint 0 (node0, horizontal) & Joint 1 (node1,2, vertical)
                if 0 in node_ids:
                    joint_positions[0] += taper_increment(joint_positions[0], raw_rotate * VELOCITY_SCALING * dt, JOINT0_MIN, JOINT0_MAX, DECEL_ZONE)
                    if joint_positions[0] < JOINT0_MIN: joint_positions[0] = JOINT0_MIN
                    if joint_positions[0] > JOINT0_MAX: joint_positions[0] = JOINT0_MAX
                    move_odrive_to_position(bus, 0, joint_positions[0])

                if shoulder_ctrl and (1 in node_ids) and (2 in node_ids):
                    new_val = shoulder_ctrl.value + taper_increment(shoulder_ctrl.value, raw_bend * VELOCITY_SCALING * dt, JOINT1_MIN, JOINT1_MAX, DECEL_ZONE)
                    if new_val < JOINT1_MIN: new_val = JOINT1_MIN
                    if new_val > JOINT1_MAX: new_val = JOINT1_MAX
                    shoulder_ctrl.value = new_val
                    shoulder_ctrl.apply()

            else:
                # Wrist mode => node5,6
                if wrist_ctrl and (5 in node_ids) and (6 in node_ids):
                    # Left stick vertical => bend (tilt), horizontal => rotate (twist)
                    wrist_bend_input   = raw_bend 
                    wrist_rotate_input = raw_rotate   

                    new_bend   = wrist_ctrl.bend_pos   + taper_increment(wrist_ctrl.bend_pos, wrist_bend_input * VELOCITY_SCALING * FOREARM_VELOCITY_SCALING * dt, BEND_MIN, BEND_MAX, DECEL_ZONE_WRIST)
                    new_rotate = wrist_ctrl.rotate_pos + taper_increment(wrist_ctrl.rotate_pos, wrist_rotate_input * VELOCITY_SCALING * FOREARM_VELOCITY_SCALING * dt, ROTATE_MIN, ROTATE_MAX, DECEL_ZONE_WRIST)

                    if new_bend < BEND_MIN: new_bend = BEND_MIN
                    if new_bend > BEND_MAX: new_bend = BEND_MAX
                    if new_rotate < ROTATE_MIN: new_rotate = ROTATE_MIN
                    if new_rotate > ROTATE_MAX: new_rotate = ROTATE_MAX

                    wrist_ctrl.bend_pos   = new_bend
                    wrist_ctrl.rotate_pos = new_rotate
                    wrist_ctrl.apply()

            # Always handle the Gripper ODrive (node7) if present
            if 7 in node_ids:
                new_pos = joint_positions[7]
                # Pressing left trigger => move negative
                if raw_lt > 0: new_pos += taper_increment(new_pos, -raw_lt * VELOCITY_SCALING * GRIPPER_SCALING * dt, TRIGGER_MIN, TRIGGER_MAX, DECEL_ZONE_GRIPPER)
                # Pressing right trigger => move positive
                if raw_rt > 0: new_pos += taper_increment(new_pos, raw_rt * VELOCITY_SCALING * GRIPPER_SCALING * dt, TRIGGER_MIN, TRIGGER_MAX, DECEL_ZONE_GRIPPER)
                
                # Clamp in [TRIGGER_MIN, TRIGGER_MAX]
                if new_pos < TRIGGER_MIN: new_pos = TRIGGER_MIN
                elif new_pos > TRIGGER_MAX: new_pos = TRIGGER_MAX

                joint_positions[7] = new_pos
                move_odrive_to_position(bus, 7, joint_positions[7])

        time.sleep(0.01)

# ------------------------------------------------------------------------------
# 7) Main Function
# ------------------------------------------------------------------------------
def main():
    signal.signal(signal.SIGINT, signal_handler)

    bus = can.interface.Bus("can0", bustype = "socketcan")
    discovered = list(discover_node_ids(bus))
    endpoints  = load_endpoints()

    if not discovered:
        print("[ERROR] No ODrives found on the CAN bus.")
        return

    discovered.sort()
    print(f"Discovered ODrive Node IDs: {discovered}")

    max_id = max(discovered)
    joint_positions = [0.0] * (max_id + 1)

    # Sync: read each node's actual current position while still idle, and
    # write it back as its own target, so arming can't cause a snap toward
    # a stale/default setpoint.
    print("[INFO] Reading actual current positions...")
    for nid in discovered:
        pos = read_position(bus, nid, endpoints)
        if pos is None:
            print(f"[WARN] Could not read position for node {nid}, assuming 0.0")
            pos = 0.0
        joint_positions[nid] = pos
        move_odrive_to_position(bus, nid, pos)
        print(f"    Node {nid}: {pos:.4f}")

    # Arm with verification - do not trust that the CAN send alone means
    # the node actually entered closed-loop control.
    armed_nodes = set()
    for nid in discovered:
        if arm_node_verified(bus, nid, endpoints):
            armed_nodes.add(nid)
            print(f"[INFO] Node {nid} confirmed armed.")
        else:
            print(f"[WARN] Node {nid} did NOT arm - it will be excluded from control.")

    if not armed_nodes:
        print("[ERROR] No nodes armed successfully. Exiting.")
        bus.shutdown()
        return

    discovered = sorted(armed_nodes)

    # TEST MODE (movement bisection): no physical joystick. Injects a
    # small, time-boxed synthetic wrist-bend command (nodes 5/6) so
    # real CAN movement traffic occurs. Neutral for first 3s (confirm
    # armed+idle), small bend command 3-6s, back to neutral after.
    import time as _t
    class _NoOpJoystick:
        _t0 = _t.time()
        def init(self): pass
        def get_hat(self, idx): return (0, 0)
        def get_button(self, idx):
            if idx == DEAD_MAN_BUTTON_INDEX: return True
            if idx == MODE_TOGGLE_BUTTON_INDEX: return True
            return False
        def get_axis(self, idx):
            elapsed = _t.time() - _NoOpJoystick._t0
            if idx == AXIS_LEFT_Y and 3.0 < elapsed < 6.0:
                return 0.35
            return 0.0
        def get_name(self): return "TEST MODE - synthetic axis injection (bak_dt_clamp movement test)"
        def get_numaxes(self): return 6

    pygame.joystick.Joystick = lambda idx: _NoOpJoystick()
    pygame.joystick.get_count = lambda: 1

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("[ERROR] No joystick found.")
        pygame.quit()
        return

    stick = pygame.joystick.Joystick(0)
    stick.init()
    print(f"Joystick: {stick.get_name()}")
    print(f"# Axes: {stick.get_numaxes()}")
    print("\n[TEST MODE] Auto-armed by main(). At t=3s a small wrist-bend\n"
          "command (nodes 5/6) will run for ~3s, then return to neutral.\n")

    # Shoulder => node1,2
    shoulder_ctrl = None
    if (1 in discovered) and (2 in discovered):
        shoulder_ctrl = ShoulderController(bus, [1,2])
        shoulder_ctrl.value = (joint_positions[1] - joint_positions[2]) / 2.0
        print("[INFO] ShoulderController for node1,node2 created.")

    # Wrist => node5,6
    wrist_ctrl = None
    if (5 in discovered) and (6 in discovered):
        wrist_ctrl = WriteController(bus, [5,6])
        wrist_ctrl.rotate_pos = (joint_positions[5] + joint_positions[6]) / 2.0
        wrist_ctrl.bend_pos   = (joint_positions[5] - joint_positions[6]) / 2.0
        print("[INFO] WriteController for node5,node6 created.")

    print("\n" + "="*70)
    print("About to move slowly to the neutral rest position (matches the")
    print("robot's marked resting pose). Ensure the arm's range of motion")
    print("is clear.")
    print("="*70)
    input("Press Enter to continue...")
    startup_targets = {7: GRIPPER_STARTUP_OPEN} if 7 in discovered else None
    move_to_neutral_slowly(bus, discovered, endpoints, targets=startup_targets)
    joint_positions = [0.0] * (max_id + 1)
    if 7 in discovered:
        joint_positions[7] = GRIPPER_STARTUP_OPEN
    if shoulder_ctrl: shoulder_ctrl.value = 0.0
    if wrist_ctrl:
        wrist_ctrl.bend_pos   = 0.0
        wrist_ctrl.rotate_pos = 0.0

    metrics_text = urwid.Text("Metrics...", align = 'left')
    joystick_text = urwid.Text("", align = 'left')
    box_metrics = urwid.LineBox(metrics_text, title = "ODrive Metrics")
    box_joy     = urwid.LineBox(joystick_text, title = "Controller Inputs")

    pile = urwid.Pile([box_metrics, box_joy])
    foot = urwid.Text("Press ESC to exit", align = 'center')
    frame = urwid.Frame(pile, footer = foot)

    loop = urwid.MainLoop(
        frame,
        palette = [('reversed','standout','')],
        unhandled_input = lambda k: handle_input(k, loop, discovered, bus, joint_positions)
    )

    ui_thread = threading.Thread(
        target = update_ui_thread,
        args = (bus, discovered, endpoints, metrics_text, joystick_text, loop),
        daemon = True
    )
    ui_thread.start()

    joy_thread = threading.Thread(
        target = joystick_thread_func,
        args = (bus, discovered, joint_positions, shoulder_ctrl, wrist_ctrl, endpoints),
        daemon = True
    )
    joy_thread.start()

    try:
        loop.run()
    except KeyboardInterrupt:
        pass
    finally:
        print("[INFO] Main loop ended => stopping threads.")
        stop_event.set()
        ui_thread.join()
        joy_thread.join()
        print("[INFO] Threads joined => final shutdown.")
        clean_shutdown(discovered, bus, joint_positions, endpoints)
        pygame.quit()


if __name__ == "__main__":
    main()