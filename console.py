#!/usr/bin/env python3

import time
import can
import urwid
import signal
from threading import Thread

from src.can_utils import discover_node_ids
from src.control import move_odrive_to_position, set_closed_loop_control, set_idle_mode
from src.metrics import get_metrics, METRIC_ENDPOINTS
from src.configure import load_endpoints, read_config, write_config

# Step increments
INCREMENT_DEFAULT = 0.1
INCREMENT_GRIPPER = 0.01

def read_position(bus, node_id, endpoints):
    """
    Reads a node's actual current position (axis0.pos_estimate).
    Used at startup so we can sync the ODrive's own setpoint to reality
    before arming closed-loop control, instead of jumping to a stale 0.
    """
    ep = endpoints['endpoints']['axis0.pos_estimate']
    return read_config(bus, node_id, ep['id'], ep['type'])

def arm_node_verified(bus, node_id, endpoints, retries=5, settle_timeout=0.3):
    """
    Requests CLOSED_LOOP_CONTROL and confirms via axis0.is_armed that the
    node actually latched into it, instead of trusting that the CAN send
    succeeded. A busy bus (every node broadcasting telemetry every 10ms)
    can occasionally drop or delay the request, so we retry a few times.
    Returns True only once the node confirms it is actually armed.
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

# Node5/6 raw safety envelope, measured empirically from a combined
# tilt+rotation test, plus a large safety margin. This is the
# authoritative clamp applied in ForearmController.apply_forearm_values()
# - the unison/diff slider bounds below are only soft bounds on the UI
# value and are NOT sufficient on their own to guarantee this range.
MOTOR5_MIN, MOTOR5_MAX = -28.41, 6.06
MOTOR6_MIN, MOTOR6_MAX = -27.36, 4.57

class ShoulderController:
    """
    Controls two ODrives on the same joint, with motors facing opposite directions.
    A single "shoulder_val" from the slider means motorA -> +val, motorB -> -val.
    """
    def __init__(self, bus, node_id_pair, endpoints, initial_val=0.0):
        self.bus          = bus
        self.node_id_pair = node_id_pair   # e.g. [1, 2]
        self.endpoints    = endpoints
        self.shoulder_val = initial_val

    def apply_shoulder_values(self):
        nodeA, nodeB  = self.node_id_pair
        motorA_target = -self.shoulder_val
        motorB_target = self.shoulder_val

        move_odrive_to_position(self.bus, nodeA, motorA_target)
        move_odrive_to_position(self.bus, nodeB, motorB_target)

class ForearmController:
    """
    Controller for a forearm that has "unison" and "diff" sliders (e.g. node [5, 6]).
    """
    def __init__(self, bus, node_id_pair, endpoints, initial_unison=0.0, initial_diff=0.0):
        self.bus          = bus
        self.node_id_pair = node_id_pair
        self.endpoints    = endpoints
        self.unison_val   = initial_unison
        self.diff_val     = initial_diff

    def apply_forearm_values(self):
        nodeA, nodeB  = self.node_id_pair
        motorA_target = self.unison_val + self.diff_val
        motorB_target = self.unison_val - self.diff_val

        # Authoritative raw safety clamp - see MOTOR5/6_MIN/MAX definition.
        motorA_target = max(MOTOR5_MIN, min(MOTOR5_MAX, motorA_target))
        motorB_target = max(MOTOR6_MIN, min(MOTOR6_MAX, motorB_target))

        move_odrive_to_position(self.bus, nodeA, motorA_target)
        move_odrive_to_position(self.bus, nodeB, motorB_target)

class ODriveSlider(urwid.WidgetWrap):
    """
    General slider for controlling:
      - A single ODrive (node_ids = [X]),
      - A pair of ODrives in a special mode (shoulder or forearm).
    """
    def __init__(self, node_ids, bus, endpoints, min_val, max_val,
                 step_size=INCREMENT_DEFAULT,  # <--- NEW
                 shared_forearm=None, forearm_mode=None,
                 shared_shoulder=None, initial_value=0.0):
        self.node_ids  = node_ids
        self.bus       = bus
        self.endpoints = endpoints
        self.min_val   = min_val
        self.max_val   = max_val
        self.value     = initial_value
        self.step_size       = step_size        # Step size (default = 0.1) or custom if specified
        self.shared_forearm  = shared_forearm
        self.forearm_mode    = forearm_mode     # 'unison', 'diff', or None
        self.shared_shoulder = shared_shoulder  # If not None => shoulder joint

        # Decide a label based on the mode
        if self.shared_shoulder:                                    label_text = f"Shoulder (Nodes {node_ids[0]}, {node_ids[1]}): {self.value:.1f}"
        elif self.shared_forearm and self.forearm_mode == 'unison': label_text = f"Forearm UNISON: {self.value:.1f}"
        elif self.shared_forearm and self.forearm_mode == 'diff':   label_text = f"Forearm DIFF: {self.value:.1f}"
        else:                                                       label_text = f"ODrive {', '.join(map(str, self.node_ids))}: {self.value:.1f}"

        self.label = urwid.Text(label_text)
        self.pile  = urwid.Pile([self.label])
        super().__init__(urwid.AttrMap(self.pile, None, focus_map='reversed'))

    def update_value(self, increment):
        self.value = max(self.min_val, min(self.max_val, self.value + increment))

        # Update label
        if self.shared_shoulder:                                    self.label.set_text(f"Shoulder (Nodes {self.node_ids[0]}, {self.node_ids[1]}): {self.value:.1f}")
        elif self.shared_forearm and self.forearm_mode == 'unison': self.label.set_text(f"Forearm UNISON: {self.value:.1f}")
        elif self.shared_forearm and self.forearm_mode == 'diff':   self.label.set_text(f"Forearm DIFF: {self.value:.1f}")
        else:                                                       self.label.set_text(f"ODrive {', '.join(map(str, self.node_ids))}: {self.value:.1f}")

        self.move_motor()

    def move_motor(self):
        # If forearm, set unison/diff
        if self.shared_forearm and self.forearm_mode in ['unison', 'diff']:
            if self.forearm_mode == 'unison': self.shared_forearm.unison_val = self.value
            else:                             self.shared_forearm.diff_val   = self.value
            self.shared_forearm.apply_forearm_values()
        # If shoulder, set the single "shoulder_val"
        elif self.shared_shoulder:
            self.shared_shoulder.shoulder_val = self.value
            self.shared_shoulder.apply_shoulder_values()
        else:
            # Single or normal pair
            for node_id in self.node_ids:
                move_odrive_to_position(self.bus, node_id, self.value)

def clean_shutdown(node_ids, bus, endpoints):
    print("\nExiting... resetting ODrives to position 0 and setting them to idle.")

    vel_ep   = endpoints['endpoints']['axis0.trap_traj.config.vel_limit']
    accel_ep = endpoints['endpoints']['axis0.trap_traj.config.accel_limit']
    decel_ep = endpoints['endpoints']['axis0.trap_traj.config.decel_limit']

    for nd in node_ids:
        vel   = read_config(bus, nd, vel_ep['id'], vel_ep['type'])
        accel = read_config(bus, nd, accel_ep['id'], accel_ep['type'])
        decel = read_config(bus, nd, decel_ep['id'], decel_ep['type'])
        if vel is not None:   write_config(bus, nd, vel_ep['id'], vel_ep['type'], vel / 2)
        if accel is not None: write_config(bus, nd, accel_ep['id'], accel_ep['type'], accel / 2)
        if decel is not None: write_config(bus, nd, decel_ep['id'], decel_ep['type'], decel / 2)

    for nd in node_ids: move_odrive_to_position(bus, nd, 0)
    time.sleep(4)
    for nd in node_ids: set_idle_mode(bus, nd)
    if bus: bus.shutdown()

def signal_handler(sig, frame):
    raise KeyboardInterrupt

def handle_input(key, columns, sliders, node_ids, bus, endpoints):
    if key == 'esc':
        clean_shutdown(node_ids, bus, endpoints)
        raise urwid.ExitMainLoop()

    elif key in ('up', 'down'):
        # Adjust the slider value
        focus  = columns.focus_position
        slider = sliders[focus]

        increment = slider.step_size
        if key == 'down': increment = -increment

        slider.update_value(increment)

    elif key in ('left', 'right'):
        # Navigate between sliders
        if key == 'right' and columns.focus_position < len(sliders) - 1: columns.focus_position += 1
        elif key == 'left' and columns.focus_position > 0:               columns.focus_position -= 1

def update_metrics_textbox(bus, node_ids, endpoints, metrics_text, loop):
    column_widths = {
        metric: max(len(metric), 4) + 3
        for metric in METRIC_ENDPOINTS.keys()
    }
    node_col_width = 6

    # Build the header line
    header = f"{'Node':<{node_col_width}}" + "".join(
        f"{name:<{column_widths[name]}}" for name in METRIC_ENDPOINTS.keys()
    )

    while True:
        lines = [header]
        for nd in node_ids:
            metrics = get_metrics(bus, nd, endpoints)
            line    = f"{nd:<{node_col_width}}"
            for metric, val in metrics.items():
                if isinstance(val, (float, int)):
                    sign_space    = ' ' if val >= 0 else ''
                    formatted_val = f"{sign_space}{val:.2f}"
                    line         += f"{formatted_val:<{column_widths[metric]}}"
                else:
                    # If None or non-numeric => show "None"
                    line += f"{'None':<{column_widths[metric]}}"
            lines.append(line)

        metrics_text.set_text("\n".join(lines))
        time.sleep(0.1)
        loop.draw_screen()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    bus       = can.interface.Bus("can0", bustype="socketcan")
    node_ids  = list(discover_node_ids(bus))
    endpoints = load_endpoints()

    if not node_ids:
        print("No ODrives detected on the CAN network. Exiting.")
        return

    node_ids.sort()
    print(f"Detected ODrive Node IDs: {node_ids}")

    # Read each node's actual current position and sync the ODrive's own
    # setpoint to match BEFORE arming closed-loop control, so nothing jumps
    # toward a stale setpoint (e.g. leftover/default 0) the instant it's
    # armed. This is done while nodes are still idle, so no motion occurs.
    current_pos = {}
    for nd in node_ids:
        pos = read_position(bus, nd, endpoints)
        current_pos[nd] = pos if pos is not None else 0.0
        move_odrive_to_position(bus, nd, current_pos[nd])
    time.sleep(0.1)
    print(f"Synced starting positions: {current_pos}")

    # Arm each discovered node, confirming via axis0.is_armed rather than
    # trusting the CAN send alone (a busy bus can drop/delay the request).
    armed_nodes = set()
    for nd in node_ids:
        if arm_node_verified(bus, nd, endpoints):
            armed_nodes.add(nd)
            print(f"[INFO] Node {nd} confirmed armed.")
        else:
            print(f"[WARN] Node {nd} did NOT arm - it will be excluded from control.")

    if not armed_nodes:
        print("[ERROR] No nodes armed successfully. Exiting.")
        bus.shutdown()
        return

    sliders = []

    # Single slider for node 0
    if 0 in armed_nodes:
        sliders.append(ODriveSlider([0], bus, endpoints, -8.22, 7.91, initial_value=current_pos.get(0, 0.0)))

    # Shoulder: node 1,2 - only build this slider if BOTH motors of the pair
    # armed. Driving one side of a mirrored joint while the other doesn't
    # resist/move is exactly the kind of mismatched load to avoid.
    if 1 in armed_nodes and 2 in armed_nodes:
        shoulder_initial = (current_pos.get(2, 0.0) - current_pos.get(1, 0.0)) / 2.0
        shoulder_ctrl = ShoulderController(bus, [1, 2], endpoints, initial_val=shoulder_initial)
        slider_shoulder = ODriveSlider(
            [1, 2],
            bus,
            endpoints,
            min_val         = 0.0,
            max_val         = 5.43,
            step_size       = INCREMENT_DEFAULT,     # shoulder uses default 0.1
            shared_shoulder = shoulder_ctrl,
            initial_value   = shoulder_initial
        )
        sliders.append(slider_shoulder)
    elif (1 in armed_nodes) != (2 in armed_nodes):
        print("[WARN] Shoulder pair (nodes 1,2) only partially armed - shoulder slider skipped for safety.")

    # Single slider for node 3
    if 3 in armed_nodes:
        sliders.append(ODriveSlider([3], bus, endpoints, -8.38, 12.89, initial_value=current_pos.get(3, 0.0)))

    # Single slider for node 4
    if 4 in armed_nodes:
        sliders.append(ODriveSlider([4], bus, endpoints, 0.0, 5.76, initial_value=current_pos.get(4, 0.0)))

    # Forearm: node 5,6 => unison/diff - only build if BOTH motors armed,
    # same reasoning as the shoulder pair above.
    if 5 in armed_nodes and 6 in armed_nodes:
        unison_initial = (current_pos.get(5, 0.0) + current_pos.get(6, 0.0)) / 2.0
        diff_initial   = (current_pos.get(5, 0.0) - current_pos.get(6, 0.0)) / 2.0
        forearm_ctrl = ForearmController(bus, [5, 6], endpoints, initial_unison=unison_initial, initial_diff=diff_initial)
        slider_unison = ODriveSlider(
            [5, 6],
            bus,
            endpoints,
            min_val        = -10.0,
            max_val        =  10.0,
            step_size      = INCREMENT_DEFAULT,     # forearm uses default 0.1
            shared_forearm = forearm_ctrl,
            forearm_mode   = 'unison',
            initial_value  = unison_initial
        )
        slider_diff = ODriveSlider(
            [5, 6],
            bus,
            endpoints,
            min_val        = -5.0,
            max_val        =  5.0,
            step_size      = INCREMENT_DEFAULT,
            shared_forearm = forearm_ctrl,
            forearm_mode   = 'diff',
            initial_value  = diff_initial
        )
        sliders.append(slider_unison)
        sliders.append(slider_diff)
    elif (5 in armed_nodes) != (6 in armed_nodes):
        print("[WARN] Forearm pair (nodes 5,6) only partially armed - forearm sliders skipped for safety.")

    # Gripper (node 7, 5208): smaller increments
    if 7 in armed_nodes:
        # Example range might be narrower if the gripper doesn't move as far
        sliders.append(ODriveSlider(
            [7],
            bus,
            endpoints,
            min_val       = 0.0,
            max_val       = 0.80,
            step_size     = INCREMENT_GRIPPER,
            initial_value = current_pos.get(7, 0.0)
        ))

    columns      = urwid.Columns([urwid.LineBox(s) for s in sliders])
    metrics_text = urwid.Text("Fetching metrics...", align='left')
    pile         = urwid.Pile([columns, metrics_text])
    frame = urwid.Frame(
        urwid.Filler(pile, valign = 'top'),
        footer = urwid.Text("Press ESC to exit | Up/Down to change value | Left/Right to switch slider", align = 'center')
    )

    loop = urwid.MainLoop(
        frame,
        palette         = [('reversed', 'standout', '')],
        unhandled_input = lambda k: handle_input(k, columns, sliders, node_ids, bus, endpoints)
    )

    # Start metrics update thread
    Thread(
        target = update_metrics_textbox,
        args   = (bus, node_ids, endpoints, metrics_text, loop),
        daemon = True
    ).start()

    try:
        loop.run()
    except KeyboardInterrupt:
        clean_shutdown(node_ids, bus, endpoints)
    finally:
        if bus: bus.shutdown()

if __name__ == "__main__":
    main()
