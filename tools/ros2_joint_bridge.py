#!/usr/bin/env python3
"""
ros2_joint_bridge.py

Publishes dARM's real joint positions as a standard ROS2 sensor_msgs/JointState
topic (/joint_states), so RViz (using the existing darm/ros2_pkg URDF) or
apps like Conduit can visualize the real robot.

READ-ONLY / SAFE BY DESIGN:
  - Pulls position data over the network from pos_stream_server.py's
    existing HTTP endpoint (same data the visualizer already uses) -
    does NOT touch CAN, does NOT import gamecontroller.py, does NOT
    send anything to the robot. Same safety category as
    pos_stream_server.py itself.
  - Intended to run on a SEPARATE machine from the one controlling the
    robot (e.g. DrJones's spare backup Pi with its own CAN HAT - not
    wired to the real CAN bus, isolation is the whole point) - but will
    also work run locally on the main Pi if that's more convenient for
    initial testing, since it never touches anything but the network.

Requires a working ROS2 install (rclpy) on whatever machine runs this -
not present on the main control Pi as of 2026-08-09. Not yet tested end
to end (no ROS2 environment available to test against).

NOT YET VERIFIED: the raw-position -> angle conversion below reuses the
exact same formulas as src/forward_kinematics.py's joint_angles(), which
is validated for the hand-built visualizer skeleton (checked against a
real photo 2026-07-31) - but that skeleton is a simplified re-derivation,
not a live parse of darm/ros2_pkg's actual darm.urdf.xacro. The published
angles may need a per-joint offset/sign correction once this is actually
tested against that real URDF in RViz - if a joint looks visually wrong
(twisted, backwards, offset), that's the first thing to check, not a
sign something else is broken.

Usage (once ROS2 + rclpy are available):
    source /opt/ros/<distro>/setup.bash
    python3 ros2_joint_bridge.py --host 192.168.1.181 --port 8080
"""
import argparse
import json
import math
import time
import urllib.request

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

DEG = math.pi / 180.0

# Same calibrated raw<->degree ratios as forward_kinematics.py / the
# visualizer panel defaults - keep in sync if those are ever recalibrated.
SCALE_8308 = 40.06
SCALE_SHOULDER = 41.6
SCALE_ELBOW = 40.73
SCALE_WRIST_BEND = 29.1
SCALE_WRIST_ROTATE = 27.7
SHOULDER_CAD_MAX_DEG = 226.0
ELBOW_CAD_MAX_DEG = 245.8

# Maps this project's node-based positions to the real URDF joint names
# from darm/ros2_pkg/urdf/darm.urdf.xacro. See module docstring - the
# numeric values are NOT yet verified to match that URDF's own zero/sign
# convention, only the visualizer's simplified skeleton.
def raw_to_joint_angles(pos):
    """pos: dict with string keys '0','1','3','4','5','6','7' (node ids,
    node '1' standing in for the shoulder pair) as returned by
    pos_stream_server.py's /positions endpoint. Returns
    {urdf_joint_name: angle_radians}."""
    n0 = pos.get('0', 0.0) or 0.0
    n12 = pos.get('1', 0.0) or 0.0
    n3 = pos.get('3', 0.0) or 0.0
    n4 = pos.get('4', 0.0) or 0.0
    n5 = pos.get('5', 0.0) or 0.0
    n6 = pos.get('6', 0.0) or 0.0
    n7 = pos.get('7', 0.0) or 0.0

    bend_pos = (n5 - n6) / 2.0
    rotate_pos = (n5 + n6) / 2.0

    return {
        'joint_1_joint':       n0 * SCALE_8308 * DEG,
        'joint_2_joint':       (SHOULDER_CAD_MAX_DEG + n12 * SCALE_SHOULDER) * DEG,
        'joint_3_joint':       n3 * SCALE_8308 * DEG,
        'forearm_joint':       -(ELBOW_CAD_MAX_DEG - n4 * SCALE_ELBOW) * DEG,
        'differential_joint':  bend_pos * SCALE_WRIST_BEND * DEG,
        'gripper_joint':       -rotate_pos * SCALE_WRIST_ROTATE * DEG,
        # Fingers are prismatic (meters), not angular - TRIGGER_MIN/MAX
        # from gamecontroller.py is -0.85..0.0, URDF limit is -0.001..0.025.
        # Rough linear remap pending real verification against the URDF.
        'finger_1_joint':      max(0.0, min(0.025, (n7 + 0.85) / 0.85 * 0.025)),
        'finger_2_joint':      max(0.0, min(0.025, (n7 + 0.85) / 0.85 * 0.025)),
    }


class JointBridge(Node):
    def __init__(self, host, port, rate_hz):
        super().__init__('darm_joint_bridge')
        self.url = f'http://{host}:{port}/positions'
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.get_logger().info(f'Polling {self.url} at {rate_hz}Hz, publishing /joint_states')
        self.timer = self.create_timer(1.0 / rate_hz, self.poll_and_publish)

    def poll_and_publish(self):
        try:
            with urllib.request.urlopen(self.url, timeout=1.0) as resp:
                data = json.loads(resp.read())
        except Exception as e:
            self.get_logger().warn(f'positions fetch failed: {e}', throttle_duration_sec=5)
            return

        if not data.get('_healthy', True):
            return  # pos_stream_server itself flagged its data as stale/unhealthy

        angles = raw_to_joint_angles(data)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.name = list(angles.keys())
        msg.position = list(angles.values())
        self.pub.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='192.168.1.181', help='Pi running pos_stream_server.py')
    ap.add_argument('--port', type=int, default=8080)
    ap.add_argument('--rate', type=float, default=10.0, help='publish rate, Hz')
    args = ap.parse_args()

    rclpy.init()
    node = JointBridge(args.host, args.port, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
