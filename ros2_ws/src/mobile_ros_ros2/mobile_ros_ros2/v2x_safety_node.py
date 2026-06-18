from __future__ import annotations

import json
import pathlib
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32, String

REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from mobile_ros.cells import V2XSafetyCell
from mobile_ros.policies import stream_policy_from_mapping


class V2XSafetyNode(Node):
    def __init__(self) -> None:
        super().__init__("mobile_ros_v2x_safety")
        self.declare_parameter("requested_speed_mps", 1.0)
        self.declare_parameter("relative_speed_mps", 1.0)
        self.requested_speed_mps = float(self.get_parameter("requested_speed_mps").value)
        self.relative_speed_mps = float(self.get_parameter("relative_speed_mps").value)
        self.cell = V2XSafetyCell("ros2_v2x_safety")
        self.criticality_pub = self.create_publisher(Float32, "/mobile_ros/task_criticality", 10)
        self.decision_pub = self.create_publisher(String, "/mobile_ros/v2x_decision", 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel/mobile_ros", 10)
        self.create_subscription(String, "/mobile_ros/policy", self._policy_cb, 10)
        self.create_subscription(Float32, "/mobile_ros/obstacle_distance", self._distance_cb, 10)

    def _policy_cb(self, msg: String) -> None:
        try:
            self.cell.update_policy(stream_policy_from_mapping(json.loads(msg.data)))
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Ignoring invalid MobileROS policy: {exc}")

    def _distance_cb(self, msg: Float32) -> None:
        decision = self.cell.decide(float(msg.data), self.requested_speed_mps, self.relative_speed_mps)
        criticality = Float32()
        criticality.data = decision.criticality
        self.criticality_pub.publish(criticality)
        twist = Twist()
        twist.linear.x = decision.target_speed_mps
        self.cmd_pub.publish(twist)
        out = String()
        out.data = json.dumps(decision.__dict__, sort_keys=True)
        self.decision_pub.publish(out)


def main() -> None:
    rclpy.init()
    node = V2XSafetyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
