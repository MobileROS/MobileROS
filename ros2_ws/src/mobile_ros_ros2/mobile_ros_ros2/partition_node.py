from __future__ import annotations

import json
import pathlib
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from mobile_ros.oai import OaiMetricRecord
from mobile_ros.tasks.partition import PartitionPlanner


class PartitionNode(Node):
    def __init__(self) -> None:
        super().__init__("mobile_ros_partition")
        self.declare_parameter("deadline_ms", 120.0)
        self.declare_parameter("criticality", 0.5)
        self.deadline_ms = float(self.get_parameter("deadline_ms").value)
        self.criticality = float(self.get_parameter("criticality").value)
        self.planner = PartitionPlanner()
        self.pub = self.create_publisher(String, "/mobile_ros/partition_decision", 10)
        self.create_subscription(String, "/mobile_ros/oai_metrics", self._metrics_cb, 10)

    def _metrics_cb(self, msg: String) -> None:
        try:
            record = OaiMetricRecord.from_mapping(json.loads(msg.data))
            decision = self.planner.decide_from_mapping(
                record.as_policy_mapping(),
                deadline_ms=self.deadline_ms,
                task_criticality=self.criticality,
            )
            out = String()
            out.data = json.dumps(decision.to_dict(), sort_keys=True)
            self.pub.publish(out)
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Ignoring invalid MobileROS metrics: {exc}")


def main() -> None:
    rclpy.init()
    node = PartitionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
