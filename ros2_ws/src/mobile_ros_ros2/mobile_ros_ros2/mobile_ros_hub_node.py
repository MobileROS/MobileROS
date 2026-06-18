from __future__ import annotations

import json
import pathlib
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from mobile_ros.oai import JsonlMetricProvider, UdpMetricProvider
from mobile_ros.policies import policy_from_mapping, stream_policy_to_mapping


class MobileRosHubNode(Node):
    def __init__(self) -> None:
        super().__init__("mobile_ros_hub")
        self.declare_parameter("metrics_source", "jsonl")
        self.declare_parameter("metrics_jsonl", str(REPO_ROOT / "benchmarks" / "oai_rfsim_metrics.jsonl"))
        self.declare_parameter("udp_host", "127.0.0.1")
        self.declare_parameter("udp_port", 62000)
        self.declare_parameter("rate_hz", 10.0)

        source = self.get_parameter("metrics_source").value
        if source == "udp":
            self.provider = UdpMetricProvider(
                (
                    str(self.get_parameter("udp_host").value),
                    int(self.get_parameter("udp_port").value),
                )
            )
        else:
            self.provider = JsonlMetricProvider(str(self.get_parameter("metrics_jsonl").value), loop=True)

        self.task_criticality = 0.5
        self.metrics_pub = self.create_publisher(String, "/mobile_ros/oai_metrics", 10)
        self.policy_pub = self.create_publisher(String, "/mobile_ros/policy", 10)
        self.create_subscription(Float32, "/mobile_ros/task_criticality", self._criticality_cb, 10)
        period = 1.0 / float(self.get_parameter("rate_hz").value)
        self.create_timer(period, self._tick)

    def _criticality_cb(self, msg: Float32) -> None:
        self.task_criticality = max(0.0, min(1.0, float(msg.data)))

    def _tick(self) -> None:
        record = self.provider.read_record()
        policy = policy_from_mapping(record.as_policy_mapping(), self.task_criticality)
        metrics_msg = String()
        metrics_msg.data = json.dumps(record.__dict__, sort_keys=True)
        self.metrics_pub.publish(metrics_msg)
        policy_msg = String()
        policy_msg.data = json.dumps(stream_policy_to_mapping(policy), sort_keys=True)
        self.policy_pub.publish(policy_msg)


def main() -> None:
    rclpy.init()
    node = MobileRosHubNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
