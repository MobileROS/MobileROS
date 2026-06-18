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

from mobile_ros.policies import stream_policy_from_mapping
from mobile_ros.slicing import PolicySliceMapper, UdpSliceClient
from mobile_ros.types import TaskState, TaskType


class SliceClientNode(Node):
    def __init__(self) -> None:
        super().__init__("mobile_ros_slice_client")
        self.declare_parameter("slice_host", "127.0.0.1")
        self.declare_parameter("slice_port", 63000)
        self.declare_parameter("ue_id", "ue-0")
        self.declare_parameter("task_id", "robot_task")
        self.declare_parameter("task_type", TaskType.VISUAL_SLAM.value)
        self.declare_parameter("deadline_ms", 70.0)
        self.declare_parameter("criticality", 0.6)
        self.ue_id = str(self.get_parameter("ue_id").value)
        self.task_id = str(self.get_parameter("task_id").value)
        self.task_type = TaskType(str(self.get_parameter("task_type").value))
        self.deadline_ms = float(self.get_parameter("deadline_ms").value)
        self.criticality = float(self.get_parameter("criticality").value)
        self.mapper = PolicySliceMapper(default_ue_id=self.ue_id)
        self.client = UdpSliceClient(
            (
                str(self.get_parameter("slice_host").value),
                int(self.get_parameter("slice_port").value),
            )
        )
        self.pub = self.create_publisher(String, "/mobile_ros/slice_command", 10)
        self.create_subscription(Float32, "/mobile_ros/task_criticality", self._criticality_cb, 10)
        self.create_subscription(String, "/mobile_ros/policy", self._policy_cb, 10)

    def _criticality_cb(self, msg: Float32) -> None:
        self.criticality = max(0.0, min(1.0, float(msg.data)))

    def _policy_cb(self, msg: String) -> None:
        try:
            policy = stream_policy_from_mapping(json.loads(msg.data))
            task_state = TaskState(
                self.task_id,
                task_type=self.task_type,
                criticality=self.criticality,
                deadline_ms=self.deadline_ms,
                metadata={"ue_id": self.ue_id},
            )
            command = self.mapper.map_policy(policy, task_state, ue_id=self.ue_id)
            self.client.send(command)
            out = String()
            out.data = command.to_json()
            self.pub.publish(out)
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Ignoring invalid MobileROS policy: {exc}")


def main() -> None:
    rclpy.init()
    node = SliceClientNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
