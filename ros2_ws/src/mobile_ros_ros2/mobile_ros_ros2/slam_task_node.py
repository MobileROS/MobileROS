from __future__ import annotations

import json
import pathlib
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from mobile_ros.cells import SlamElasticStreamCell
from mobile_ros.policies import stream_policy_from_mapping
from mobile_ros.types import FrameMeta, ImageFrame


class SlamTaskNode(Node):
    def __init__(self) -> None:
        super().__init__("mobile_ros_slam_task")
        self.declare_parameter("input_topic", "/camera/mobile_ros/image")
        self.declare_parameter("output_topic", "/mobile_ros/slam_features")
        self.declare_parameter("pose_delta_m", 0.08)
        self.pose_delta_m = float(self.get_parameter("pose_delta_m").value)
        self.sequence = 0
        self.cell = SlamElasticStreamCell("ros2_slam_task")
        self.pub = self.create_publisher(String, str(self.get_parameter("output_topic").value), 10)
        self.create_subscription(String, "/mobile_ros/policy", self._policy_cb, 10)
        self.create_subscription(Image, str(self.get_parameter("input_topic").value), self._image_cb, 2)

    def _policy_cb(self, msg: String) -> None:
        try:
            self.cell.update_policy(stream_policy_from_mapping(json.loads(msg.data)))
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Ignoring invalid MobileROS policy: {exc}")

    def _image_cb(self, msg: Image) -> None:
        self.sequence += 1
        payload = bytes(msg.data) if msg.data else b""
        frame = ImageFrame(
            meta=FrameMeta(msg.header.frame_id or "camera", timestamp=time.time(), sequence=self.sequence),
            width=msg.width,
            height=msg.height,
            encoding=msg.encoding,
            payload=payload,
        )
        result = self.cell.process(frame, pose_delta_m=self.pose_delta_m)
        out = String()
        out.data = json.dumps(result.to_dict(), sort_keys=True, default=str)
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = SlamTaskNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
