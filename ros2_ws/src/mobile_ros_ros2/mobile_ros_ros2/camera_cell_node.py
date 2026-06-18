from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class CameraCellNode(Node):
    def __init__(self) -> None:
        super().__init__("mobile_ros_camera_cell")
        self.declare_parameter("input_topic", "/camera/image_raw")
        self.declare_parameter("output_topic", "/camera/mobile_ros/image")
        self.policy = {"publish_rate_hz": 10.0, "resolution_scale": 1.0, "jpeg_quality": 95}
        self.last_publish = 0.0
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.pub = self.create_publisher(Image, output_topic, 2)
        self.create_subscription(String, "/mobile_ros/policy", self._policy_cb, 10)
        self.create_subscription(Image, input_topic, self._image_cb, 2)

    def _policy_cb(self, msg: String) -> None:
        try:
            self.policy = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("Ignoring invalid MobileROS policy JSON")

    def _image_cb(self, msg: Image) -> None:
        publish_rate = max(0.1, float(self.policy.get("publish_rate_hz", 10.0)))
        now = time.time()
        if now - self.last_publish < 1.0 / publish_rate:
            return
        self.last_publish = now
        msg.header.frame_id = f"{msg.header.frame_id}|mobile_ros:q={self.policy.get('jpeg_quality', 95)}"
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = CameraCellNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
