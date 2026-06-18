#!/usr/bin/env python3
from __future__ import annotations

import json
import time

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String


class CameraCellNode:
    def __init__(self) -> None:
        self.input_topic = rospy.get_param("~input_topic", "/camera/image_raw")
        self.output_topic = rospy.get_param("~output_topic", "/camera/mobile_ros/image")
        self.policy = {"publish_rate_hz": 10.0, "resolution_scale": 1.0, "jpeg_quality": 95}
        self.last_publish = 0.0
        self.pub = rospy.Publisher(self.output_topic, Image, queue_size=2)
        rospy.Subscriber("/mobile_ros/policy", String, self._policy_cb)
        rospy.Subscriber(self.input_topic, Image, self._image_cb, queue_size=2)

    def _policy_cb(self, msg: String) -> None:
        try:
            self.policy = json.loads(msg.data)
        except json.JSONDecodeError:
            rospy.logwarn("Ignoring invalid MobileROS policy JSON")

    def _image_cb(self, msg: Image) -> None:
        publish_rate = max(0.1, float(self.policy.get("publish_rate_hz", 10.0)))
        now = time.time()
        if now - self.last_publish < 1.0 / publish_rate:
            return
        self.last_publish = now
        msg.header.frame_id = f"{msg.header.frame_id}|mobile_ros:q={self.policy.get('jpeg_quality', 95)}"
        self.pub.publish(msg)


def main() -> None:
    rospy.init_node("mobile_ros_camera_cell")
    CameraCellNode()
    rospy.spin()


if __name__ == "__main__":
    main()
