#!/usr/bin/env python3
from __future__ import annotations

import json
import pathlib
import sys
import time

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from mobile_ros.cells import SlamElasticStreamCell
from mobile_ros.policies import stream_policy_from_mapping
from mobile_ros.types import FrameMeta, ImageFrame


class SlamTaskNode:
    def __init__(self) -> None:
        self.input_topic = rospy.get_param("~input_topic", "/camera/mobile_ros/image")
        self.output_topic = rospy.get_param("~output_topic", "/mobile_ros/slam_features")
        self.pose_delta_m = float(rospy.get_param("~pose_delta_m", 0.08))
        self.sequence = 0
        self.cell = SlamElasticStreamCell("ros1_slam_task")
        self.pub = rospy.Publisher(self.output_topic, String, queue_size=10)
        rospy.Subscriber("/mobile_ros/policy", String, self._policy_cb, queue_size=10)
        rospy.Subscriber(self.input_topic, Image, self._image_cb, queue_size=2)

    def _policy_cb(self, msg: String) -> None:
        try:
            self.cell.update_policy(stream_policy_from_mapping(json.loads(msg.data)))
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            rospy.logwarn("Ignoring invalid MobileROS policy: %s", exc)

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
        self.pub.publish(json.dumps(result.to_dict(), sort_keys=True, default=str))


def main() -> None:
    rospy.init_node("mobile_ros_slam_task")
    SlamTaskNode()
    rospy.spin()


if __name__ == "__main__":
    main()
