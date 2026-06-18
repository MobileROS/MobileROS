#!/usr/bin/env python3
from __future__ import annotations

import json
import pathlib
import sys
import time

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

try:
    import sensor_msgs.point_cloud2 as pc2
except Exception:  # pragma: no cover - ROS optional path
    pc2 = None

REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from mobile_ros.cells import LidarAdaptiveCell
from mobile_ros.policies import stream_policy_from_mapping
from mobile_ros.types import FrameMeta, PointCloudFrame


class LidarCellNode:
    def __init__(self) -> None:
        self.input_topic = rospy.get_param("~input_topic", "/points_raw")
        self.output_topic = rospy.get_param("~output_topic", "/points/mobile_ros")
        self.max_points = int(rospy.get_param("~max_points", 20000))
        self.sequence = 0
        self.cell = LidarAdaptiveCell("ros1_lidar_cell")
        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=2)
        self.stats_pub = rospy.Publisher("/mobile_ros/lidar_stats", String, queue_size=10)
        rospy.Subscriber("/mobile_ros/policy", String, self._policy_cb, queue_size=10)
        rospy.Subscriber(self.input_topic, PointCloud2, self._cloud_cb, queue_size=2)

    def _policy_cb(self, msg: String) -> None:
        try:
            self.cell.update_policy(stream_policy_from_mapping(json.loads(msg.data)))
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            rospy.logwarn("Ignoring invalid MobileROS policy: %s", exc)

    def _cloud_to_frame(self, msg: PointCloud2) -> PointCloudFrame:
        self.sequence += 1
        points = []
        if pc2 is not None:
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append((float(point[0]), float(point[1]), float(point[2])))
                if len(points) >= self.max_points:
                    break
        return PointCloudFrame(
            meta=FrameMeta(msg.header.frame_id or "lidar", timestamp=time.time(), sequence=self.sequence),
            points=tuple(points),
        )

    def _cloud_cb(self, msg: PointCloud2) -> None:
        if pc2 is None:
            self.pub.publish(msg)
            return
        frame = self._cloud_to_frame(msg)
        result = self.cell.process(frame)
        if result.accepted:
            out = pc2.create_cloud_xyz32(msg.header, list(result.output.points))
            self.pub.publish(out)
        self.stats_pub.publish(json.dumps(result.to_dict(), sort_keys=True, default=str))


def main() -> None:
    rospy.init_node("mobile_ros_lidar_cell")
    LidarCellNode()
    rospy.spin()


if __name__ == "__main__":
    main()
