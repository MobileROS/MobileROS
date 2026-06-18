from __future__ import annotations

import json
import pathlib
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

try:
    from sensor_msgs_py import point_cloud2 as pc2
except Exception:  # pragma: no cover - ROS optional path
    pc2 = None

REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from mobile_ros.cells import LidarAdaptiveCell
from mobile_ros.policies import stream_policy_from_mapping
from mobile_ros.types import FrameMeta, PointCloudFrame


class LidarCellNode(Node):
    def __init__(self) -> None:
        super().__init__("mobile_ros_lidar_cell")
        self.declare_parameter("input_topic", "/points_raw")
        self.declare_parameter("output_topic", "/points/mobile_ros")
        self.declare_parameter("max_points", 20000)
        self.max_points = int(self.get_parameter("max_points").value)
        self.sequence = 0
        self.cell = LidarAdaptiveCell("ros2_lidar_cell")
        self.pub = self.create_publisher(PointCloud2, str(self.get_parameter("output_topic").value), 2)
        self.stats_pub = self.create_publisher(String, "/mobile_ros/lidar_stats", 10)
        self.create_subscription(String, "/mobile_ros/policy", self._policy_cb, 10)
        self.create_subscription(PointCloud2, str(self.get_parameter("input_topic").value), self._cloud_cb, 2)

    def _policy_cb(self, msg: String) -> None:
        try:
            self.cell.update_policy(stream_policy_from_mapping(json.loads(msg.data)))
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Ignoring invalid MobileROS policy: {exc}")

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
        stats = String()
        stats.data = json.dumps(result.to_dict(), sort_keys=True, default=str)
        self.stats_pub.publish(stats)


def main() -> None:
    rclpy.init()
    node = LidarCellNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
