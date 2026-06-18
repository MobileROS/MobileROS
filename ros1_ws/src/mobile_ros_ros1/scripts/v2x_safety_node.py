#!/usr/bin/env python3
from __future__ import annotations

import json
import pathlib
import sys

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String

REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from mobile_ros.cells import V2XSafetyCell
from mobile_ros.policies import stream_policy_from_mapping


class V2XSafetyNode:
    def __init__(self) -> None:
        self.requested_speed_mps = float(rospy.get_param("~requested_speed_mps", 1.0))
        self.relative_speed_mps = float(rospy.get_param("~relative_speed_mps", 1.0))
        self.cell = V2XSafetyCell("ros1_v2x_safety")
        self.criticality_pub = rospy.Publisher("/mobile_ros/task_criticality", Float32, queue_size=10)
        self.decision_pub = rospy.Publisher("/mobile_ros/v2x_decision", String, queue_size=10)
        self.cmd_pub = rospy.Publisher("/cmd_vel/mobile_ros", Twist, queue_size=10)
        rospy.Subscriber("/mobile_ros/policy", String, self._policy_cb, queue_size=10)
        rospy.Subscriber("/mobile_ros/obstacle_distance", Float32, self._distance_cb, queue_size=10)

    def _policy_cb(self, msg: String) -> None:
        try:
            self.cell.update_policy(stream_policy_from_mapping(json.loads(msg.data)))
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            rospy.logwarn("Ignoring invalid MobileROS policy: %s", exc)

    def _distance_cb(self, msg: Float32) -> None:
        decision = self.cell.decide(float(msg.data), self.requested_speed_mps, self.relative_speed_mps)
        self.criticality_pub.publish(Float32(data=decision.criticality))
        twist = Twist()
        twist.linear.x = decision.target_speed_mps
        self.cmd_pub.publish(twist)
        self.decision_pub.publish(json.dumps(decision.__dict__, sort_keys=True))


def main() -> None:
    rospy.init_node("mobile_ros_v2x_safety")
    V2XSafetyNode()
    rospy.spin()


if __name__ == "__main__":
    main()
