#!/usr/bin/env python3
from __future__ import annotations

import json
import pathlib
import sys

import rospy
from std_msgs.msg import String

REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from mobile_ros.oai import OaiMetricRecord
from mobile_ros.tasks.partition import PartitionPlanner


class PartitionNode:
    def __init__(self) -> None:
        self.deadline_ms = float(rospy.get_param("~deadline_ms", 120.0))
        self.criticality = float(rospy.get_param("~criticality", 0.5))
        self.planner = PartitionPlanner()
        self.pub = rospy.Publisher("/mobile_ros/partition_decision", String, queue_size=10)
        rospy.Subscriber("/mobile_ros/oai_metrics", String, self._metrics_cb, queue_size=10)

    def _metrics_cb(self, msg: String) -> None:
        try:
            record = OaiMetricRecord.from_mapping(json.loads(msg.data))
            decision = self.planner.decide_from_mapping(
                record.as_policy_mapping(),
                deadline_ms=self.deadline_ms,
                task_criticality=self.criticality,
            )
            self.pub.publish(json.dumps(decision.to_dict(), sort_keys=True))
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            rospy.logwarn("Ignoring invalid MobileROS metrics: %s", exc)


def main() -> None:
    rospy.init_node("mobile_ros_partition")
    PartitionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
