#!/usr/bin/env python3
from __future__ import annotations

import json
import pathlib
import sys

import rospy
from std_msgs.msg import Float32, String

REPO_ROOT = pathlib.Path(__file__).resolve().parents[4]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from mobile_ros.policies import stream_policy_from_mapping
from mobile_ros.slicing import PolicySliceMapper, UdpSliceClient
from mobile_ros.types import TaskState, TaskType


class SliceClientNode:
    def __init__(self) -> None:
        host = rospy.get_param("~slice_host", "127.0.0.1")
        port = int(rospy.get_param("~slice_port", 63000))
        self.ue_id = rospy.get_param("~ue_id", "ue-0")
        self.task_id = rospy.get_param("~task_id", "robot_task")
        self.task_type = TaskType(rospy.get_param("~task_type", TaskType.VISUAL_SLAM.value))
        self.deadline_ms = float(rospy.get_param("~deadline_ms", 70.0))
        self.criticality = float(rospy.get_param("~criticality", 0.6))
        self.mapper = PolicySliceMapper(default_ue_id=self.ue_id)
        self.client = UdpSliceClient((host, port))
        self.pub = rospy.Publisher("/mobile_ros/slice_command", String, queue_size=10)
        rospy.Subscriber("/mobile_ros/task_criticality", Float32, self._criticality_cb, queue_size=10)
        rospy.Subscriber("/mobile_ros/policy", String, self._policy_cb, queue_size=10)

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
            self.pub.publish(command.to_json())
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            rospy.logwarn("Ignoring invalid MobileROS policy: %s", exc)


def main() -> None:
    rospy.init_node("mobile_ros_slice_client")
    SliceClientNode()
    rospy.spin()


if __name__ == "__main__":
    main()
