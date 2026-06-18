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

from mobile_ros.oai import JsonlMetricProvider, UdpMetricProvider
from mobile_ros.policies import policy_from_mapping, stream_policy_to_mapping


class MobileRosHubNode:
    def __init__(self) -> None:
        self.task_criticality = 0.5
        source = rospy.get_param("~metrics_source", "jsonl")
        if source == "udp":
            bind_host = rospy.get_param("~udp_host", "127.0.0.1")
            bind_port = int(rospy.get_param("~udp_port", 62000))
            self.provider = UdpMetricProvider((bind_host, bind_port))
        else:
            metrics_jsonl = rospy.get_param("~metrics_jsonl", str(REPO_ROOT / "benchmarks" / "oai_rfsim_metrics.jsonl"))
            self.provider = JsonlMetricProvider(metrics_jsonl, loop=True, interval_s=0.0)

        self.metrics_pub = rospy.Publisher("/mobile_ros/oai_metrics", String, queue_size=10)
        self.policy_pub = rospy.Publisher("/mobile_ros/policy", String, queue_size=10)
        rospy.Subscriber("/mobile_ros/task_criticality", Float32, self._criticality_cb)
        self.rate = rospy.Rate(float(rospy.get_param("~rate_hz", 10.0)))

    def _criticality_cb(self, msg: Float32) -> None:
        self.task_criticality = max(0.0, min(1.0, float(msg.data)))

    def spin(self) -> None:
        while not rospy.is_shutdown():
            record = self.provider.read_record()
            policy = policy_from_mapping(record.as_policy_mapping(), self.task_criticality)
            self.metrics_pub.publish(json.dumps(record.__dict__, sort_keys=True))
            self.policy_pub.publish(json.dumps(stream_policy_to_mapping(policy), sort_keys=True))
            self.rate.sleep()


def main() -> None:
    rospy.init_node("mobile_ros_hub")
    MobileRosHubNode().spin()


if __name__ == "__main__":
    main()
