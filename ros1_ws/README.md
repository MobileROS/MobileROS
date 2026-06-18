# MobileROS ROS 1 workspace

This workspace provides ROS 1 wrappers around the shared `mobile_ros` Python runtime.
It targets ROS Noetic on Ubuntu 20.04 and can run with OAI RF simulator logs, UDP
metrics, or USRP-backed OAI metric exporters.

## Build

```bash
cd ros1_ws
catkin_make
source devel/setup.bash
```

## SLAM elastic stream demo

```bash
roslaunch mobile_ros_ros1 slam_elastic_stream.launch metrics_jsonl:=$(pwd)/../benchmarks/oai_rfsim_metrics.jsonl
```

The launch file wraps a camera topic with MobileROS policy updates. Replace the
input topic with the ORB-SLAM3 camera feed in the robot launch file.
