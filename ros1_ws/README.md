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

## Full-stack demo

```bash
roslaunch mobile_ros_ros1 full_stack.launch metrics_jsonl:=$(pwd)/../benchmarks/oai_rfsim_metrics.jsonl
```

The launch file starts Hub, Camera, SLAM feature stream, LiDAR adaptive voxel,
slice client, V2X safety, and partition nodes. Replace the camera and point
cloud input topics with the robot task topics.
