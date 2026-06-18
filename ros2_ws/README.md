# MobileROS ROS 2 workspace

This workspace provides ROS 2 wrappers around the shared `mobile_ros` runtime.
It targets ROS 2 Foxy and Humble. The wrappers publish the same policy JSON used
by the ROS 1 package so experiment logic remains consistent across both versions.

## Build

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Full-stack demo

```bash
ros2 launch mobile_ros_ros2 full_stack.launch.py metrics_jsonl:=$(pwd)/../benchmarks/oai_rfsim_metrics.jsonl
```

The launch file starts Hub, Camera, SLAM feature stream, LiDAR adaptive voxel,
slice client, V2X safety, and partition nodes. Use `/camera/mobile_ros/image`
as the camera input for ORB-SLAM3 or another visual SLAM frontend.
