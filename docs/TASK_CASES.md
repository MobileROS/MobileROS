# Task cases

This document records where each case comes from and what MobileROS implements.

## Visual SLAM

- Backend: ORB-SLAM3, installed with `third_party/install_orb_slam3.sh`.
- MobileROS code: `mobile_ros/tasks/visual_slam.py`,
  `mobile_ros/cells.py`, ROS `slam_task_node`.
- Function: adapt camera rate/resolution/JPEG policy, emit keyframe/features
  status, and expose a command builder for ORB-SLAM3.

## LiDAR Perception

- Backend: OpenPCDet, installed with `third_party/install_openpcdet.sh`.
- MobileROS code: `mobile_ros/tasks/lidar_perception.py`, ROS
  `lidar_cell_node`.
- Function: adaptive voxel filtering based on OAI bandwidth/latency metrics.

## V2X Safety

- Backend: ROS command topics and robot controller.
- MobileROS code: `mobile_ros/tasks/v2x.py`, ROS `v2x_safety_node`.
- Function: raise task criticality and gate velocity when obstacle distance and
  link condition require a conservative action.

## Multi-Robot Map Sharing

- Backend: map fragments from the robot SLAM stack.
- MobileROS code: `mobile_ros/tasks/multi_robot.py`.
- Function: schedule map fragment packaging and priority across robot peers.

## Partitioned Pipeline

- Backend: local robot compute plus edge/cloud process manager.
- MobileROS code: `mobile_ros/tasks/partition.py`, ROS `partition_node`.
- Function: decide which perception stages stay local and which can run at the
  edge under the current link.

## Radio Resource Path

- Backend: OpenAirInterface v2.1.0 with RF simulator or USRP B210/X410.
- MobileROS code: `mobile_ros/oai.py`, `mobile_ros/slicing.py`,
  `oai_integration/hooks/`, ROS `slice_client_node`.
- Function: consume OAI metrics, build policies, and send slice commands.
