# MobileROS Drop-in ROS Packages

This workspace contains ROS1 catkin packages that provide network-aware drop-in cell wrappers and a Hub container capable of loading them via pluginlib/nodelets.

- `mobile_ros_cell_wrappers`: camera/imu/cmd_vel nodelets registered in `plugin.xml`.
- `mobile_ros_hub_manager`: node that acts as a nodelet manager and injects wrappers via parameters.
- `mobile_ros_cell_templates`: placeholder package for additional wrappers with TODO markers.

## Build
```
cd ros_pkgs
catkin_make
source devel/setup.bash
```

## Drop-in launch examples
- `roslaunch mobile_ros_cell_wrappers drop_in_demo.launch` (uses `nodelet manager` directly)
- `roslaunch mobile_ros_hub_manager drop_in_loader.launch` (Hub as nodelet/container loading wrappers)
