cmake_minimum_required(VERSION 3.0.2)
project(mobile_ros_ros1)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs sensor_msgs geometry_msgs)
catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/mobile_ros_hub_node.py
    scripts/camera_cell_node.py
    scripts/slam_task_node.py
    scripts/lidar_cell_node.py
    scripts/slice_client_node.py
    scripts/v2x_safety_node.py
    scripts/partition_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
