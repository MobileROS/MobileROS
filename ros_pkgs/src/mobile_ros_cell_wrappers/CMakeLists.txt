cmake_minimum_required(VERSION 3.0.2)
project(mobile_ros_cell_wrappers)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  pluginlib
  sensor_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mobile_ros_cell_wrappers
  CATKIN_DEPENDS roscpp nodelet pluginlib sensor_msgs geometry_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/network_aware_cell.cpp
  src/camera_cell_wrapper.cpp
  src/imu_cell_wrapper.cpp
  src/cmd_vel_cell_wrapper.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
