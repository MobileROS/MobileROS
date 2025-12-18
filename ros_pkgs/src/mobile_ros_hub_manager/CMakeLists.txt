cmake_minimum_required(VERSION 3.0.2)
project(mobile_ros_hub_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  pluginlib
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mobile_ros_hub_manager
  CATKIN_DEPENDS roscpp nodelet pluginlib std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/hub_container.cpp
)

add_executable(${PROJECT_NAME}_node src/hub_container_node.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
