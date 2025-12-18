#include "mobile_ros_hub_manager/hub_container.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mobile_ros_hub_container");
  mobile_ros_hub_manager::HubContainer container;
  container.spin();
  return 0;
}
