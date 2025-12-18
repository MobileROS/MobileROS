#include "mobile_ros_cell_wrappers/cmd_vel_cell_wrapper.hpp"
#include <pluginlib/class_list_macros.h>

namespace mobile_ros_cell_wrappers
{
void CmdVelCellWrapper::setup()
{
  std::string input_topic = private_nh_.param<std::string>("input_topic", "/cmd_vel");
  std::string output_topic = private_nh_.param<std::string>("output_topic", "/mobile_ros/network_cmd_vel");
  cmd_sub_ = nh_.subscribe(input_topic, 5, &CmdVelCellWrapper::cmdVelCallback, this);
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(output_topic, 5);
  logNetworkProfile("cmd_vel wrapper active");
}

void CmdVelCellWrapper::cmdVelCallback(const geometry_msgs::TwistConstPtr &msg)
{
  geometry_msgs::Twist forwarded = *msg;
  cmd_pub_.publish(forwarded);
}
}  // namespace mobile_ros_cell_wrappers

PLUGINLIB_EXPORT_CLASS(mobile_ros_cell_wrappers::CmdVelCellWrapper, nodelet::Nodelet)

