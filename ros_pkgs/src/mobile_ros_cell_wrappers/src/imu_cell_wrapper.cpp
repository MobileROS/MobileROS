#include "mobile_ros_cell_wrappers/imu_cell_wrapper.hpp"
#include <pluginlib/class_list_macros.h>

namespace mobile_ros_cell_wrappers
{
void ImuCellWrapper::setup()
{
  std::string input_topic = private_nh_.param<std::string>("input_topic", "/imu/data");
  std::string output_topic = private_nh_.param<std::string>("output_topic", "/mobile_ros/network_imu");
  imu_sub_ = nh_.subscribe(input_topic, 5, &ImuCellWrapper::imuCallback, this);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>(output_topic, 5);
  logNetworkProfile("imu wrapper active");
}

void ImuCellWrapper::imuCallback(const sensor_msgs::ImuConstPtr &msg)
{
  sensor_msgs::Imu routed = *msg;
  routed.header.frame_id = cell_name_.empty() ? "network_imu" : cell_name_;
  imu_pub_.publish(routed);
}
}  // namespace mobile_ros_cell_wrappers

PLUGINLIB_EXPORT_CLASS(mobile_ros_cell_wrappers::ImuCellWrapper, nodelet::Nodelet)

