#include "mobile_ros_cell_wrappers/camera_cell_wrapper.hpp"
#include <pluginlib/class_list_macros.h>

namespace mobile_ros_cell_wrappers
{
void CameraCellWrapper::setup()
{
  std::string input_topic = private_nh_.param<std::string>("input_topic", "/camera/image_raw");
  std::string output_topic = private_nh_.param<std::string>("output_topic", "/mobile_ros/network_camera");
  image_sub_ = nh_.subscribe(input_topic, 10, &CameraCellWrapper::imageCallback, this);
  image_pub_ = nh_.advertise<sensor_msgs::Image>(output_topic, 10);
  logNetworkProfile("camera wrapper active");
}

void CameraCellWrapper::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  sensor_msgs::Image routed = *msg;
  routed.header.frame_id = cell_name_.empty() ? "network_camera" : cell_name_;
  image_pub_.publish(routed);
}
}  // namespace mobile_ros_cell_wrappers

PLUGINLIB_EXPORT_CLASS(mobile_ros_cell_wrappers::CameraCellWrapper, nodelet::Nodelet)

