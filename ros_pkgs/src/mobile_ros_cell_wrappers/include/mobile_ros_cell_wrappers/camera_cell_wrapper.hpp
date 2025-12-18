#pragma once

#include "mobile_ros_cell_wrappers/network_aware_cell.hpp"
#include <sensor_msgs/Image.h>

namespace mobile_ros_cell_wrappers
{
class CameraCellWrapper : public NetworkAwareCell
{
public:
  void setup() override;

private:
  void imageCallback(const sensor_msgs::ImageConstPtr &msg);
  ros::Subscriber image_sub_;
  ros::Publisher image_pub_;
};
}  // namespace mobile_ros_cell_wrappers

