#pragma once

#include "mobile_ros_cell_wrappers/network_aware_cell.hpp"
#include <sensor_msgs/Imu.h>

namespace mobile_ros_cell_wrappers
{
class ImuCellWrapper : public NetworkAwareCell
{
public:
  void setup() override;

private:
  void imuCallback(const sensor_msgs::ImuConstPtr &msg);
  ros::Subscriber imu_sub_;
  ros::Publisher imu_pub_;
};
}  // namespace mobile_ros_cell_wrappers

