#pragma once

#include "mobile_ros_cell_wrappers/network_aware_cell.hpp"
#include <geometry_msgs/Twist.h>

namespace mobile_ros_cell_wrappers
{
class CmdVelCellWrapper : public NetworkAwareCell
{
public:
  void setup() override;

private:
  void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg);
  ros::Subscriber cmd_sub_;
  ros::Publisher cmd_pub_;
};
}  // namespace mobile_ros_cell_wrappers

