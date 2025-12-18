#pragma once

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <string>

namespace mobile_ros_cell_wrappers
{
class NetworkAwareCell : public nodelet::Nodelet
{
public:
  NetworkAwareCell();
  ~NetworkAwareCell() override = default;

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  double network_latency_budget_{0.0};
  double reliability_hint_{0.0};
  std::string cell_name_;

  void onInit() override;
  virtual void setup() = 0;
  void logNetworkProfile(const std::string &profile);
};
}  // namespace mobile_ros_cell_wrappers

