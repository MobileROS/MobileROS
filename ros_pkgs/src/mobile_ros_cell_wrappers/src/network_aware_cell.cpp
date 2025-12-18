#include "mobile_ros_cell_wrappers/network_aware_cell.hpp"

namespace mobile_ros_cell_wrappers
{
NetworkAwareCell::NetworkAwareCell() : private_nh_("~") {}

void NetworkAwareCell::onInit()
{
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();
  private_nh_.param("network_latency_budget", network_latency_budget_, 50.0);
  private_nh_.param("reliability_hint", reliability_hint_, 0.95);
  private_nh_.param("cell_name", cell_name_, std::string(getName()));
  setup();
  logNetworkProfile("initialized");
}

void NetworkAwareCell::logNetworkProfile(const std::string &profile)
{
  ROS_INFO_STREAM("[" << cell_name_ << "] network-aware profile: " << profile
                  << " (latency_budget_ms=" << network_latency_budget_
                  << ", reliability_hint=" << reliability_hint_ << ")");
}
}  // namespace mobile_ros_cell_wrappers

