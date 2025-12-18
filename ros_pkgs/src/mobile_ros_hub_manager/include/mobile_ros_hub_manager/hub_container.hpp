#pragma once

#include <nodelet/loader.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <map>
#include <string>
#include <vector>

namespace mobile_ros_hub_manager
{
struct WrapperConfig
{
  std::string name;
  std::string type;
  std::map<std::string, std::string> remappings;
  std::map<std::string, std::string> params;
};

class HubContainer
{
public:
  HubContainer();
  void spin();

private:
  void loadDefaults();
  void loadWrappers(const std::vector<WrapperConfig> &configs);
  void publishHeartbeat(const ros::TimerEvent &);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  nodelet::Loader loader_;
  ros::Timer heartbeat_timer_;
  ros::Publisher status_pub_;
};
}  // namespace mobile_ros_hub_manager

