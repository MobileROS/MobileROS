#include "mobile_ros_hub_manager/hub_container.hpp"

#include <nodelet/loader.h>
#include <ros/console.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace mobile_ros_hub_manager
{
HubContainer::HubContainer() : private_nh_("~"), loader_(private_nh_, private_nh_)
{
  status_pub_ = nh_.advertise<std_msgs::String>("/mobile_ros/hub_container_status", 5);
  heartbeat_timer_ = nh_.createTimer(ros::Duration(1.0), &HubContainer::publishHeartbeat, this);

  XmlRpc::XmlRpcValue wrapper_list;
  std::vector<WrapperConfig> configs;
  if (private_nh_.getParam("wrappers", wrapper_list) && wrapper_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (int i = 0; i < wrapper_list.size(); ++i)
    {
      if (wrapper_list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_WARN("Wrapper entry %d is not a struct, skipping", i);
        continue;
      }
      WrapperConfig config;
      if (wrapper_list[i].hasMember("name"))
      {
        config.name = static_cast<std::string>(wrapper_list[i]["name"]);
      }
      if (wrapper_list[i].hasMember("type"))
      {
        config.type = static_cast<std::string>(wrapper_list[i]["type"]);
      }
      if (wrapper_list[i].hasMember("remap"))
      {
        auto remap = wrapper_list[i]["remap"];
        if (remap.getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
          for (auto it = remap.begin(); it != remap.end(); ++it)
          {
            config.remappings[it->first] = static_cast<std::string>(it->second);
          }
        }
      }
      if (wrapper_list[i].hasMember("params"))
      {
        auto params = wrapper_list[i]["params"];
        if (params.getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
          for (auto it = params.begin(); it != params.end(); ++it)
          {
            config.params[it->first] = static_cast<std::string>(it->second);
          }
        }
      }
      configs.push_back(config);
    }
  }
  else
  {
    loadDefaults();
    return;
  }

  loadWrappers(configs);
}

void HubContainer::loadDefaults()
{
  std::vector<WrapperConfig> defaults = {
      {"camera_wrapper", "mobile_ros_cell_wrappers/CameraCellWrapper", {}, {{"cell_name", "camera"}}},
      {"imu_wrapper", "mobile_ros_cell_wrappers/ImuCellWrapper", {}, {{"cell_name", "imu"}}},
      {"cmd_vel_wrapper", "mobile_ros_cell_wrappers/CmdVelCellWrapper", {}, {{"cell_name", "cmd_vel"}}}};
  loadWrappers(defaults);
}

void HubContainer::loadWrappers(const std::vector<WrapperConfig> &configs)
{
  for (const auto &config : configs)
  {
    nodelet::M_string remappings;
    for (const auto &pair : config.remappings)
    {
      remappings[pair.first] = pair.second;
    }

    nodelet::V_string nargv;
    for (const auto &param : config.params)
    {
      nargv.push_back(param.first + "=" + param.second);
    }

    if (loader_.load(config.name, config.type, remappings, nargv))
    {
      ROS_INFO_STREAM("Hub container loaded wrapper " << config.name << " as " << config.type);
    }
    else
    {
      ROS_ERROR_STREAM("Failed to load wrapper " << config.name << " of type " << config.type);
    }
  }
}

void HubContainer::publishHeartbeat(const ros::TimerEvent &)
{
  std_msgs::String msg;
  msg.data = "hub_container_alive";
  status_pub_.publish(msg);
}

void HubContainer::spin()
{
  ros::spin();
}

}  // namespace mobile_ros_hub_manager

