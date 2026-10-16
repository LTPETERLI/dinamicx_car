//
// Created by luotinkai on 2022/7/10.
//

#pragma once

#include <rm_common/decision/command_sender.h>
#include <rm_msgs/GimbalDesError.h>
#include <rm_msgs/TrackData.h>
#include <ros/ros.h>

using namespace rm_common;

class UnionCommandSender
{
public:
  UnionCommandSender(ros::NodeHandle& nh)
  {
    ros::NodeHandle gimbal_nh(nh, "gimbal");
    gimbal_cmd_sender_ = new GimbalCommandSender(gimbal_nh);
    ros::NodeHandle switcher_nh(nh, "switcher");
    double_barrel_cmd_sender_ = new DoubleBarrelCommandSender(switcher_nh);
    ros::NodeHandle auto_nh(nh, "auto");
    XmlRpc::XmlRpcValue pitch_value;
    try
    {
      auto_nh.getParam("pitch", pitch_value);
      pitch_min_ = static_cast<double>(pitch_value[0]);
      pitch_max_ = static_cast<double>(pitch_value[1]);
      auto_nh.getParam("circle", circle_);
    }
    catch (XmlRpc::XmlRpcException& e)
    {
      ROS_ERROR("%s", e.getMessage().c_str());
    }
  };
  GimbalCommandSender* gimbal_cmd_sender_;
  DoubleBarrelCommandSender* double_barrel_cmd_sender_{};
  double pitch_min_{}, pitch_max_{};
  int circle_{1};
  double yaw_direct_{ 1. }, pitch_direct_{ 1. };
};
