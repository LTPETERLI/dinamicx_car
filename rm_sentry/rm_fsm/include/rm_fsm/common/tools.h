//
// Created by spy on 2023/7/26.
//
#pragma once

#include "rm_common/decision/service_caller.h"
#include <rm_msgs/EnableGyro.h>
#include <rm_msgs/SetLimitVel.h>
#include <math.h>

class EnableGyroServiceCaller : public ServiceCallerBase<rm_msgs::EnableGyro>
{
public:
  explicit EnableGyroServiceCaller(ros::NodeHandle& nh) : ServiceCallerBase<rm_msgs::EnableGyro>(nh, "/enable_gyro")
  {
    service_.request.gyro_speed = 0.0;
    callService();
  }
  void setGyro(double gyro_speed)
  {
    service_.request.gyro_speed = gyro_speed;
  }
  bool isGyro()
  {
    return service_.response.is_gyro;
  }
  void enable()
  {
    if (fabs(service_.request.gyro_speed) >= 0.0)
      ROS_INFO("Enable gyro true");
    else
      ROS_INFO("stop gyro");
    callService();
  }
};

class SetLimitVelServiceCaller : public ServiceCallerBase<rm_msgs::SetLimitVel>
{
public:
  explicit SetLimitVelServiceCaller(ros::NodeHandle& nh,double init_limit_vel) : ServiceCallerBase<rm_msgs::SetLimitVel>(nh, "/set_limit_vel")
  {
    service_.request.limit_vel=init_limit_vel;
    callService();
  }
  void setLimitVel(double& limit_vel)
  {
    service_.request.limit_vel=limit_vel;
    ROS_INFO("set planner's limit vel: %f",service_.request.limit_vel);
    callService();
  }
  double getLimtVel()
  {
    return service_.response.current_limit_vel;
  }
};