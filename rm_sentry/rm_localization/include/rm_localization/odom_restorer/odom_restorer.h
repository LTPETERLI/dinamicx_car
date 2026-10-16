//
// Created by luotinkai on 3/12/23.
//

#pragma once

#include "rm_common/tf_rt_broadcaster.h"
#include <rm_common/ros_utilities.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <realtime_tools/realtime_buffer.h>

class OdomRestorer
{
public:
  OdomRestorer(ros::NodeHandle& nh);
  void odomCallBack(const nav_msgs::Odometry::ConstPtr& msg);
  void run();

private:
  bool initial_pose{ true };
  tf2::Transform init_odom2base_;
  tf2::Transform odom2base_;
  ros::Subscriber odom_sub_;
  tf2_ros::Buffer* tf_buffer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_{};
};
