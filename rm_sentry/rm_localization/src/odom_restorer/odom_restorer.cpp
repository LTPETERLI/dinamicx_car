//
// Created by luotinkai on 3/12/23.
//

#include "rm_localization/odom_restorer/odom_restorer.h"

OdomRestorer::OdomRestorer(ros::NodeHandle& nh)
{
  ros::NodeHandle odom_nh;
  tf_buffer_ = new tf2_ros::Buffer(ros::Duration(0.5));
  odom_sub_ = odom_nh.subscribe<nav_msgs::Odometry>("/Odometry", 100, &OdomRestorer::odomCallBack, this);
  init_odom2base_.setOrigin(tf2::Vector3(0, 0, 0));
  init_odom2base_.setRotation(tf2::Quaternion(0, 0, 0, 1));

  geometry_msgs::TransformStamped init_odom2base_msg;
  init_odom2base_msg.header.stamp = ros::Time::now();
  init_odom2base_msg.header.frame_id = "odom";
  init_odom2base_msg.child_frame_id = "base_link";
  init_odom2base_msg.transform = tf2::toMsg(init_odom2base_);
  tf_buffer_->setTransform(init_odom2base_msg, "restorer");
  tf_broadcaster_.sendTransform(init_odom2base_msg);
}

void OdomRestorer::odomCallBack(const nav_msgs::Odometry::ConstPtr& msg)
{
  if (initial_pose)
  {
    try
    {
      geometry_msgs::TransformStamped tf_msg = tf_buffer_->lookupTransform("odom", "base_link", ros::Time(0));
      tf2::fromMsg(tf_msg.transform, init_odom2base_);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    initial_pose = false;
  }

  tf2::Quaternion quaternion;
  odom2base_.setOrigin(tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                    init_odom2base_.getOrigin().getZ() + msg->pose.pose.position.z));

  quaternion.setRPY(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    init_odom2base_.getRotation().getZ() + msg->pose.pose.orientation.z);
  quaternion.setW(msg->pose.pose.orientation.w);
  odom2base_.setRotation(quaternion);
}

void OdomRestorer::run()
{
  geometry_msgs::TransformStamped odom2base_msg;
  odom2base_msg.header.stamp = ros::Time::now();
  odom2base_msg.header.stamp.nsec += 1;
  odom2base_msg.header.frame_id = "odom";
  odom2base_msg.child_frame_id = "base_link";
  odom2base_msg.transform = tf2::toMsg(odom2base_);
  tf_buffer_->setTransform(odom2base_msg, "restorer");
  tf_broadcaster_.sendTransform(odom2base_msg);
}
