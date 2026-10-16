//
// Created by aung on 2023/4/29.
//

#pragma once
#include "rm_fsm/state/common/auto.h"

class Conduct : public Auto
{
public:
  Conduct(ros::NodeHandle& nh, Subscriber& subscriber,
          std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>& mb_client)
    : Auto(nh, subscriber, mb_client)
  {
    try
    {
      ros::NodeHandle auto_nh(nh, "auto");
      auto_nh.getParam("minimap_origin", minimap_origin_);
      auto_nh.getParam("is_blue", is_blue_);
      auto_nh.getParam("vel_coeff", vel_coeff_);
      ros::NodeHandle vel_nh(nh, "vel");
      vel_2d_cmd_sender_ = new rm_common::Vel2DCommandSender(vel_nh);
    }
    catch (XmlRpc::XmlRpcException& e)
    {
      ROS_ERROR("%s", e.getMessage().c_str());
    }
    minimapToWorld();
  }

  void sendChassisCmd() override
  {
    ros::Time time = ros::Time::now();
    Auto::sendChassisCmd();
    move_base_msgs::MoveBaseGoal mb_goal;

    if (subscriber_.client_map_update_)
    {
      is_standby_ = subscriber_.client_map_send_data_.command_keyboard == rm_msgs::ClientMapSendData::KEY_O;
      subscriber_.client_map_update_ = false;
    }

    if (is_standby_)
    {
      cancelGoal();
      setGryo();
    }
    else
    {
      if (subscriber_.client_map_send_data_.command_keyboard == rm_msgs::ClientMapSendData::KEY_H)
      {
        cancelGoal();
        geometry_msgs::PoseStamped out_pose;
        targetPoseTransform(subscriber_.client_map_send_data_.target_position_x,
                            subscriber_.client_map_send_data_.target_position_y, &out_pose);
        geometry_msgs::TransformStamped map2base;
        try
        {
          map2base = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
        }
        catch (tf2::TransformException& ex)
        {
          ROS_WARN("%s", ex.what());
          return;
        }
        vel_2d_cmd_sender_->setLinearXVel(vel_coeff_ * (out_pose.pose.position.x - map2base.transform.translation.x));
        vel_2d_cmd_sender_->setLinearYVel(vel_coeff_ * (out_pose.pose.position.y - map2base.transform.translation.y));
        vel_2d_cmd_sender_->sendCommand(ros::Time::now());
      }
      else if (mb_client_->getState() != actionlib::SimpleClientGoalState::ACTIVE &&
               subscriber_.client_map_send_data_.command_keyboard != rm_msgs::ClientMapSendData::KEY_H)
      {
        geometry_msgs::PoseStamped target_pose;
        targetPoseTransform(subscriber_.client_map_send_data_.target_position_x,
                            subscriber_.client_map_send_data_.target_position_y, &target_pose);
        mb_goal.target_pose = target_pose;
        mb_client_->sendGoal(mb_goal);
        subscriber_.chassis_cmd_sender_->sendChassisCommand(time, false);
      }
    }
  }

  void sendGimbalCmd() override
  {
    Auto::sendGimbalCmd();
  }

  void sendShooterCmd() override
  {
    Auto::sendShooterCmd();
  }

  void minimapToWorld()
  {
    minimap2world_.setOrigin(tf2::Vector3(static_cast<double>(minimap_origin_[is_blue_][0]),
                                          static_cast<double>(minimap_origin_[is_blue_][1]), 0));
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, static_cast<double>(minimap_origin_[is_blue_][2]));
    minimap2world_.setRotation(quaternion);
  }

  void targetPoseTransform(int32_t sub_x, int32_t sub_y, geometry_msgs::PoseStamped* target_pose)
  {
    tf2::Transform minimap2target, world2target;
    minimap2target.setOrigin(tf2::Vector3(sub_x, sub_y, 0));
    minimap2target.setRotation(tf2::Quaternion(0, 0, 0, 1));
    world2target = minimap2world_.inverse() * minimap2target;
    target_pose->header.frame_id = "map";
    target_pose->pose.position.x = world2target.getOrigin().x();
    target_pose->pose.position.y = world2target.getOrigin().y();
    target_pose->pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));
  }
  tf2::Transform minimap2world_;

private:
  bool is_standby_{ true };
  double vel_coeff_{ 0.5 };
  rm_common::Vel2DCommandSender* vel_2d_cmd_sender_;
  XmlRpc::XmlRpcValue minimap_origin_;
  bool is_blue_;
};
