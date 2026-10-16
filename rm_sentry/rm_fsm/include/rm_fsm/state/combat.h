//
// Created by aung on 2023/3/9.
//

#pragma once
#include "rm_fsm/common/subscriber.h"

class Combat : public Auto
{
public:
  Combat(ros::NodeHandle& nh, Subscriber& subscriber,
         std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>& mb_client)
    : Auto(nh, subscriber, mb_client)
  {
    try
    {
      ros::NodeHandle auto_nh(nh, "auto");
      auto_nh.getParam("tuning_interval", tuning_interval_);
      auto_nh.getParam("lost_duration", lost_duration_);
      auto_nh.getParam("enable_moving_combat", enable_moving_combat_);
    }
    catch (XmlRpc::XmlRpcException& e)
    {
      ROS_ERROR("%s", e.getMessage().c_str());
    }
  }

  void sendChassisCmd() override
  {
    Auto::sendChassisCmd();
    ros::Time time = ros::Time::now();
    if (subscriber_.referee_is_online_)
    {
      if ((subscriber_.game_robot_status_.robot_id > 100 && subscriber_.game_robot_hp_.blue_outpost_hp > 0.0) ||
          (subscriber_.game_robot_status_.robot_id < 100 && subscriber_.game_robot_hp_.red_outpost_hp > 0.0))
      {
        subscriber_.chassis_cmd_sender_->sendChassisCommand(time, false);
        vel_2d_cmd_sender_->set2DVel(0.0, 0.0, 0.0);
        vel_2d_cmd_sender_->sendCommand(time);
      }
      else
      {
        subscriber_.chassis_cmd_sender_->sendChassisCommand(time, true);
        vel_2d_cmd_sender_->set2DVel(0.0, 0.0, getDirection());
        vel_2d_cmd_sender_->sendCommand(time);
      }
    }
    else
    {
      subscriber_.chassis_cmd_sender_->sendChassisCommand(time, true);
      vel_2d_cmd_sender_->set2DVel(0.0, 0.0, getDirection());
      vel_2d_cmd_sender_->sendCommand(time);
    }
  }

  void sendGimbalCmd() override
  {
    if (subscriber_.track_data_.id != 0)
      entry_track_ = subscriber_.track_data_.header.stamp;
    ros::Time time = ros::Time::now();
    subscriber_.union_cmd_sender_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
    subscriber_.union_cmd_sender_->gimbal_cmd_sender_->setBulletSpeed(
        subscriber_.union_cmd_sender_->double_barrel_cmd_sender_->getSpeed());
    subscriber_.union_cmd_sender_->gimbal_cmd_sender_->sendCommand(time);
  }

  void checkMoving()
  {
    if (!enable_moving_combat_)
      cancelGoal();
  }

  void sendShooterCmd() override
  {
    ros::Time time = ros::Time::now();
    if (subscriber_.track_data_.id != 0)
    {
      subscriber_.union_cmd_sender_->double_barrel_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
      subscriber_.union_cmd_sender_->double_barrel_cmd_sender_->checkError(ros::Time::now());
      subscriber_.union_cmd_sender_->double_barrel_cmd_sender_->sendCommand(time);
    }
    else
      Auto::sendShooterCmd();
  }

  double getDirection()
  {
    if ((ros::Time::now() - enter_time_).toSec() > tuning_interval_)
    {
      gyro_direct_ *= -1;
      enter_time_ = ros::Time::now();
    }
    combat_gryo_vel_ = gyro_direct_ * (static_cast<double>(standby_velocity_[0]) * std::sin(ros::Time::now().toSec()) +
                                       static_cast<double>(standby_velocity_[1]));
    return combat_gryo_vel_;
  }

  double lost_duration_{ 1.5 };
  ros::Time entry_track_;
  bool enable_moving_combat_{ false };

private:
  double combat_gryo_vel_{};
  double gyro_direct_{ -1 };
  double tuning_interval_{};
};
