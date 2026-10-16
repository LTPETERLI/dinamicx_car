//
// Created by luotinkai on 2/3/23.
//

#pragma once

#include "rm_fsm/common/union_command_sender.h"
#include "rm_fsm/common/subscriber.h"
#include <rm_common/decision/command_sender.h>

class Manual
{
public:
  explicit Manual(ros::NodeHandle& nh, Subscriber& subscriber) : subscriber_(subscriber)
  {
    ros::NodeHandle vel_nh(nh, "vel");
    vel_2d_cmd_sender_ = new rm_common::Vel2DCommandSender(vel_nh);
    if (!vel_nh.getParam("gyro_move_reduction", gyro_move_reduction_))
      ROS_ERROR("Gyro move reduction no defined (namespace: %s)", nh.getNamespace().c_str());
  }

  void sendChassisCmd()
  {
    ros::Time time = ros::Time::now();
    bool is_gyro;
    if (std::abs(subscriber_.dbus_.wheel) > 0.01)
    {
      subscriber_.chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
      is_gyro = true;
    }
    else
    {
      subscriber_.chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::FOLLOW);
      is_gyro = false;
    }
    vel_2d_cmd_sender_->setAngularZVel(
        (std::abs(subscriber_.dbus_.ch_r_y) > 0.01 || std::abs(subscriber_.dbus_.ch_r_x) > 0.01) ?
            subscriber_.dbus_.wheel * gyro_rotate_reduction_ :
            subscriber_.dbus_.wheel);
    vel_2d_cmd_sender_->setLinearXVel(is_gyro ? subscriber_.dbus_.ch_r_y * gyro_move_reduction_ :
                                                subscriber_.dbus_.ch_r_y);
    vel_2d_cmd_sender_->setLinearYVel(is_gyro ? -subscriber_.dbus_.ch_r_x * gyro_move_reduction_ :
                                                -subscriber_.dbus_.ch_r_x);
    subscriber_.chassis_cmd_sender_->power_limit_->updateState(rm_common::PowerLimit::NORMAL);
    subscriber_.chassis_cmd_sender_->sendChassisCommand(time, is_gyro);
    vel_2d_cmd_sender_->sendCommand(time);
  }

  void sendGimbalCmd()
  {
    ros::Time time = ros::Time::now();
    if (subscriber_.track_data_.id == 0)
    {
      subscriber_.union_cmd_sender_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
      subscriber_.union_cmd_sender_->gimbal_cmd_sender_->setRate(-subscriber_.dbus_.ch_l_x, -subscriber_.dbus_.ch_l_y);
    }
    else
    {
      subscriber_.union_cmd_sender_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::TRACK);
      subscriber_.union_cmd_sender_->gimbal_cmd_sender_->setBulletSpeed(
          subscriber_.union_cmd_sender_->double_barrel_cmd_sender_->getSpeed());
    }
    subscriber_.union_cmd_sender_->gimbal_cmd_sender_->sendCommand(time);
  }

  void sendShooterCmd()
  {
    ros::Time now_time = ros::Time::now();
    if (subscriber_.dbus_.s_l == rm_msgs::DbusData::UP)
    {
      if (now_time.toSec() - last_time_.toSec() > 1.0 || continue_shoot_)
      {
        if (one_shoot_)
          continue_shoot_ = true;
        last_time_ = now_time;
        one_shoot_ = true;
        subscriber_.union_cmd_sender_->double_barrel_cmd_sender_->setMode(rm_msgs::ShootCmd::PUSH);
      }
      else
        subscriber_.union_cmd_sender_->double_barrel_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
    }
    else
    {
      continue_shoot_ = false;
      one_shoot_ = false;
      if (subscriber_.dbus_.s_l == rm_msgs::DbusData::MID)
        subscriber_.union_cmd_sender_->double_barrel_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
      else
        subscriber_.union_cmd_sender_->double_barrel_cmd_sender_->setMode(rm_msgs::ShootCmd::STOP);
    }

    subscriber_.union_cmd_sender_->double_barrel_cmd_sender_->checkError(ros::Time::now());
    subscriber_.union_cmd_sender_->double_barrel_cmd_sender_->sendCommand(now_time);
  }
  void setZero()
  {
    vel_2d_cmd_sender_->setZero();
    subscriber_.union_cmd_sender_->gimbal_cmd_sender_->setZero();
  }

private:
  rm_common::Vel2DCommandSender* vel_2d_cmd_sender_;
  ros::Time last_time_{};
  bool one_shoot_{ false };
  bool continue_shoot_{ false };
  Subscriber& subscriber_;
  double gyro_move_reduction_{ 1. };
  double gyro_rotate_reduction_{ 1. };
};
