//
// Created by luotinkai on 2/3/23.
//

#pragma once
#include "rm_fsm/state/common/auto.h"

class Defense : public Auto
{
public:
  Defense(ros::NodeHandle& nh, Subscriber& subscriber,
          std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>& mb_client)
    : Auto(nh, subscriber, mb_client)
  {
    ros::NodeHandle auto_nh(nh, "auto");
    auto_nh.getParam("gyro_duration", gyro_duration_);
    auto_nh.param("if_gyro", if_gyro_, false);
    auto_nh.getParam("durations", durations_);
    iterator_ = std::begin(defense_list_);
  }

  void sendChassisCmd() override
  {
    ros::Time time = ros::Time::now();
    Auto::sendChassisCmd();
    if (subscriber_.referee_is_online_)
    {
      if ((subscriber_.game_robot_status_.robot_id > 100 && subscriber_.game_robot_hp_.blue_outpost_hp > 0.0) ||
          (subscriber_.game_robot_status_.robot_id < 100 && subscriber_.game_robot_hp_.red_outpost_hp > 0.0))
      {
        hitted_ = false;
      }
      else
      {
        hitted_ = time.toSec() - subscriber_.robot_hurt_msgs_.stamp.toSec() < gyro_duration_ &&
                  subscriber_.robot_hurt_msgs_.hurt_type == 0;
      }
    }
    else
      hitted_ = false;

    if (if_gyro_ || (use_referee_ && !subscriber_.referee_is_online_))
    {
      setGryo();
    }
    else
    {
      if (hitted_)
      {
        cancelGoal();
        setGryo();
      }
      else
      {
        if (!reached_)
        {
          if(iterator_ == defense_list_.end())
            iterator_ = defense_list_.begin();

          if (mb_client_->getState().state_ != actionlib::SimpleClientGoalState::ACTIVE ||
              changeGoal(cruise_begin_, static_cast<double>(durations_[3]), iterator_))
          {
            mb_goal_.target_pose = *iterator_;
            mb_client_->sendGoal(mb_goal_);
            cruise_begin_ = ros::Time::now();
          }

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

          double tor = hypot((mb_goal_.target_pose.pose.position.x - map2base.transform.translation.x),
                             (mb_goal_.target_pose.pose.position.y - map2base.transform.translation.y));
          if (tor < 0.5)
          {
            reached_ = true;
          }

          if (timeout(cruise_begin_, static_cast<double>(durations_[3])))
          {
            if (iterator_ != defense_list_.end())
              ++iterator_;
          }
          subscriber_.chassis_cmd_sender_->sendChassisCommand(time, false);
        }

        if (reached_)
        {
          cancelGoal();
          if (!use_referee_ ||
              (subscriber_.referee_is_online_ &&
               ((subscriber_.game_robot_status_.robot_id > 100 && subscriber_.game_robot_hp_.blue_outpost_hp > 0.0) ||
                (subscriber_.game_robot_status_.robot_id < 100 && subscriber_.game_robot_hp_.red_outpost_hp > 0.0))))
          {
            // use_referee_ = false 时：到点后站住，不原地转小陀螺
            subscriber_.chassis_cmd_sender_->sendChassisCommand(time, false);
            vel_2d_cmd_sender_->set2DVel(0.0, 0.0, 0.0);
            vel_2d_cmd_sender_->sendCommand(time);
          }
          else
            setGryo();
        }

        if (subscriber_.client_map_update_ &&
            subscriber_.client_map_send_data_.command_keyboard == rm_msgs::ClientMapSendData::KEY_U)
        {
          subscriber_.client_map_update_ = false;
          reached_ = false;
          if (iterator_ != defense_list_.end())
            ++iterator_;
        }
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

  bool reached_{ false };

private:
  XmlRpc::XmlRpcValue durations_;
  double gyro_duration_{ 3.0 };
  bool if_gyro_{};
  ros::Time cruise_begin_{};
  PoseStampedIterator iterator_;
  move_base_msgs::MoveBaseGoal mb_goal_;
  bool hitted_{ false };
};
