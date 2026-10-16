//
// Created by luotinkai on 2/3/23.
//

#pragma once
#include "rm_fsm/state/common/auto.h"

class Cruise : public Auto
{
public:
  Cruise(ros::NodeHandle& nh, Subscriber& subscriber,
         std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>& mb_client)
    : Auto(nh, subscriber, mb_client)
  {
    ros::NodeHandle auto_nh(nh, "auto");
    auto_nh.getParam("gyro_duration", gyro_duration_);
    auto_nh.param("if_gyro", if_gyro_, false);
    auto_nh.getParam("durations", durations_);
    auto_nh.param("debug_mode", debug_mode_, false);
  }

  void sendChassisCmd() override
  {
    ros::Time time = ros::Time::now();
    Auto::sendChassisCmd();
    if (subscriber_.referee_is_online_)
      hitted_ = time.toSec() - subscriber_.robot_hurt_msgs_.stamp.toSec() < gyro_duration_ &&
                subscriber_.robot_hurt_msgs_.hurt_type == 0;
    else
      hitted_ = false;

    if (if_gyro_)
    {
      setGryo();
    }
    else if (gameStart())
    {
      if (use_referee_ && !subscriber_.referee_is_online_)
      {
        setGryo();
      }

      else if (hitted_)
      {
        if ((subscriber_.game_robot_status_.robot_id > 100 && subscriber_.game_robot_hp_.blue_outpost_hp > 0.0) ||
            (subscriber_.game_robot_status_.robot_id < 100 && subscriber_.game_robot_hp_.red_outpost_hp > 0.0))
        {
          patrol();
        }
        else
        {
          cancelGoal();
          setGryo();
        }
      }
      else
      {
        patrol();
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

  void patrol()
  {
    ros::Time time = ros::Time::now();
    std::vector<geometry_msgs::PoseStamped> cruise;
    if (first_point_)
    {
      mb_goal_.target_pose = patrols_["first_point"][0];
      if (mb_client_->getState().state_ != actionlib::SimpleClientGoalState::ACTIVE && first_plan_)
      {
        mb_client_->sendGoal(mb_goal_);
        first_plan_time_ = ros::Time::now();
        first_plan_ = false;
      }
      if (mb_client_->getState().isDone())
      {
        subscriber_.chassis_cmd_sender_->sendChassisCommand(time, false);
        vel_2d_cmd_sender_->set2DVel(0.0, 0.0, 0.0);
        vel_2d_cmd_sender_->sendCommand(time);
      }
      if (timeout(first_plan_time_, static_cast<double>(durations_[2])))
        first_point_ = false;
    }
    else if (!reach_goal_)
    {
      if (mb_client_->getState().state_ != actionlib::SimpleClientGoalState::ACTIVE ||
          timeout(cruise_begin_, static_cast<double>(durations_[0])))
      {
        if (subscriber_.client_map_send_data_.command_keyboard == rm_msgs::ClientMapSendData::KEY_K)
          cruise = patrols_["defense_points"];
        else
          cruise = patrols_["combat_points"];
        unsigned int rand_index;
        rand_index = rand() % cruise.size();
        if (rand_index == last_rand_index_)
          rand_index = (last_rand_index_ == cruise.size() - 1 ? 0 : last_rand_index_ + 1);
        mb_goal_.target_pose = cruise[rand_index];
        mb_client_->sendGoal(mb_goal_);
        cruise_begin_ = ros::Time::now();
        last_rand_index_ = rand_index;
      }

      if (ReachGoal(mb_goal_))
      {
        reach_goal_ = true;
        reach_time_ = ros::Time::now();
      }
    }
    else if (reach_goal_)
    {
      if (timeout(reach_time_, durations_[4]))
        reach_goal_ = false;
    }
    subscriber_.chassis_cmd_sender_->sendChassisCommand(time, false);
  }

  bool gameStart()
  {
    bool start;
    if (debug_mode_ || !use_referee_)
      start = true;
    else
    {
      if (subscriber_.referee_is_online_)
        start = subscriber_.game_status_.game_progress == 4;
      else
        start = timeout(enter_time_, static_cast<double>(durations_[1]));
    }
    return start;
  }

private:
  XmlRpc::XmlRpcValue durations_;
  bool debug_mode_;
  move_base_msgs::MoveBaseGoal mb_goal_;
  double gyro_duration_{ 3.0 };
  bool if_gyro_{};
  bool first_point_{ true };
  bool reach_goal_{ false };
  bool first_plan_{ true };
  ros::Time reach_time_{};
  ros::Time cruise_begin_{};
  ros::Time first_plan_time_{};
  unsigned int last_rand_index_{};
  bool hitted_{ false };
};
