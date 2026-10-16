//
// Created by luotinkai on 2022/2/27.
//

#pragma once

#include "StateMachine_sm.h"
#include "rm_fsm/common/union_command_sender.h"
#include "rm_fsm/common/subscriber.h"
#include "rm_fsm/state/cruise.h"
#include "rm_fsm/state/manual.h"
#include "rm_fsm/state/combat.h"
#include "rm_fsm/state/conduct.h"
#include "rm_fsm/state/defense.h"

#include <rm_common/decision/calibration_queue.h>
#include <rm_common/decision/command_sender.h>
#include <rm_common/decision/controller_manager.h>
#include <rm_common/ros_utilities.h>

using namespace rm_common;

class StateMachine
{
public:
  explicit StateMachine(ros::NodeHandle& nh);

  static void info(const std::string& string)
  {
    ROS_INFO_STREAM_ONCE(string);
  }

  template <typename T>
  void sendChassisCmd(T& state)
  {
    state->sendChassisCmd();
  }
  template <typename T>
  void sendGimbalCmd(T& state) const
  {
    state->sendGimbalCmd();
  }
  template <typename T>
  void sendShooterCmd(T& state)
  {
    state->sendShooterCmd();
  }

  template <typename T>
  void setEnterTime(T& state, ros::Time time)
  {
    state->setTime(time);
  }

  template <typename T>
  void cancelGoal(T& state)
  {
    state->cancelGoal();
  }

  void checkMoving()
  {
    combat_state_->checkMoving();
  }

  void setGyroState()
  {
    subscriber_.client_map_update_ = false;
    if (enable_gyro_srv_->isGyro())
      enable_gyro_srv_->setGyro(0.0);
    else
      enable_gyro_srv_->setGyro(move_gyro_vel_);
    enable_gyro_srv_->enable();
  }

  void setLimitVel(double& limit_vel)
  {
    if (combat_state_->enable_moving_combat_)
      set_limit_vel_srv_->setLimitVel(limit_vel);
  }

  void calibrate() const
  {
    gimbal_calibration_->reset();
    shooter_calibration_->reset();
    subscriber_.union_cmd_sender_->double_barrel_cmd_sender_->init();
  }
  void setReached()
  {
    defense_state_->reached_ = false;
  }
  void setAutoChassisMode()
  {
    ros::Time time = ros::Time::now();
    subscriber_.chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
    subscriber_.chassis_cmd_sender_->getMsg()->command_source_frame = "base_link";
    subscriber_.chassis_cmd_sender_->sendChassisCommand(time, false);
  }
  void publishFsmData();
  void remoteControlTurnOff();
  void controllerUpdate();
  void switchTarget();
  bool escape();
  void initGyroState();
  bool checkOutpostActive();
  void update();
  void sendPathData();
  void pathPointTransform(rm_msgs::MapSentryData* map_sentry_data, const geometry_msgs::PoseStamped& goal, int num,
                          bool is_start_point);
  void pathTransform(const nav_msgs::Path& goal_path, rm_msgs::MapSentryData* map_sentry_data);
  rm_common::CalibrationQueue *gimbal_calibration_{}, *shooter_calibration_{};
  rm_common::ControllerManager controller_manager_;
  rm_common::SwitchDetectionCaller* switch_detection_srv_{};
  EnableGyroServiceCaller* enable_gyro_srv_;
  SetLimitVelServiceCaller* set_limit_vel_srv_;
  rm_msgs::ManualToReferee fsm_to_referee_pub_data_;

  StateMachineContext context_;
  Subscriber subscriber_;
  std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> mb_client_;
  ros::Publisher fsm_to_referee_pub_;
  double outpost_risk_hp_;
  double outpost_urgent_hp_;
  bool init_gyro_state_{ false };
  Cruise* cruise_state_;
  Manual* manual_state_;
  Combat* combat_state_;
  Conduct* conduct_state_;
  Defense* defense_state_;
  double move_gyro_vel_{ 1.0 }, normal_limit_vel_{ 2.0 }, safe_limit_vel_{ 0.5 };

protected:
private:
  double last_point_x_{}, last_point_y_{};
};
