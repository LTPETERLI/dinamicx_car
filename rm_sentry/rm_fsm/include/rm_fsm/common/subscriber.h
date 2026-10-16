//
// Created by qiayuan on 2022/6/15.
//
#pragma once

#include "rm_fsm/state_machine.h"

#include <rm_msgs/DbusData.h>
#include <rm_msgs/GameRobotStatus.h>
#include <rm_msgs/GameStatus.h>
#include <rm_msgs/GimbalDesError.h>
#include <rm_msgs/RobotHurt.h>
#include <rm_msgs/GameRobotHp.h>
#include <rm_msgs/TrackData.h>
#include <rm_msgs/ManualToReferee.h>
#include <rm_msgs/MapSentryData.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <rm_msgs/ClientMapSendData.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

class Subscriber
{
public:
  explicit Subscriber(StateMachineContext& fsm_context, ros::NodeHandle& nh) : context_(fsm_context)
  {
    ros::NodeHandle subscriber_nh;
    map_sentry_data_pub_ = nh.advertise<rm_msgs::MapSentryData>("/map_sentry_data", 10);
    dbus_sub_ = subscriber_nh.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &Subscriber::dbusCallback, this);
    track_sub_ = subscriber_nh.subscribe<rm_msgs::TrackData>("/track", 10, &Subscriber::trackCallback, this);
    gimbal_des_error_sub_ = subscriber_nh.subscribe<rm_msgs::GimbalDesError>("/controllers/gimbal_controller/error", 10,
                                                                             &Subscriber::gimbalDesErrorCallback, this);
    game_robot_status_sub_ = subscriber_nh.subscribe<rm_msgs::GameRobotStatus>(
        "/rm_referee/game_robot_status", 10, &Subscriber::gameRobotStatusCallback, this);
    power_heat_data_sub_ = subscriber_nh.subscribe<rm_msgs::PowerHeatData>("/rm_referee/power_heat_data", 10,
                                                                           &Subscriber::powerHeatDataCallback, this);
    game_status_sub_ = subscriber_nh.subscribe<rm_msgs::GameStatus>("/rm_referee/game_status", 10,
                                                                    &Subscriber::gameStatusCallback, this);
    robot_hurt_sub_ = subscriber_nh.subscribe<rm_msgs::RobotHurt>("/rm_referee/robot_hurt_data", 10,
                                                                  &Subscriber::robotHurtCallback, this);
    robot_hp_sub_ = subscriber_nh.subscribe<rm_msgs::GameRobotHp>("/rm_referee/game_robot_hp", 10,
                                                                  &Subscriber::robotHpCallback, this);
    client_map_send_data_sub_ = subscriber_nh.subscribe<rm_msgs::ClientMapSendData>(
        "/rm_referee/client_map_send_data", 10, &Subscriber::clientMapSendDataCallback, this);
    global_planner_sub_ = subscriber_nh.subscribe<nav_msgs::Path>("/move_base/GlobalPlanner/plan", 10,
                                                                  &Subscriber::gloabPlannerCallback, this);
    // Used to update referee data.
    ros::NodeHandle chassis_nh(nh, "chassis");
    chassis_cmd_sender_ = new rm_common::ChassisCommandSender(chassis_nh);
    ros::NodeHandle union_nh(nh, "union");
    union_cmd_sender_ = new UnionCommandSender(union_nh);
  }

  rm_msgs::DbusData dbus_;
  rm_msgs::RobotHurt robot_hurt_msgs_{};
  rm_msgs::TrackData track_data_;
  rm_msgs::GameRobotStatus game_robot_status_;
  rm_msgs::GameRobotHp game_robot_hp_{};
  rm_msgs::GameStatus game_status_;
  rm_msgs::ClientMapSendData client_map_send_data_;
  nav_msgs::Path goal_planner_;
  rm_common::ChassisCommandSender* chassis_cmd_sender_{};
  UnionCommandSender* union_cmd_sender_{};
  ros::Publisher map_sentry_data_pub_;
  ros::Time last_map_data_update_{};
  bool referee_is_online_{ false };
  bool client_map_update_{ false };
  bool goal_planner_update_{ false };

private:
  void dbusCallback(const DbusData::ConstPtr& data)
  {
    dbus_ = *data;
    context_.dbusUpdate(*data);
    chassis_cmd_sender_->updateRefereeStatus(referee_is_online_);
    union_cmd_sender_->double_barrel_cmd_sender_->updateRefereeStatus(referee_is_online_);
  }
  void gimbalDesErrorCallback(const GimbalDesError::ConstPtr& data)
  {
    union_cmd_sender_->double_barrel_cmd_sender_->updateGimbalDesError(*data);
  }
  void trackCallback(const TrackData::ConstPtr& data)
  {
    track_data_ = *data;
    context_.trackUpdate(*data);
    union_cmd_sender_->double_barrel_cmd_sender_->updateTrackData(*data);
  }
  void gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data)
  {
    chassis_cmd_sender_->updateGameStatus(*data);
    game_status_ = *data;
  }
  void robotHurtCallback(const rm_msgs::RobotHurt::ConstPtr& data)
  {
    robot_hurt_msgs_ = *data;
  }
  void robotHpCallback(const rm_msgs::GameRobotHp::ConstPtr& data)
  {
    game_robot_hp_ = *data;
    context_.robotHpUpdate(*data);
  }
  void gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data)
  {
    context_.robotStatusUpdate(*data);
    game_robot_status_ = *data;
    chassis_cmd_sender_->updateGameRobotStatus(*data);
    union_cmd_sender_->double_barrel_cmd_sender_->updateGameRobotStatus(*data);
  }
  void powerHeatDataCallback(const rm_msgs::PowerHeatData::ConstPtr& data)
  {
    referee_is_online_ = (ros::Time::now() - data->stamp < ros::Duration(0.3));
    chassis_cmd_sender_->updatePowerHeatData(*data);
    union_cmd_sender_->double_barrel_cmd_sender_->updatePowerHeatData(*data);
  }
  void clientMapSendDataCallback(const rm_msgs::ClientMapSendData::ConstPtr& data)
  {
    client_map_send_data_ = *data;
    if((ros::Time::now() - last_map_data_update_) > ros::Duration(0.1))
      client_map_update_ = true;
    last_map_data_update_ = ros::Time::now();
    context_.clientMapSendDataUpdate(*data);
  }
  void gloabPlannerCallback(const nav_msgs::Path::ConstPtr& data)
  {
    goal_planner_ = *data;
    goal_planner_update_ = true;
  }

  StateMachineContext& context_;
  ros::Subscriber dbus_sub_;
  ros::Subscriber track_sub_;
  ros::Subscriber robot_hp_sub_;
  ros::Subscriber game_robot_status_sub_, game_status_sub_, robot_hurt_sub_;
  ros::Subscriber gimbal_des_error_sub_;
  ros::Subscriber power_heat_data_sub_, capacity_sub_;
  ros::Subscriber client_map_send_data_sub_;
  ros::Subscriber global_planner_sub_;
};
