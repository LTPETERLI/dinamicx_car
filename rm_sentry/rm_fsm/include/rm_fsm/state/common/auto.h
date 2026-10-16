//
// Created by luotinkai on 2/3/23.
//

#pragma once

#include "rm_fsm/common/union_command_sender.h"
#include "rm_fsm/common/subscriber.h"
#include "rm_fsm/common/tools.h"

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <random>
#include <unordered_map>
#include <rm_common/decision/command_sender.h>
#include <rm_common/ori_tool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class Auto
{
public:
  typedef std::vector<geometry_msgs::PoseStamped>::const_iterator PoseStampedIterator;

  Auto(ros::NodeHandle& nh, Subscriber& subscriber,
       std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>& mb_client)
    : subscriber_(subscriber), tf_listener_(tf_buffer_), mb_client_(mb_client)
  {
    ros::NodeHandle vel_nh(nh, "vel");
    vel_2d_cmd_sender_ = new rm_common::Vel2DCommandSender(vel_nh);

    ros::NodeHandle auto_nh(nh, "auto");
    auto_nh.getParam("standby_velocity", standby_velocity_);
    auto_nh.param("use_referee", use_referee_, true);
    XmlRpc::XmlRpcValue patrols;
    auto_nh.getParam("patrols", patrols);
    getPatrolList(patrols);
    defense_list_ = patrols_["escape_points"];

    mb_client_ = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
    vel_sub_ = nh.subscribe<geometry_msgs::Twist>("/remap_vel", 10, &Auto::sendVelCommand, this);
  }

  void sendVelCommand(const geometry_msgs::Twist::ConstPtr& vel)
  {
    ros::Time time = ros::Time::now();
    vel_2d_cmd_sender_->set2DVel(vel->linear.x, vel->linear.y, vel->angular.z);
    vel_2d_cmd_sender_->sendCommand(time);
  }

  virtual void sendChassisCmd()
  {
    subscriber_.chassis_cmd_sender_->setMode(rm_msgs::ChassisCmd::RAW);
    subscriber_.chassis_cmd_sender_->getMsg()->command_source_frame = "base_link";
  }

  virtual void sendGimbalCmd()
  {
    ros::Time time = ros::Time::now();
    try
    {
      map2yaw_ = tf_buffer_.lookupTransform("map", "yaw", ros::Time(0));
      yaw2pitch_ = tf_buffer_.lookupTransform("yaw", "pitch", ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }
    double yaw = yawFromQuat(map2yaw_.transform.rotation);
    double roll_temp, pitch, yaw_temp;
    quatToRPY(yaw2pitch_.transform.rotation, roll_temp, pitch, yaw_temp);

    if (circle_count_ == 0)
    {
      subscriber_.union_cmd_sender_->yaw_direct_ = 1;
    }

    if (circle_count_ > subscriber_.union_cmd_sender_->circle_)
    {
      subscriber_.union_cmd_sender_->yaw_direct_ = -1;
    }

    if (yaw - last_yaw_ > M_PI)
    {
      circle_count_--;
    }
    else if (yaw - last_yaw_ < -M_PI)
    {
      circle_count_++;
    }
    last_yaw_ = yaw;

    if (pitch >= subscriber_.union_cmd_sender_->pitch_max_)
      subscriber_.union_cmd_sender_->pitch_direct_ = -1.;
    else if (pitch <= subscriber_.union_cmd_sender_->pitch_min_)
      subscriber_.union_cmd_sender_->pitch_direct_ = 1.;
    subscriber_.union_cmd_sender_->gimbal_cmd_sender_->setMode(rm_msgs::GimbalCmd::RATE);
    subscriber_.union_cmd_sender_->gimbal_cmd_sender_->setRate(subscriber_.union_cmd_sender_->yaw_direct_,
                                                               subscriber_.union_cmd_sender_->pitch_direct_);
    subscriber_.union_cmd_sender_->gimbal_cmd_sender_->sendCommand(time);
  }

  virtual void sendShooterCmd()
  {
    ros::Time time = ros::Time::now();
    subscriber_.union_cmd_sender_->double_barrel_cmd_sender_->setMode(rm_msgs::ShootCmd::READY);
    subscriber_.union_cmd_sender_->double_barrel_cmd_sender_->sendCommand(time);
  }

  void setGryo()
  {
    ros::Time time = ros::Time::now();
    vel_2d_cmd_sender_->setAngularZVel(static_cast<double>(standby_velocity_[0]) * std::sin(ros::Time::now().toSec()) +
                                       static_cast<double>(standby_velocity_[1]));
    subscriber_.chassis_cmd_sender_->sendChassisCommand(time, true);
    vel_2d_cmd_sender_->sendCommand(time);
  }

  void setZero()
  {
    vel_2d_cmd_sender_->setZero();
    subscriber_.union_cmd_sender_->gimbal_cmd_sender_->setZero();
    subscriber_.union_cmd_sender_->double_barrel_cmd_sender_->setZero();
  }

  void setTime(ros::Time start)
  {
    enter_time_ = start;
  }

  void cancelGoal()
  {
    if (mb_client_->getState().state_ == actionlib::SimpleClientGoalState::ACTIVE ||
        mb_client_->getState().state_ == actionlib::SimpleClientGoalState::PENDING)
      mb_client_->cancelGoal();
  }

  bool timeout(ros::Time begin, double duration)
  {
    return (ros::Time::now() - begin).toSec() > duration;
  }

  bool changeGoal(ros::Time begin, double duration, PoseStampedIterator& iterator)
  {
    bool change_goal = timeout(begin, duration);
    if (change_goal)
    {
      if (iterator != defense_list_.end())
        ++iterator;
      else
        iterator = defense_list_.begin();
    }
    return change_goal;
  }

  void getPatrolList(const XmlRpc::XmlRpcValue& patrols)
  {
    for (const auto& patrol : patrols)
    {
      ROS_ASSERT(patrol.second.hasMember("position"));
      ROS_ASSERT(patrol.second["position"].getType() == XmlRpc::XmlRpcValue::TypeArray);

      std::vector<geometry_msgs::PoseStamped> point;
      for (int i = 0; i < patrol.second["position"].size(); ++i)
      {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = static_cast<double>(patrol.second["position"][i][0]);
        pose_stamped.pose.position.y = static_cast<double>(patrol.second["position"][i][1]);
        tf2::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, static_cast<double>(patrol.second["position"][i][2]));
        pose_stamped.pose.orientation = tf2::toMsg(quaternion);
        point.push_back(pose_stamped);
      }
      patrols_.insert(std::make_pair(patrol.first, point));
    }
  }

  bool ReachGoal(move_base_msgs::MoveBaseGoal goal)
  {
    geometry_msgs::TransformStamped map2base;
    try
    {
      map2base = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }

    double tor = hypot((goal.target_pose.pose.position.x - map2base.transform.translation.x),
                       (goal.target_pose.pose.position.y - map2base.transform.translation.y));
    if (tor < 0.5)
    {
      return true;
    }
    else
      return false;
  }

  rm_common::Vel2DCommandSender* vel_2d_cmd_sender_;
  ros::Subscriber vel_sub_;
  Subscriber& subscriber_;
  XmlRpc::XmlRpcValue standby_velocity_;
  ros::Time enter_time_;
  std::vector<geometry_msgs::PoseStamped> defense_list_;
  std::unordered_map<std::string, std::vector<geometry_msgs::PoseStamped>> patrols_{};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>& mb_client_;
  geometry_msgs::TransformStamped map2yaw_{};
  geometry_msgs::TransformStamped yaw2pitch_{};
  double last_yaw_{};
  int circle_count_{};
  // false = 调试/空地图模式：不依赖裁判系统数据（默认 true，比赛行为不变）
  bool use_referee_{ true };
};
