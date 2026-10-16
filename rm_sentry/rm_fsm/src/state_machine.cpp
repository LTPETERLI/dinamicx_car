//
// Created by luotinkai on 2022/3/2.
//
#include "rm_fsm/state_machine.h"

StateMachine::StateMachine(ros::NodeHandle& nh) : controller_manager_(nh), context_(*this), subscriber_(context_, nh)
{
  try
  {
    XmlRpc::XmlRpcValue gimbal_calibration, shooter_calibration;
    nh.getParam("gimbal_calibration", gimbal_calibration);
    gimbal_calibration_ = new rm_common::CalibrationQueue(gimbal_calibration, nh, controller_manager_);
    nh.getParam("shooter_calibration", shooter_calibration);
    shooter_calibration_ = new rm_common::CalibrationQueue(shooter_calibration, nh, controller_manager_);
  }
  catch (XmlRpc::XmlRpcException& e)
  {
    ROS_ERROR("%s", e.getMessage().c_str());
  }
  ros::NodeHandle detection_switch_nh(nh, "detection_switch");
  switch_detection_srv_ = new rm_common::SwitchDetectionCaller(detection_switch_nh);
  nh.getParam("outpost_risk_hp", outpost_risk_hp_);
  nh.getParam("outpost_urgent_hp", outpost_urgent_hp_);
  nh.getParam("init_gyro_state", init_gyro_state_);
  fsm_to_referee_pub_ = nh.advertise<rm_msgs::ManualToReferee>("/manual_to_referee", 1);
  ros::NodeHandle planner_nh(nh, "planner");
  planner_nh.getParam("move_gyro_vel", move_gyro_vel_);
  planner_nh.getParam("normal_limit_vel", normal_limit_vel_);
  planner_nh.getParam("safe_limit_vel", safe_limit_vel_);

  cruise_state_ = new Cruise(nh, subscriber_, mb_client_);
  manual_state_ = new Manual(nh, subscriber_);
  combat_state_ = new Combat(nh, subscriber_, mb_client_);
  conduct_state_ = new Conduct(nh, subscriber_, mb_client_);
  defense_state_ = new Defense(nh, subscriber_, mb_client_);

  enable_gyro_srv_ = new EnableGyroServiceCaller(nh);
  set_limit_vel_srv_ = new SetLimitVelServiceCaller(nh, normal_limit_vel_);

  controller_manager_.startStateControllers();
  context_.enterStartState();
}

void StateMachine::controllerUpdate()
{
  ros::Time time = ros::Time::now();
  gimbal_calibration_->update(time);
  shooter_calibration_->update(time);
  controller_manager_.update();
}

void StateMachine::remoteControlTurnOff()
{
  controller_manager_.stopMainControllers();
  controller_manager_.stopCalibrationControllers();
  if (strcmp(context_.getState().getName(), "StateMachineMap::Cruise"))
    cruise_state_->setZero();
  if (strcmp(context_.getState().getName(), "StateMachineMap::Manual"))
    manual_state_->setZero();
  if (strcmp(context_.getState().getName(), "StateMachineMap::Combat"))
    combat_state_->setZero();
  if (strcmp(context_.getState().getName(), "StateMachineMap::Conduct"))
    conduct_state_->setZero();
}

bool StateMachine::escape()
{
  bool escape;
  if (subscriber_.game_robot_status_.robot_id > 100)
    escape = (subscriber_.game_robot_hp_.blue_outpost_hp < outpost_urgent_hp_ &&
              subscriber_.game_robot_hp_.blue_outpost_hp > 100) &&
             !defense_state_->reached_;
  if (subscriber_.game_robot_status_.robot_id < 100)
    escape = (subscriber_.game_robot_hp_.red_outpost_hp < outpost_urgent_hp_ &&
              subscriber_.game_robot_hp_.red_outpost_hp > 100) &&
             !defense_state_->reached_;

  if (!subscriber_.referee_is_online_)
    escape = false;

  return escape;
}

void StateMachine::publishFsmData()
{
  fsm_to_referee_pub_data_.power_limit_state = subscriber_.chassis_cmd_sender_->power_limit_->getState();
  fsm_to_referee_pub_data_.shoot_frequency =
      subscriber_.union_cmd_sender_->double_barrel_cmd_sender_->getShootFrequency();
  fsm_to_referee_pub_data_.gimbal_eject = subscriber_.union_cmd_sender_->gimbal_cmd_sender_->getEject();
  fsm_to_referee_pub_data_.det_armor_target = switch_detection_srv_->getArmorTarget();
  fsm_to_referee_pub_data_.det_color = switch_detection_srv_->getColor();
  fsm_to_referee_pub_data_.det_exposure = switch_detection_srv_->getExposureLevel();
  fsm_to_referee_pub_data_.det_target = switch_detection_srv_->getTarget();
  fsm_to_referee_pub_data_.stamp = ros::Time::now();
  fsm_to_referee_pub_.publish(fsm_to_referee_pub_data_);
}

bool StateMachine::checkOutpostActive()
{
  bool outpost_state;
  if (subscriber_.referee_is_online_)
  {
    if (subscriber_.game_robot_status_.robot_id > 100)
      outpost_state = subscriber_.game_robot_hp_.blue_outpost_hp > outpost_risk_hp_;
    if (subscriber_.game_robot_status_.robot_id < 100)
      outpost_state = subscriber_.game_robot_hp_.red_outpost_hp > outpost_risk_hp_;
  }
  else
    outpost_state = false;

  return outpost_state;
}

void StateMachine::switchTarget()
{
  std::string robot_color = subscriber_.game_robot_status_.robot_id >= 100 ? "blue" : "red";
  switch_detection_srv_->setEnemyColor(subscriber_.game_robot_status_.robot_id, robot_color);
  switch_detection_srv_->switchTargetType();
  info("switched target");
}

void StateMachine::update()
{
  context_.checkRc();
  publishFsmData();
  controllerUpdate();
  sendPathData();
  initGyroState();
}

void StateMachine::sendPathData()
{
  if (subscriber_.goal_planner_update_)
  {
    rm_msgs::MapSentryData map_sentry_data{};
    pathTransform(subscriber_.goal_planner_, &map_sentry_data);
    subscriber_.map_sentry_data_pub_.publish(map_sentry_data);
    subscriber_.goal_planner_update_ = false;
  }
}

void StateMachine::pathTransform(const nav_msgs::Path& goal_path, rm_msgs::MapSentryData* map_sentry_data)
{
  map_sentry_data->stamp = goal_path.header.stamp;
  int num = 0;
  int step = ceil(goal_path.poses.size() / 10);
  for (const auto& goal : goal_path.poses)
  {
    if (step != 0)
    {
      if (num == 0)
        pathPointTransform(map_sentry_data, goal, num / step - 1, true);
      else if (num % step == 0)
        pathPointTransform(map_sentry_data, goal, num / step - 1, false);
    }
    num = num + 1;
  }
}

void StateMachine::pathPointTransform(rm_msgs::MapSentryData* map_sentry_data, const geometry_msgs::PoseStamped& goal,
                                      int num, bool is_start_point)
{
  tf2::Transform minimap2path, world2path;
  world2path.setOrigin(tf2::Vector3(goal.pose.position.x, goal.pose.position.y, 0.0));
  world2path.setRotation(tf2::Quaternion(0, 0, 0, 1));
  minimap2path = conduct_state_->minimap2world_ * world2path;
  if (is_start_point)
  {
    map_sentry_data->start_position_x = minimap2path.getOrigin().x() * 10.0;
    map_sentry_data->start_position_y = minimap2path.getOrigin().y() * 10.0;
  }
  else
  {
    map_sentry_data->delta_x[num] = (int8_t)(minimap2path.getOrigin().x() * 10.0 - last_point_x_ * 10.0);
    map_sentry_data->delta_y[num] = (int8_t)(minimap2path.getOrigin().y() * 10.0 - last_point_y_ * 10.0);
  }
  last_point_x_ = minimap2path.getOrigin().x();
  last_point_y_ = minimap2path.getOrigin().y();
}

void StateMachine::initGyroState()
{
  if (init_gyro_state_ && !enable_gyro_srv_->isGyro())
    setGyroState();
  if (enable_gyro_srv_->isGyro())
    init_gyro_state_ = false;
}