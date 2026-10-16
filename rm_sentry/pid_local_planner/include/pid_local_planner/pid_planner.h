#pragma once

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <control_toolbox/pid.h>

#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>

#include <nav_msgs/Odometry.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <tf2/utils.h>
#include <Eigen/Dense>

namespace pid_local_planner {
class PidPlanner : public nav_core::BaseLocalPlanner {
public:
    PidPlanner();
    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) override;
    bool isGoalReached() override;
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;

private:
    void getTransformedPosition(geometry_msgs::PoseStamped& src, double& x, double& y, double& angle);
    void gainError(double &error, double threshold, double gain_value);
    void limitSpeed(double &speed, double &limit_speed);
    bool isReached(const geometry_msgs::PoseStamped& goal_ps, double x, double y);

    int plan_index_;
    bool initialized_, goal_reached_;
    bool is_speed_limit_{false};
    double limit_speed_{0.0};
    double slide_window_;        // next point distance/turning angle
    double position_tolerance_, orientation_tolerance_;
    double k_theta_, goal_yaw;

    tf2_ros::Buffer* tf_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    base_local_planner::OdometryHelperRos* odom_helper_;

    std::string base_frame_, map_frame_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    geometry_msgs::PoseStamped target_ps_, current_ps_;

    ros::Publisher target_pose_pub_, current_pose_pub_;
    ros::Time last_time_{ ros::Time::now() };
    ros::Duration dt_;

    control_toolbox::Pid linear_x_pid_;
    control_toolbox::Pid linear_y_pid_;
    control_toolbox::Pid angular_z_pid_;
};
}  // namespace pid_local_planner
