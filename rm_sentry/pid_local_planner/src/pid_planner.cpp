#include "pid_local_planner/pid_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(pid_local_planner::PidPlanner, nav_core::BaseLocalPlanner)

namespace pid_local_planner {
PidPlanner::PidPlanner()
        : initialized_(false)
        , tf_(nullptr)
        , costmap_ros_(nullptr)
        , goal_reached_(false)
        , plan_index_(0)
        , base_frame_("base_link")
        , map_frame_("map") {
    linear_x_pid_.reset();
    linear_y_pid_.reset();
    angular_z_pid_.reset();
}

void PidPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
    if (!initialized_) {
        initialized_ = true;
        tf_ = tf;
        costmap_ros_ = costmap_ros;

        ros::NodeHandle nh = ros::NodeHandle("~/" + name);
        ROS_INFO_STREAM(name);

        nh.param("slide_window", slide_window_, 0.2);
        nh.param("position_tolerance", position_tolerance_, 0.2);
        nh.param("k_theta", k_theta_, 0.5);
        nh.param("orientation_tolerance", orientation_tolerance_, 0.5);

        ros::NodeHandle linear_x_nh(nh, "linear_x");
        ros::NodeHandle linear_y_nh(nh, "linear_y");
        ros::NodeHandle angular_z_nh(nh, "angular_z");
        if (!linear_x_pid_.init(ros::NodeHandle(linear_x_nh, "pid")) ||
            !linear_y_pid_.init(ros::NodeHandle(linear_y_nh, "pid")) ||
            !angular_z_pid_.init(ros::NodeHandle(angular_z_nh, "pid")))
            ROS_WARN("pid has not define");

        odom_helper_ = new base_local_planner::OdometryHelperRos("/ep/odom");
        target_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/target_pose", 10);
        current_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);
        last_time_ = ros::Time::now();

        ROS_INFO("pid planner initialized!");
    }
    else {
        ROS_WARN("pid planner has already been initialized.");
    }
}

bool PidPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
        return false;
    }

    // set new plan
    global_plan_.clear();
    global_plan_ = orig_global_plan;

    // reset plan parameters
    plan_index_ = 4;

    geometry_msgs::PoseStamped current_ps_odom, current_ps_map;
    costmap_ros_->getRobotPose(current_ps_odom);
    tf_->transform(current_ps_odom, current_ps_map, map_frame_);

    if (current_ps_map.pose.position.x != global_plan_.back().pose.position.x || current_ps_map.pose.position.y != global_plan_.back().pose.position.y) {
        goal_yaw = tf2::getYaw(global_plan_.back().pose.orientation);
        goal_reached_ = false;
    }

    return true;
}

bool PidPlanner::isGoalReached() {
    if (!initialized_) {
        ROS_ERROR("pid planner has not been initialized");
        return false;
    }
    if (goal_reached_) {
        ROS_INFO("GOAL Reached!");
        return true;
    }

    return false;
}

bool PidPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    if (!initialized_) {
        ROS_ERROR("pid planner has not been initialized");
        return false;
    }

    ros::Time current_time = ros::Time::now();

    // current pose
    geometry_msgs::PoseStamped current_ps_odom;
    costmap_ros_->getRobotPose(current_ps_odom);

    // transform into map
    tf_->transform(current_ps_odom, current_ps_, map_frame_);

    double cur_x, cur_y;
    cur_x = current_ps_.pose.position.x;
    cur_y = current_ps_.pose.position.y;

    // desired x, y, yaw in base frame
    double b_x_d, b_y_d, base_angle;
    double e_theta;
    double yaw = tf2::getYaw(current_ps_odom.pose.orientation);

    e_theta = goal_yaw - yaw;
    if(e_theta > M_PI) {
        e_theta -= 2 * M_PI;
    }
    if(e_theta < -M_PI){
        e_theta += 2 * M_PI;
    }

    while (plan_index_ < global_plan_.size())
    {
        if(plan_index_ < global_plan_.size()*0.8){//剩余路程大于0.25m，则不开启限速
            is_speed_limit_=false;
        }else{
            if(plan_index_ > global_plan_.size()*0.95){//剩余路程小于0.21m，限速0.2m
                limit_speed_ = 0.1;
            }else{//剩余路程>0.23且<0.28m，限速0.35
                limit_speed_ = 0.2;
            }
            is_speed_limit_ = true;
        }
        target_ps_ = global_plan_[plan_index_];

        tf2::Quaternion q;
        tf2::convert(q, target_ps_.pose.orientation);

        // transform from map into base_frame
        getTransformedPosition(target_ps_, b_x_d, b_y_d, base_angle);

        if (std::hypot(b_x_d, b_y_d) > slide_window_) {
            break;
        }
        ++plan_index_;
    }

    dt_ = current_time - last_time_;
    last_time_ = current_time;

    if (plan_index_ >= global_plan_.size() - 1)//剩余路程小于0.2m
    {
        getTransformedPosition(global_plan_.back(), b_x_d, b_y_d, base_angle);
        is_speed_limit_ = true;
        limit_speed_ = 0.2; //最后一个点，限速0.2m
        if(std::hypot(b_x_d, b_y_d) < 0.08){
            limit_speed_ = 0.1;
            gainError(b_x_d,0.03,0.025);
            gainError(b_y_d,0.03,0.025);
        }
    }
    nav_msgs::Odometry base_odom;
    odom_helper_->getOdom(base_odom);

    // position reached
    if (isReached(global_plan_.back(), cur_x, cur_y)) {
        // orientation reached
        if(std::fabs(e_theta) <= orientation_tolerance_) {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
            goal_reached_ = true;
        }else {         // orientation not reached
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = angular_z_pid_.computeCommand(e_theta, ros::Duration(dt_));
        }
    } else {             // posistion not reached
        cmd_vel.linear.x = linear_x_pid_.computeCommand(b_x_d, ros::Duration(dt_));
        cmd_vel.linear.y = linear_y_pid_.computeCommand(b_y_d, ros::Duration(dt_));
        cmd_vel.angular.z = angular_z_pid_.computeCommand(e_theta, ros::Duration(dt_));

        if(is_speed_limit_) {
            limitSpeed(cmd_vel.linear.x,limit_speed_);
            limitSpeed(cmd_vel.linear.y,limit_speed_);
        }
    }
    if(fabs(e_theta) >= 0.15) {//偏差角大于8.6度，角速度拉满了跑
        if(cmd_vel.angular.z > 0) {
            cmd_vel.angular.z = 0.5;
        }else if(cmd_vel.angular.z < 0) {
            cmd_vel.angular.z -= 0.5;
        }
    }else if(fabs(e_theta) < 0.08) {
        if(cmd_vel.angular.z > 0.1) {
            cmd_vel.angular.z = 0.1;
        }else if(cmd_vel.angular.z < -0.1) {
            cmd_vel.angular.z -= 0.1;
        }
    }

    // publish target pose
    target_pose_pub_.publish(target_ps_);

    // publish robot current pose
    current_pose_pub_.publish(current_ps_);

    return true;
}

bool PidPlanner::isReached(const geometry_msgs::PoseStamped &goal_ps, double x, double y) {
    if((fabs(x - goal_ps.pose.position.x)<position_tolerance_) && (fabs(y - goal_ps.pose.position.y)<position_tolerance_)) {
        return true;
    }else {
        return false;
    }
}

void PidPlanner::getTransformedPosition(geometry_msgs::PoseStamped& input, double& x, double& y, double& angle) {
    geometry_msgs::PoseStamped output;

    input.header.stamp = ros::Time(0);
    tf_->transform(input, output, base_frame_);

    x = output.pose.position.x;
    y = output.pose.position.y;
    angle = tf2::getYaw(output.pose.orientation);
}

void PidPlanner::gainError(double &error, double threshold, double gain_value) {
    //add error of gain
    if(error >= threshold) {
        error += gain_value;
    }else if(error <= -threshold) {
        error -= gain_value;
    }
}

void PidPlanner::limitSpeed(double &speed, double &limit_speed) {
    if(speed > limit_speed) {
        speed = limit_speed;
    }else if(speed < -limit_speed) {
        speed -= limit_speed;
    }
}

}  // namespace pid_local_planner