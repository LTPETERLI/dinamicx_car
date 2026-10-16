//
// Created by yawara on 24-1-9.
//

#include "track.h"

namespace robot_track{
Track::Track(ros::NodeHandle& nh) {
    linear_x_pid_.reset();
    linear_y_pid_.reset();
    ros::NodeHandle linear_x_nh(nh, "linear_x");
    ros::NodeHandle linear_y_nh(nh, "linear_y");
    ros::NodeHandle track_nh(nh, "track");

    if (!linear_x_pid_.init(ros::NodeHandle(linear_x_nh, "pid")) ||
        !linear_y_pid_.init(ros::NodeHandle(linear_y_nh, "pid")) ) {
        ROS_WARN("pid has not define");
    }
    track_nh.getParam("robot_distance", robot_distance_);

    tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
    track_sub_ = nh.subscribe<rm_msgs::TrackData>("/track", 1, &Track::detectionCallback, this);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
}

void Track::detectionCallback(const rm_msgs::TrackData::ConstPtr& msg) { track_buffer_ = *msg; }

void Track::SpeedTfTargetToBase() {
    distance_in_camera_.position = track_buffer_.position;
    try{
        camera2base_ = tf_buffer_.lookupTransform("base_link", "camera_optical_frame", ros::Time(0));
        tf2::fromMsg(camera2base_.transform, camera2base);
    } catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        return;
    }
    distance_in_base_ =
            camera2base.inverse().getBasis() * tf2::Vector3(distance_in_camera_.position.x, distance_in_camera_.position.y, distance_in_camera_.position.z) + camera2base.inverse().getOrigin();
//    tf2::doTransform(distance_in_camera_, distance_in_base_, camera2base_);
//    camera2base_ tf.::vector3(distance_in_camera_.position.x, distance_in_camera_.position.y, distance_in_camera_.position.z);

}

void Track::Run() {
//    current_time = ros::Time::now();

    SpeedTfTargetToBase();
//    dt_ = current_time - last_time_;

    ROS_INFO("distance_in_base_.x: %f",distance_in_base_.getX());
    ROS_INFO("distance_in_base_.y: %f",distance_in_base_.getY());
//    ROS_INFO("distance_in_base_.z: %f",distance_in_base_.getZ());
//    if(std::sqrt(std::pow(distance_in_base_.getX(), 2) + std::pow(distance_in_base_.getY(), 2) + std::pow(distance_in_base_.getZ(), 2)) > robot_distance_) {
//    if(std::hypot(distance_in_base_.getX(),distance_in_base_.getY()) > robot_distance_) {
//        ROS_INFO("Pid Test");
//        cmd_vel_.linear.x = linear_x_pid_.computeCommand(distance_in_base_.getX(),dt_);
//        cmd_vel_.linear.y = linear_y_pid_.computeCommand(distance_in_base_.getY(),dt_);
//    } else {
//        cmd_vel_.linear.x = 0;
//        cmd_vel_.linear.y = 0;
//    }
//    vel_pub.publish(cmd_vel_);
//    last_time_ = current_time;
}

}// robot_track