//
// Created by yawara on 24-1-9.
//

#ifndef SRC_TRACK_H
#define SRC_TRACK_H

#include "ros/ros.h"
#include <rm_msgs/TrackData.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <control_toolbox/pid.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>


namespace robot_track{
class Track{
public:
    explicit Track(ros::NodeHandle& nh);
    void Run();
private:
    tf2_ros::TransformListener* tf_listener_;
    tf2_ros::Buffer tf_buffer_;
    tf2::Transform camera2base;
    tf2::Vector3 distance_in_base_;

    geometry_msgs::Twist cmd_vel_;
    geometry_msgs::TransformStamped  camera2base_;
    geometry_msgs::Pose distance_in_camera_;
//    geometry_msgs::Pose distance_in_base_;

    float robot_distance_{0.5};
    ros::Duration dt_;
    ros::Time last_time_{ros::Time::now()};
    ros::Time current_time;

    ros::Publisher vel_pub;
    ros::Subscriber track_sub_;
    rm_msgs::TrackData track_buffer_;

    control_toolbox::Pid linear_x_pid_;
    control_toolbox::Pid linear_y_pid_;

    void SpeedTfTargetToBase();
    void detectionCallback(const rm_msgs::TrackData::ConstPtr& msg);
};
} // robot_track

#endif //SRC_TRACK_H
