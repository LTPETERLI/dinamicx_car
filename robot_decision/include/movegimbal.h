//
// Created by yawara on 23-8-29.
//

#ifndef SRC_MOVEGIMBAL_H
#define SRC_MOVEGIMBAL_H

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "ros/ros.h"

#include <rm_msgs/GimbalCmd.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <rm_msgs/TrackData.h>
#include "rm_common/ori_tool.h"

namespace behavior_tree{
class MoveGimbal : public BT::SyncActionNode{
public:
    MoveGimbal(const std::string& name, const BT::NodeConfiguration& config, ros::NodeHandle& root_nh,
               const ros::NodeHandle& decision_nh);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    std::unique_ptr<realtime_tools::RealtimePublisher<rm_msgs::GimbalCmd>> gimbal_pub_;
    void MoveGimbalRate();

    geometry_msgs::TransformStamped base2yaw;
    geometry_msgs::TransformStamped yaw2pitch;

    rm_msgs::TrackData track_buffer_;
    ros::Subscriber track_sub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener* tf_listener_;

    double yaw_rate_{0.5}, pitch_rate_{0.7};
    double pitch_lower_limit_{-0.29}, pitch_upper_limit_{0.25};
};
}// behavior_tree

#endif //SRC_MOVEGIMBAL_H
