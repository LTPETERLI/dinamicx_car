//
// Created by aung on 2023/3/3.
//

#pragma once

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <utility>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>

#include <rm_common/filters/filters.h>
#include <rm_common/ori_tool.h>
#include <rm_common/ros_utilities.h>

#include <rm_msgs/TagMsgArray.h>

namespace rm_localization
{
template <class MsgType>
class ReceiverBase
{
public:
  ReceiverBase(ros::NodeHandle& nh, std::string camera_frame, int moving_average_num, tf2_ros::Buffer* tf_buffer,
               std::string topic)
    : camera_frame_(std::move(camera_frame)), tf_buffer_(tf_buffer), tf_listener_(*tf_buffer)
  {
    msg_sub_.subscribe(nh, topic, 100);
    msg_sub_.registerCallback(boost::bind(&ReceiverBase::msgCallback, this, _1));

    filter_x_ = new MovingAverageFilter<double>(moving_average_num);
    filter_y_ = new MovingAverageFilter<double>(moving_average_num);
    filter_z_ = new MovingAverageFilter<double>(moving_average_num);

    map2odom_.setOrigin(tf2::Vector3(0, 0, 0));
    map2odom_.setRotation(tf2::Quaternion(0, 0, 0, 1));

    map2odom_msg_.header.stamp = ros::Time::now();
    map2odom_msg_.header.frame_id = "map";
    map2odom_msg_.child_frame_id = "odom";
    map2odom_msg_.transform = tf2::toMsg(map2odom_);
  }

  void mapToOdomBroadcaster()
  {
    map2odom_msg_.header.stamp = ros::Time::now();
    tf_broadcaster_.sendTransform(map2odom_msg_);
  }

protected:
  void msgCallback(const boost::shared_ptr<const MsgType>& msg)
  {
    stamp_ = msg->header.stamp;

    getCamToBase();

    fuseDetectedTags(msg);
    if (fabs(weight_sum_) < 1e-8)
      return;

    getFusedMapToBase();

    getBaseToOdom();

    mapToOdomTransform();
  }

  void getCamToBase()
  {
    try
    {
      geometry_msgs::TransformStamped tf_msg = tf_buffer_->lookupTransform(camera_frame_, "base_link", ros::Time(0));
      tf2::fromMsg(tf_msg.transform, cam2base_);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
    }
  }
  void getBaseToOdom()
  {
    try
    {
      geometry_msgs::TransformStamped tf_msg = tf_buffer_->lookupTransform("base_link", "odom", stamp_);
      tf2::fromMsg(tf_msg.transform, base2odom_);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
    }
  }

  void getMapToBase()
  {
    tf2::Transform tag2base;
    tag2base = cam2tag_.inverse() * cam2base_;

    // Lookup map -> base Transform
    tf2::Transform map2tag;
    try
    {
      geometry_msgs::TransformStamped tf_msg =
          tf_buffer_->lookupTransform("map", "tag" + std::to_string(static_cast<int>(tag_id_)), ros::Time(0));
      tf2::fromMsg(tf_msg.transform, map2tag);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
    }

    // Base to map
    tf2::Transform map2base;
    map2base = map2tag * tag2base;
    // Send current tag location
    geometry_msgs::TransformStamped map2base_msg;
    map2base_msg.header.stamp = stamp_;
    map2base_msg.header.frame_id = "map";
    map2base_msg.child_frame_id = "not_fuse_base" + std::to_string(static_cast<int>(tag_id_));
    map2base_msg.transform = tf2::toMsg(map2base);
    tf_broadcaster_.sendTransform(map2base_msg);

    double cur_weight = 1 / cam2tag_v_.length();

    // Sum Up Position Estimate
    estimate_x_ += cur_weight * map2base.getOrigin().getX();
    estimate_y_ += cur_weight * map2base.getOrigin().getY();
    estimate_z_ += cur_weight * map2base.getOrigin().getZ();
    weight_sum_ += cur_weight;
    // Ori Estimate
    tags_quat_.push_back(map2base.getRotation());
    tags_weight_.push_back(cur_weight);
  }

  void mapToOdomTransform()
  {
    // set map -> odom
    map2odom_ = map2base_ * base2odom_;

    map2odom_msg_.header.stamp = stamp_;
    map2odom_msg_.header.frame_id = "map";
    map2odom_msg_.child_frame_id = "odom";
    map2odom_msg_.transform = tf2::toMsg(map2odom_);
    tf_broadcaster_.sendTransform(map2odom_msg_);
  }

  void getFusedMapToBase()
  {
    filter_x_->input(estimate_x_ / weight_sum_);
    filter_y_->input(estimate_y_ / weight_sum_);
    filter_z_->input(estimate_z_ / weight_sum_);
    estimate_x_ = 0;
    estimate_y_ = 0;
    estimate_z_ = 0;
    weight_sum_ = 0;
    tf2::Quaternion average_quat = getAverageQuaternion(tags_quat_, tags_weight_);
    tags_quat_ = {};
    tags_weight_ = {};

    map2base_.setOrigin(tf2::Vector3(filter_x_->output(), filter_y_->output(), filter_z_->output()));
    map2base_.setRotation(average_quat);

    // Send current tag location
    geometry_msgs::TransformStamped new_map2base_msg;
    new_map2base_msg.header.stamp = stamp_;
    new_map2base_msg.header.frame_id = "map";
    new_map2base_msg.child_frame_id = "fused_base";
    new_map2base_msg.transform = tf2::toMsg(map2base_);

    tf_broadcaster_.sendTransform(new_map2base_msg);
  }

  enum
  {
    MISIDENTIFICATION = 6
  };

  tf2_ros::Buffer* tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string camera_frame_;
  MovingAverageFilter<double>*filter_x_, *filter_y_, *filter_z_;
  message_filters::Subscriber<MsgType> msg_sub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped map2odom_msg_;
  tf2::Transform cam2tag_, cam2base_, map2base_, base2odom_, map2odom_;
  ros::Time stamp_;
  std::vector<tf2::Quaternion> tags_quat_;
  std::vector<double> tags_weight_;
  double estimate_x_{}, estimate_y_{}, estimate_z_{}, weight_sum_{};
  unsigned long tag_id_{};
  tf2::Vector3 cam2tag_v_;
  tf2::Quaternion cam2tag_q_;

private:
  virtual void fuseDetectedTags(const boost::shared_ptr<const MsgType>& msg) = 0;
};

class RmDetectionReceiver : public ReceiverBase<rm_msgs::TagMsgArray>
{
public:
  using ReceiverBase<rm_msgs::TagMsgArray>::ReceiverBase;

private:
  void fuseDetectedTags(const rm_msgs::TagMsgArray::ConstPtr& msg) override
  {
    // If no detection
    if (msg->tag_msgs_array.empty())
      return;

    // Loop all detections
    for (const auto& detection : msg->tag_msgs_array)
    {
      tag_id_ = static_cast<unsigned long>(detection.letter);
      tf2::fromMsg(detection.pose.position, cam2tag_v_);
      tf2::convert(detection.pose.orientation, cam2tag_q_);
      if (tag_id_ != MISIDENTIFICATION)
      {
        cam2tag_.setOrigin(cam2tag_v_);
        cam2tag_.setRotation(cam2tag_q_);
      }
      else
        continue;

      getMapToBase();
    }
  }
};

class AprilTagReceiver : public ReceiverBase<apriltag_ros::AprilTagDetectionArray>
{
public:
  using ReceiverBase<apriltag_ros::AprilTagDetectionArray>::ReceiverBase;

private:
  void fuseDetectedTags(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) override
  {
    if (msg->detections.empty())
      return;
    // Loop all detections
    for (const auto& detection : msg->detections)
    {
      tag_id_ = static_cast<unsigned long>(detection.id[0]);
      tf2::fromMsg(detection.pose.pose.pose.position, cam2tag_v_);
      tf2::convert(detection.pose.pose.pose.orientation, cam2tag_q_);

      cam2tag_.setOrigin(cam2tag_v_);
      cam2tag_.setRotation(cam2tag_q_);

      getMapToBase();
    }
  }
};

}  // namespace rm_localization
