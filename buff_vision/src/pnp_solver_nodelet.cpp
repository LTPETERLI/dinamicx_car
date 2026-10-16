//
// Created by koonyu on 2023/1/10.
//

#include "../include/buff_vision/pnp_solver.h"

namespace rm_buff_vision
{
void PnpSolverNodelet::onInit()
{
  ros::NodeHandle nh = getPrivateNodeHandle();
  initReConfig(nh);
  this->it_ = std::make_shared<image_transport::ImageTransport>(nh);
  if (initCameraInfo(nh) != SUCCESS)
    nh.shutdown();
  this->status_sub_ = nh.subscribe("/status", 1, &PnpSolverNodelet::statusCallback, this);
  this->roi_sub_ = it_->subscribe("/buff_vision/roi_img", 1, &PnpSolverNodelet::callback, this);

//  target_pub_ = nh.advertise<rm_msgs::TargetDetectionArray>("/detection", 1); // Used for PnpSolverBase::solvePnp
  prediction_pub_ = nh.advertise<rm_msgs::TargetDetectionArray>("/prediction", 1);
//  auto windmill_color = nh.param("buff_color", std::string("red"));
  this->threshold_ = nh.param("binary_threshold", 100);

  if(!nh.getParam("distance", distance_))
    ROS_WARN("No distance specified");
  if(!nh.getParam("armor_width", armor_width_))
    ROS_WARN("No armor_width specified");
  if(!nh.getParam("armor_height", armor_height_))
    ROS_WARN("No armor_height specified");
  if(!nh.getParam("stddev_typ", stddev_typ_))
    ROS_WARN("No distance specified");
  if(!nh.getParam("cv_typ", coefficient_of_variation_typ_))
    ROS_WARN("No cv_typ specified");
}

void PnpSolverNodelet::statusCallback(const rm_msgs::TargetDetectionArray& status)
{
  int32_t temp[2]{};
  for (const auto &detection : status.detections)
  {
    memcpy(&temp[0], &detection.pose.orientation.x, sizeof(int32_t));
    memcpy(&temp[1], &detection.pose.orientation.y, sizeof(int32_t));
  }

  target_is_red_ = temp[0];
  armor_type_ = temp[1];
}

void PnpSolverNodelet::callback(const sensor_msgs::ImageConstPtr& src)
{
  if (!armor_type_)
    return;
  this->sync_ = src->header;
  cv::Mat3b rois = cv_bridge::toCvShare(src, "bgr8")->image.clone();
  execute(rois);
}
} // namespace rm_buff_vision

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rm_buff_vision::PnpSolverNodelet, nodelet::Nodelet)
