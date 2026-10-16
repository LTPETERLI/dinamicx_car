//
// Created by kaxns on 3/19/22.
//

#include "windmill_dev/windmill_analyzer.h"

namespace rm_windmill_vision
{
void WindmillAnalyzerNodelet::onInit()
{
  ros::NodeHandle nh = getPrivateNodeHandle();
  this->it_ = std::make_shared<image_transport::ImageTransport>(nh);
  if (initCameraInfo(nh) != SUCCESS)
    nh.shutdown();
  this->roi_sub_ = it_->subscribe("/windmill_vision/roi_img", 1, &WindmillAnalyzerNodelet::callback, this);
  this->distance_ = nh.param("distance", 0.7);
  constructLUT(nh.param("big_armor_width", 0.235), nh.param("big_armor_height", 0.127), distance_);
  target_pub_ = nh.advertise<rm_msgs::TargetDetectionArray>("/processor/result_msg", 1);
  prediction_pub_ = nh.advertise<rm_msgs::TargetDetectionArray>("/prediction", 1);
  auto windmill_color = nh.param("windmill_color", std::string("red"));
  setColorMethod(windmill_color);
  this->threshold_ = nh.param("binary_threshold", 100);
  initReConfig(nh);
}

void WindmillAnalyzerNodelet::callback(const sensor_msgs::ImageConstPtr& src)
{
  this->sync_ = src->header;
  cv::Mat3b rois = cv_bridge::toCvShare(src, "bgr8")->image.clone();
  execute(rois);
}
}  // namespace rm_windmill_vision

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rm_windmill_vision::WindmillAnalyzerNodelet, nodelet::Nodelet)