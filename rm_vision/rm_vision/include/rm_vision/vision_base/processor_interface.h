//
// Created by heyicheng on 10/6/21.
//

#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace rm_vision 
{

class ProcessorInterface 
{
 public:
  virtual void initialize(ros::NodeHandle &nh) = 0;

  virtual void imageProcess(cv_bridge::CvImagePtr &cv_image) = 0;

  virtual void findArmor() = 0;

  virtual void paramReconfig() = 0;

  virtual void draw() = 0;

  std::vector<std::vector<cv::Point2i>> points;
  std::vector<int> id;

  typedef struct
  {
    std::vector<std::vector<cv::Point2d>> points;
    std::vector<int> id;
    std::vector<float> prob;
  } Object;

  virtual Object getObj() = 0;

  virtual void putObj() = 0;

};

}  // namespace rm_vision
