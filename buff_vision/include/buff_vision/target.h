//
// Created by koonyu on 2023/1/25.
//

#pragma once
#include <opencv2/opencv.hpp>
#include <utility>

namespace rm_buff_vision
{
class Target
{
 public:
  Target(){};
  ~Target(){};

  void getRectLength(int length)
  {
    length_ = length;
  }

  int returnRectLength()
  {
    return length_;
  }

  void getTargetVector(cv::Point2f vector)
  {
    target_vector_ = std::move(vector);
  }

  void returnTargetVector(cv::Point2f& vector)
  {
    vector = target_vector_;
  }

  void getCenter(cv::Point2f center)
  {
    center_ = std::move(center);
  }

  void returnCenter(cv::Point2f& center)
  {
    center = center_;
  }

  void getArmorPoints(std::array<cv::Point, 4> points)
  {
    armor_points_ = std::move(points);
  }

  void returnArmorPoints(std::array<cv::Point, 4>& points)
  {
    points = armor_points_;
  }

  void getRectPoints(std::array<cv::Point2f, 4> points)
  {
    rect_points_ = std::move(points);
  }

  void returnRectPoints(std::array<cv::Point2f, 4>& points)
  {
    points = rect_points_;
  }

  void getRCVector(cv::Point2f vector)
  {
    rc_vector_ = std::move(vector);
  }

  void returnRCVector(cv::Point2f& vector)
  {
    vector = rc_vector_;
  }

 private:
  int length_;
  cv::Point2f target_vector_{};
  cv::Point2f center_{};
  std::array<cv::Point, 4> armor_points_{};
  std::array<cv::Point2f, 4> rect_points_{};
  cv::Point2f rc_vector_{};
};

} // namespace rm_buff_vision