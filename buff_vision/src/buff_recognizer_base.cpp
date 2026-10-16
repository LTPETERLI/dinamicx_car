//
// Created by koonyu on 2023/1/28.
//

#include "../include/rm_cgal_tools_polygon_simplification_2.h"
#include "../include/buff_vision/buff_recognizer.h"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace rm_buff_vision
{
bool BuffRecognizerBase::locateNotTriggeredFan(cv::Mat1b &sel)
{
  // clear containers for storing data of next frame every time
  is_hammer_.clear();

  // TODO: size of searching_area can be adjustable
  static cv::Rect2i const SEARCHING_AREA(cv::Point2i(sel.cols / 6, sel.rows / 6), sel.size() * 4 / 6);
#if ENABLE_VIRTUALIZATION
  cv::rectangle(*img_to_draw_, SEARCHING_AREA, cv::Scalar(100, 100, 100), 1);
#endif
  // check if there are enough points in area_to_search
  if (cv::countNonZero(sel(SEARCHING_AREA)) < SEARCHING_AREA.height + SEARCHING_AREA.width)
    return false;

  // indeed enough points, let's move on
  external_contours_type ext_contours;  // external contours
  cv::findContours(sel, ext_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

  findHammer(ext_contours, sel);

  is_found_ = false;
  checkHammerAndLocateRCenter(ext_contours);

  if(is_found_)
  {
    fixRectPoints();
    extractHammerRoi(hammer_.returnRectLength());
    return true;
  }
  return false;
}

void BuffRecognizerBase::findHammer(external_contours_type& ext_contours, cv::Mat1b& sel)
{
  for (auto& ext_contour : ext_contours)
  {
    auto r_rect = cv::minAreaRect(ext_contour);
    if (r_rect.size.area() < 20 * 20)
      continue;
    cv::Point2f pts[4];
    r_rect.points(pts);

    auto dist_1 = ::abs(pts[0].x - pts[1].x) + ::abs(pts[0].y - pts[1].y);
    auto dist_2 = ::abs(pts[1].x - pts[2].x) + ::abs(pts[1].y - pts[2].y);

    float area = cv::contourArea(ext_contour);
    float rect_ratio = dist_1 > dist_2 ? dist_1 / dist_2 : dist_2 / dist_1;
    float white_ratio = area / r_rect.size.area();

    if((area > hammer_min_area_ && area < hammer_max_area_) && (rect_ratio > hammer_min_rect_ratio_ && rect_ratio < hammer_max_rect_ratio_)
    && (white_ratio > min_white_ratio_ && white_ratio < max_white_ratio_))
    {
      int length;
      cv::Point median_pt, split_center_1, split_center_2;
      if(dist_1 > dist_2)
      {
        median_pt = cv::Point((pts[0].x + pts[1].x) / 2, (pts[0].y + pts[1].y) / 2);
        split_center_1 = cv::Point((median_pt.x + pts[2].x) / 2, (median_pt.y + pts[2].y) / 2);
        split_center_2 = cv::Point((median_pt.x + pts[3].x) / 2, (median_pt.y + pts[3].y) / 2);
        length = static_cast<int>(dist_1);
      }
      else
      {
        median_pt = cv::Point((pts[1].x + pts[2].x) / 2, (pts[1].y + pts[2].y) / 2);
        split_center_1 = cv::Point((median_pt.x + pts[0].x) / 2, (median_pt.y + pts[0].y) / 2);
        split_center_2 = cv::Point((median_pt.x + pts[3].x) / 2, (median_pt.y + pts[3].y) / 2);
        length = static_cast<int>(dist_2);
      }

      cv::Rect rect_1(split_center_1.x - big_rect_size_, split_center_1.y - big_rect_size_, big_rect_size_ * 2, big_rect_size_ * 2);
      cv::Rect rect_2(split_center_2.x - big_rect_size_, split_center_2.y - big_rect_size_, big_rect_size_ * 2, big_rect_size_ * 2);

      if(split_center_1.x - big_rect_size_<=0 || split_center_1.y - big_rect_size_<=0 || split_center_2.x - big_rect_size_<=0 || split_center_2.y - big_rect_size_<=0 ||
          split_center_1.x + big_rect_size_>=sel.cols - 1 || split_center_1.y + big_rect_size_>=sel.rows - 1 || split_center_2.x + big_rect_size_>=sel.cols - 1 || split_center_2.y + big_rect_size_>=sel.rows - 1)
        continue;

#if ENABLE_VIRTUALIZATION
      std::array<cv::Point, 4> pts1 = {cv::Point(split_center_1.x - big_rect_size_, split_center_1.y - big_rect_size_),
                                       cv::Point(split_center_1.x + big_rect_size_, split_center_1.y - big_rect_size_),
                                       cv::Point(split_center_1.x + big_rect_size_, split_center_1.y + big_rect_size_),
                                       cv::Point(split_center_1.x - big_rect_size_, split_center_1.y + big_rect_size_)};
      std::array<cv::Point, 4> pts2 = {cv::Point(split_center_2.x - big_rect_size_, split_center_2.y - big_rect_size_),
                                       cv::Point(split_center_2.x + big_rect_size_, split_center_2.y - big_rect_size_),
                                       cv::Point(split_center_2.x + big_rect_size_, split_center_2.y + big_rect_size_),
                                       cv::Point(split_center_2.x - big_rect_size_, split_center_2.y + big_rect_size_)};
      cv::polylines(*img_to_draw_, pts1, true, cv::Scalar(255, 0, 255), 2);
      cv::polylines(*img_to_draw_, pts2, true, cv::Scalar(255, 0, 255), 2);
#endif

      cv::Mat dst_1 = (sel)(rect_1);
      int sum1 = static_cast<int>(cv::mean(dst_1).val[0]);
      cv::Mat dst_2 = (sel)(rect_2);
      int sum2 = static_cast<int>(cv::mean(dst_2).val[0]);

      cv::Point x_target;
      if(sum1 > sum2)
        x_target = split_center_1;
      else
        x_target = split_center_2;

      std::array<cv::Point, 8> sort = {pts[0], pts[1], pts[2], pts[3],
                                       pts[0], pts[1], pts[2], pts[3]};
      std::array<cv::Point, 4> points;
      float px_distance_1 = pow(pts[0].x - x_target.x, 2) + pow(pts[0].y - x_target.y, 2);
      float px_distance_2 = pow(pts[2].x - x_target.x, 2) + pow(pts[2].y - x_target.y, 2);
      float px_max_distance = px_distance_1 > px_distance_2 ? px_distance_1 : px_distance_2;
      for(int i = 0; i < 4; ++i)
      {
        if((pow(sort[i].x - x_target.x, 2) + pow(sort[i].y - x_target.y, 2) < px_max_distance * 0.6) &&
            (pow(sort[i + 1].x - x_target.x, 2) + pow(sort[i + 1].y - x_target.y, 2) < px_max_distance * 0.6))
          points = {sort[i], sort[i + 1], sort[i + 2], sort[i + 3]};
      }

      cv::Point2f pt1, pt2;
      pt1 = small_rect_move_ * (points[3] - points[0]) + points[0];
      pt2 = small_rect_move_ * (points[2] - points[1]) + points[1];

      rect_1 = cv::Rect(pt1.x - small_rect_size_, pt1.y - small_rect_size_, 2 * small_rect_size_, 2 * small_rect_size_);
      rect_2 = cv::Rect(pt2.x - small_rect_size_, pt2.y - small_rect_size_, 2 * small_rect_size_, 2 * small_rect_size_);

      if(pt1.x - small_rect_size_<=0 || pt1.y - small_rect_size_<=0 || pt2.x - small_rect_size_<=0 || pt2.y - small_rect_size_<=0 ||
          pt1.x + small_rect_size_>=sel.cols - 1 || pt1.y + small_rect_size_>=sel.rows - 1 || pt2.x + small_rect_size_>=sel.cols - 1 || pt2.y + small_rect_size_>=sel.rows - 1)
        continue;

#if ENABLE_VIRTUALIZATION
      std::array<cv::Point, 4> pts3 = {cv::Point(pt1.x - small_rect_size_, pt1.y - small_rect_size_),
                                       cv::Point(pt1.x + small_rect_size_, pt1.y - small_rect_size_),
                                       cv::Point(pt1.x + small_rect_size_, pt1.y + small_rect_size_),
                                       cv::Point(pt1.x - small_rect_size_, pt1.y + small_rect_size_)};
      std::array<cv::Point, 4> pts4 = {cv::Point(pt2.x - small_rect_size_, pt2.y - small_rect_size_),
                                       cv::Point(pt2.x + small_rect_size_, pt2.y - small_rect_size_),
                                       cv::Point(pt2.x + small_rect_size_, pt2.y + small_rect_size_),
                                       cv::Point(pt2.x - small_rect_size_, pt2.y + small_rect_size_)};
      cv::polylines(*img_to_draw_, pts3, true, cv::Scalar(255, 0, 255), 2);
      cv::polylines(*img_to_draw_, pts4, true, cv::Scalar(255, 0, 255), 2);
#endif

      dst_1 = (sel)(rect_1);
      sum1 = static_cast<int>(cv::mean(dst_1).val[0]);
      dst_2 = (sel)(rect_2);
      sum2 = static_cast<int>(cv::mean(dst_2).val[0]);
      if(sum1 > intensity_ || sum2 > intensity_)
        continue;

      hammer_ = {};
      ext_contour[0].x = -1;
      hammer_.getRectPoints({points[0], points[1], points[2], points[3]});
      hammer_.getRectLength(length);
      hammer_.getTargetVector(points[0] - points[3]);
      hammer_.getCenter(x_target);
      is_hammer_.push_back(hammer_);
      hammer_ = {};
#if ENABLE_VIRTUALIZATION
      cv::polylines(*img_to_draw_, points, true, cv::Scalar(0, 255, 0), 2);
#endif
    }
  }
}

void BuffRecognizerBase::checkHammerAndLocateRCenter(external_contours_type& ext_contours)
{
  std::vector<cv::Point2f> maybe_r_center{};
  bool r_is_found = false;
  // locate item looks like r icon
  for (auto& item : ext_contours)
  {
    // Previously, any contour which is possible to be a hammer , hammer has already had its first element marked.
    // Just skip it.
    auto bounding_rect = cv::boundingRect(item);
    if(item[0].x == -1 || bounding_rect.size().area() > r_icon_max_area_)
      continue;
    float rect_ratio = bounding_rect.width >= bounding_rect.height ? bounding_rect.width / bounding_rect.height : bounding_rect.height / bounding_rect.width;
    float area = bounding_rect.size().area();
    float length = 2 * bounding_rect.width + 2 * bounding_rect.height;

    if((rect_ratio > r_icon_min_rect_ratio_ && rect_ratio < r_icon_max_rect_ratio_) && (area > r_icon_min_area_ && area < r_icon_max_area_)
    && (length > r_icon_min_length_ && r_icon_max_area_))
      maybe_r_center.push_back((bounding_rect.tl() + bounding_rect.br()) / 2);
  }

  for(auto& hammer : is_hammer_)
  {
    cv::Point2f x_target;
    hammer.returnCenter(x_target);
    cv::Point2f hammer_vector;
    hammer.returnTargetVector(hammer_vector);

    for(auto& r_icon : maybe_r_center)
    {
      float rc_distance = sqrt(pow(x_target.x - r_icon.x, 2) + pow(x_target.y - r_icon.y, 2));
      cv::Point2f rc_vector = x_target - r_icon;

      float theta = abs(asin(hammer_vector.cross(rc_vector) / (norm(hammer_vector) * norm(rc_vector))));
      constexpr float rad_15_deg = 15. / 180. * CV_PI;
      constexpr float rad_0_deg = 0. / 180. * CV_PI;

      if ((theta < sin(rad_15_deg) && theta > sin(rad_0_deg)) && (rc_distance > min_rc_distance_ && rc_distance < max_rc_distance_))
      {
        r_center_ = r_icon;
        r_is_found = true;
        hammer_ = hammer;
        is_found_ = true;

        hammer_.getCenter(x_target);
        hammer_.getTargetVector(hammer_vector);
        hammer_.getRCVector(rc_vector);

#if ENABLE_VIRTUALIZATION
        cv::line(*img_to_draw_, r_icon, x_target, cv::Scalar(255, 0, 255), 2);
        cv::circle(*img_to_draw_, x_target, 4, cv::Scalar(0, 255, 0), -1);
        cv::circle(*img_to_draw_, r_icon, 4, cv::Scalar(0, 255, 0), -1);
#endif
        break;
      }
    }

    if(r_is_found)
      break;
  }
}

void BuffRecognizerBase::fixRectPoints()
{
  cv::Point2f x_target;
  hammer_.returnCenter(x_target);
  cv::Point2f hammer_vector;
  hammer_.returnTargetVector(hammer_vector);
  std::array<cv::Point2f, 4> rect_points;
  hammer_.returnRectPoints(rect_points);
  std::array<cv::Point2f, 4> pts = {rect_points[0], rect_points[1], rect_points[2], rect_points[3]};
  cv::Point2f rc_vector;
  hammer_.returnRCVector(rc_vector);

  float theta = asin(rc_vector.cross(hammer_vector) / (norm(hammer_vector) * norm(rc_vector)));
  float cos_theta = cos(theta);
  float sin_theta = sin(theta);

  cv::Mat_<float> rotation = (cv::Mat_<float>(2, 2) << cos_theta, -sin_theta,
                                                       sin_theta, cos_theta);
  cv::Mat_<float> vectors = (cv::Mat_<float>(2, 4) << pts[0].x - x_target.x, pts[1].x - x_target.x, pts[2].x - x_target.x, pts[3].x - x_target.x,
                                                      pts[0].y - x_target.y, pts[1].y - x_target.y, pts[2].y - x_target.y, pts[3].y - x_target.y);
  vectors = rotation.inv() * vectors;
  std::array<cv::Point2f, 4> rot_pts = {cv::Point2f(vectors[0][0] + x_target.x, vectors[1][0] + x_target.y),
                                      cv::Point2f(vectors[0][1] + x_target.x, vectors[1][1] + x_target.y),
                                      cv::Point2f(vectors[0][2] + x_target.x, vectors[1][2] + x_target.y),
                                      cv::Point2f(vectors[0][3] + x_target.x, vectors[1][3] + x_target.y)};
  hammer_.getRectPoints(rot_pts);
  hammer_.getRectLength(sqrt(pow(rot_pts[0].x - rot_pts[1].x, 2) + pow(rot_pts[0].y - rot_pts[1].y, 2)));
  std::array<cv::Point, 4> fixed_points = {rot_pts[0], rot_pts[1], (rot_pts[1] + rot_pts[2]) / 2, (rot_pts[3] + rot_pts[0]) / 2};
#if ENABLE_VIRTUALIZATION
  // These points can only represent approximate positions of armor's points.
  cv::polylines(*img_to_draw_, fixed_points, true, cv::Scalar(255, 255, 0), 2);
  cv::circle(*img_to_draw_, fixed_points[0], 4, cv::Scalar(255, 0, 0), -1);
  cv::circle(*img_to_draw_, fixed_points[1], 4, cv::Scalar(0, 255, 0), -1);
  cv::circle(*img_to_draw_, fixed_points[2], 4, cv::Scalar(0, 0, 255), -1);
  cv::circle(*img_to_draw_, fixed_points[3], 4, cv::Scalar(255, 0, 255), -1);
#endif
}

Target BuffRecognizerBase::hammer_{};
std::vector<Target> BuffRecognizerBase::is_hammer_{};
} // namespace rm_buff_vision