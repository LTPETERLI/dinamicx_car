//
// Created by kaxns on 11/11/21.
//

#include "rm_cgal_tools_polygon_simplification_2.h"
#include "windmill_dev/windmill_recognizer.h"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace rm_windmill_vision
{
/**
 * @brief Return the sign of a number.
 * @result sign of a number
 */
template <typename T>
int sgn(T val)
{
  return (T{ 0 } < val) - (val < T{ 0 });
}

void WindmillRecognizerBase::findHammerAndFans(external_contours_type& ext_contours, hammer_type& hammer,
                                               fans_type* const fans)
{
  bool hammer_is_found = false;
  for (auto& ext_contour : ext_contours)
  {
    auto r_rect = cv::minAreaRect(ext_contour);
    if (r_rect.size.area() < 20 * 20)
      continue;
    cv::Point2f pts[4];
    r_rect.points(pts);
#if ENABLE_VIRTUALIZATION
    cv::polylines(*img_to_draw_, std::vector<cv::Point>{ pts[0], pts[1], pts[2], pts[3] }, true, cv::Scalar(255), 1);
#endif

    // calculate the direction vector of a rotated rect
    // def angle = the minimum of the angle between the long side and X axis and its supplementary angle
    // angle will only be calculated if input angle != nullptr
    auto direction_vector_calculator = [&pts](float* angle = nullptr) {
      // first_pt = pts[0], second_pt = pts[1], third_pt = pts[2]
      auto dist_1 = ::abs(pts[0].x - pts[1].x) + ::abs(pts[0].y - pts[1].y);
      auto dist_2 = ::abs(pts[1].x - pts[2].x) + ::abs(pts[1].y - pts[2].y);
      cv::Point2f direction_vec = dist_1 > dist_2 ? pts[1] - pts[0] : pts[2] - pts[1];

      if (angle != nullptr)
      {
        *angle = ::abs(acos(direction_vec.x) / sqrt(direction_vec.dot(direction_vec))) / CV_PI * 180;
        *angle = ::fmin(*angle, 180 - *angle);  // therefore, angle was limited into [0, 90]
      }
      return direction_vec;
    };

    // let height always >= width
    (void)(r_rect.size.height < r_rect.size.width && (std::swap(r_rect.size.height, r_rect.size.width), 1));
    float ratio = r_rect.size.height / r_rect.size.width;
    // height / width outside (1.5, 3.0) is definitely wrong
    if (!(ratio > 1.5 && ratio < 3.0))  // TODO: range of ratio can be adjustable
      continue;
    // If fans == nullptr, just find the hammer.
    // Contour area > 2/3, assuming it is a fan, otherwise it is a hammer.
    // Put them in different containers and label them.
    // With markers, they can be quickly skipped when looking for 'R' icon in the original set of contours.
    else if (cv::contourArea(ext_contour, false) / r_rect.size.area() > 0.66 && fans != nullptr)
    {
      // Use cv::approxPolyDP or approxPolyCGAL to approximate the contour to further confirm whether it is a fan.
      // Different types{hammer, fan} are handled with different approximation methods.
#define USE_CGAL 0
#if !USE_CGAL
      std::vector<cv::Point> out_approx;
      cv::approxPolyDP(ext_contour, out_approx, 10, true);  // TODO: make epsilon a parameter
      if (out_approx.size() != 6)                           // The fitted polygon has 6 vertices.
        continue;
#else
      std::array<cv::Point, 6> out_approx{};
      approxPolyCGAL(ext_contour, out_approx, out_approx.size());
#endif
      // The contour of the fan is convex, therefore, the approximating result is a convex polygon.
      if (cv::isContourConvex(out_approx))
      {
#if ENABLE_VIRTUALIZATION
        cv::polylines(*img_to_draw_, out_approx, true, cv::Scalar(255, 0, 200), 2);
        for (auto& pt : out_approx)
        {
          cv::circle(*img_to_draw_, pt, 3, cv::Scalar(0, 255, 0), -1);
        }
#endif
        fans->emplace_back(fan_type{ r_rect.center, direction_vector_calculator(), pts[0], pts[1], pts[2],
                                     pts[3] });  // This is a fan, record it.
        ext_contour[0].x = -1;                   // Mark it.
      }
    }
    else
    {
      // The contour seems to belong to the hammer, approximate it to see if it is true.
      // If confirmed, record the contour for applying PCA in the next step.
      // Record the length of the hammer also, for searching the 'R' icon in the following step.
      // hammer.first = the length of hammer(the largest one between r_rect.size.height and r_rect.size.width)

      // Once a hammer was found, do not access this branch! Otherwise hammer_description_ will be modified and destroyed.
      if (hammer_is_found)
        continue;

      std::vector<cv::Point> out_approx(8);
      rm_cgal_tools_polygon_simplification::approxPolyCGAL(ext_contour, out_approx, out_approx.size());
      // Cross product discriminant
      // For a hammer shape, freq will finally = 2.
      // And a concave octagon has at most freq = 4, in this case it is a four-pointed star.
      unsigned int freq = 0, sz = out_approx.size();
      for (unsigned int i = 0; i < sz - 2 && freq < 4; ++i)
      // freq can accumulate up to 3 in the for loop
      // hammer_description_ is now used to record the concave position.
      {
        // The order of points is the right-hand spiral(anti-clockwise).
        ((out_approx[i + 1] - out_approx[i]).cross(out_approx[i + 2] - out_approx[i + 1]) > 0) &&
            (hammer_description_[freq].x = i + 1, ++freq);
      }
      ((out_approx.back() - out_approx[sz - 2]).cross(out_approx[0] - out_approx.back()) > 0) &&
          (hammer_description_[freq].x = sz - 1, ++freq);
      if (freq == 2)
      {
#if ENABLE_VIRTUALIZATION
        cv::polylines(*img_to_draw_, out_approx, true, cv::Scalar(0, 0, 255), 2);
        for (auto& pt : out_approx)
        {
          cv::circle(*img_to_draw_, pt, 3, cv::Scalar(0, 255, 0), -1);
        }
        // Draw concave positions.
        cv::circle(*img_to_draw_, out_approx[static_cast<int>(hammer_description_[0].x)], 3, cv::Scalar(205, 90, 106),
                   -1);
        cv::circle(*img_to_draw_, out_approx[static_cast<int>(hammer_description_[1].x)], 3, cv::Scalar(205, 90, 106),
                   -1);
#endif
        this->concave_position_ = (out_approx[static_cast<int>(hammer_description_[0].x)] +
                                   out_approx[static_cast<int>(hammer_description_[1].x)]) /
                                  2;
        // Index of the point at the end of the hammer handle in out_approx.
        // Calculated by comparing the distance.
        int index;
        for (int r = 0; r < 2; ++r)
        {
          if (hammer_description_[r].x == 0)
          {
            index = (out_approx.back() - out_approx[0]).dot(out_approx.back() - out_approx[0]) >
                            (out_approx[1] - out_approx[0]).dot(out_approx[1] - out_approx[0]) ?
                        sz - 1 /* i.e., back() */ :
                        1;
          }
          else if (hammer_description_[r].x == sz - 1)
          {
            index = (out_approx.back() - out_approx[0]).dot(out_approx.back() - out_approx[0]) >
                            (out_approx.back() - out_approx[sz - 2]).dot(out_approx.back() - out_approx[sz - 2]) ?
                        0 :
                        sz - 2;
          }
          else  // hammer_description_[r].x in range [1, sz - 2]
          {
            int i = static_cast<int>(hammer_description_[r].x);
            index = (out_approx[i] - out_approx[i - 1]).dot(out_approx[i] - out_approx[i - 1]) >
                            (out_approx[i + 1] - out_approx[i]).dot(out_approx[i + 1] - out_approx[i]) ?
                        i - 1 :
                        i + 1;
          }
          // Replace the point at the end of the hammer handle with out_approx[hammer_description_[r].x] which is
          // close to the hammerhead.
          // Thus, the center calculated by averaging or cv::moments() in the next step will be in the hammerhead.
          // Since the original out_approx is still useful, the last two elements of hammer_description_ are used to
          // temporarily store the data to be replaced.
          *(hammer_description_.rbegin() - r) = out_approx[index];
          out_approx[index] = out_approx[static_cast<int>(hammer_description_[r].x)];
          // Store the index.
          hammer_description_[r].x = index;
        }

        auto moment = cv::moments(out_approx, false);
        // !!! moment.m00 == 0 is possible! Ref: https://docs.opencv.org/4.2.0/d8/d23/classcv_1_1Moments.html
        if (moment.m00 == 0)
          continue;

        // Recover the out_approx
        for (int r = 0; r < 2; ++r)
        {
          out_approx[static_cast<int>(hammer_description_[r].x)] = *(hammer_description_.rbegin() - r);
#if ENABLE_VIRTUALIZATION
          cv::circle(*img_to_draw_, out_approx[static_cast<int>(hammer_description_[r].x)], 3, cv::Scalar(255), -1);
#endif
        }

        // Store the "center" of hammerhead.
        hammer_description_[0].x = moment.m10 / moment.m00;
        hammer_description_[0].y = moment.m01 / moment.m00;
#if ENABLE_VIRTUALIZATION
        cv::circle(*img_to_draw_, hammer_description_[0], 4, cv::Scalar(0, 255, 255), -1);
#endif
      }
      else
        continue;
      // TODO: Use one class svm or other ML methods to get more accuracy
      hammer = std::make_pair(r_rect.size.height, &ext_contour);  // Here we find the hammer.
      hammer_is_found = true;
      pcaHammer(hammer, out_approx);
      ext_contour[0].x = -1;  // Mark it.
      if (fans == nullptr)    // If fans == nullptr, just find out the hammer.
        return;
    }
  }
}

bool WindmillRecognizerBase::pcaHammer(hammer_type& hammer, std::vector<cv::Point>& out_approx)
{
  cv::Mat data(hammer.second->size(), 2, CV_64FC1);  // CV_64FC1 is necessary
  auto data_start = reinterpret_cast<double*>(data.data);
  for (auto& i : *hammer.second)
  {
    *(data_start) = i.x;
    ++data_start;
    *(data_start) = i.y;
    ++data_start;
  }
  cv::PCA pca_analysis(data, cv::Mat(), cv::PCA::DATA_AS_ROW);

  // Store the center of the object
  type_2D center{ static_cast<real_type>(pca_analysis.mean.at<double>(0, 0)),
                  static_cast<real_type>(pca_analysis.mean.at<double>(0, 1)) };

  // Note: eigenvalues always > 0
  auto eigenvector_corresponding_to_larger_eigenvalue =
      pca_analysis.eigenvalues.at<double>(1) > pca_analysis.eigenvalues.at<double>(0) ?
          type_2D(static_cast<real_type>(pca_analysis.eigenvectors.at<double>(1, 0)),
                  static_cast<real_type>(pca_analysis.eigenvectors.at<double>(1, 1))) :
          type_2D(static_cast<real_type>(pca_analysis.eigenvectors.at<double>(0, 0)),
                  static_cast<real_type>(pca_analysis.eigenvectors.at<double>(0, 1)));
  auto& eigenvector = eigenvector_corresponding_to_larger_eigenvalue;

  this->r_icon_position_ = { center, eigenvector };

#if ENABLE_VIRTUALIZATION
  cv::line(*img_to_draw_, center, center + eigenvector * 100, cv::Scalar(255, 255, 0), 2);
  cv::line(*img_to_draw_, center - eigenvector * 100, center, cv::Scalar(0, 255, 0), 2);
#endif
  normApprox(center, eigenvector, out_approx);
  getHammerDescription(hammer.first);
  return true;
}

inline void WindmillRecognizerBase::normApprox(type_2D& center, type_2D& eigenvector, std::vector<cv::Point>& out_approx)
{
  // note: |eigenvector| = 1
  // angle = acos(abs(eigenvector.dot(type_2D{1, 0})))
  // that is the minimum angle required to rotate hammer to horizontal
  auto cos_angle = abs(eigenvector.x);

  // consider the rotation orientation
  // x, y same sign angle is positive, different sign angle is negative
  // angle > 0 : anticlockwise rotation ; angle < 0 : clockwise rotation
  //
  // o-------> +x
  // |
  // |
  // v +y
  //
  auto sin_angle = abs(eigenvector.y) * sgn(eigenvector.x) * sgn(eigenvector.y);

  // clang-format off
  cv::Matx<real_type, 2, 2> rotation{ cos_angle,   sin_angle,
                                      sin_angle,  -cos_angle };
  // clang-format on

  // Used to control the horizontal flip of the hammer.
  // Make sure the hammer head is on the right side after rotation.
  real_type horizontal_flip = sgn((rotation * (hammer_description_[0] - center)).x);
  if (horizontal_flip == -1)
  {
    rotation(0, 0) *= horizontal_flip;
    rotation(0, 1) *= horizontal_flip;
  }

  for (auto& item : out_approx)
  {
    item = (rotation * (type_2D(item) - center));
  }

  rotation_inverse_.push_back(cv::Point2f(rotation(0, 0), rotation(0, 1)));
  rotation_inverse_.push_back(cv::Point2f(rotation(1, 0), rotation(1, 1)));

  cv::Rect rect = cv::boundingRect(out_approx);
  float x_ratio = 120.f / rect.width;
  float y_ratio = 50.f / rect.height;
  for (auto& item : out_approx)
  {
    item.x *= x_ratio;
    item.y *= y_ratio;
    item.x -= rect.x * x_ratio;  // rect.x is absolutely negative
    item.y -= rect.y * y_ratio;  // rect.y is absolutely negative
  }

  std::vector<cv::Point> circular_sequence{};
  circular_sequence.reserve(out_approx.size() * 3);
  for (int i = 0; i < 3; ++i)
    for (auto& pt : out_approx)
      circular_sequence.emplace_back(pt);

  // Find the start point(id = 0, see comment below).
  size_t index;
  size_t approx_sz = out_approx.size();
  std::vector<std::pair<size_t, int>> val_table{};
  for (size_t i = 0; i < approx_sz; ++i)
  {
    auto& pt = out_approx[i];
    val_table.emplace_back(std::pair<size_t, int>(i, pt.x * pt.x + pt.y * pt.y));
  }
  std::sort(val_table.begin(), val_table.end(),
            [](std::pair<size_t, int>& lhs, std::pair<size_t, int>& rhs) { return lhs.second < rhs.second; });
  // Find the two points closest to (0, 0) and choose the point with smaller y.
  if (val_table[0].second * 2 < val_table[1].second)  // handle the case where out_approx has a thorn
    index = val_table[0].first;  // the point closest to (0, 0) is the only solution
  else
    index =
        out_approx[val_table[0].first].y < out_approx[val_table[1].first].y ? val_table[0].first : val_table[1].first;
  // Now sort out_approx as shown below.
  //            2 *-------* 3
  //              |       |
  // 0            |       |
  // *------------*       |
  //              1
  index += approx_sz;
  if ((circular_sequence[index + 1].x - circular_sequence[index].x) >
      (circular_sequence[index].x - circular_sequence[index - 1].x) &&
      (circular_sequence[index + 2].y - circular_sequence[index + 1].y) <
          (circular_sequence[index - 2].y - circular_sequence[index - 1].y))
    out_approx.assign(circular_sequence.begin() + index, circular_sequence.begin() + index + approx_sz);
  else
  {
    for (size_t i = 0; i < approx_sz; ++i)
      out_approx[i] = circular_sequence[index - i];
  }
  // If out_approx has a thorn, rectify it:
  //   0                     1                                            0                     1
  //   *                     * <--- concave position                      *                     * <--- concave position
  //                                                        ====>>
  //                         * <--- concave position                      *                     * <--- concave position
  //                         ^                                            ^                     ^
  //                         .back()                                      .back()               .rbegin() + 1
  //
  cv::Point p1 = out_approx.back() - out_approx[1], p2 = out_approx.back() - out_approx[0];
  if ((p1.x * p1.x + p1.y * p1.y) < (p2.x * p2.x + p2.y * p2.y))
  {
    out_approx.emplace_back(cv::Point(out_approx[0].x, out_approx.back().y >= out_approx[0].y ?
                                                       out_approx.back().y :
                                                       out_approx[0].y + out_approx.back().y - out_approx[1].y));
    double cmp_val = 0.;  // cos(90deg)
    size_t id = 0;
    for (size_t i = 2; i < out_approx.size() - 2; ++i)
    {
      cv::Point v1 = out_approx[i + 1] - out_approx[i], v2 = out_approx[i] - out_approx[i - 1];
      double val = abs(v1.ddot(v2)) / (cv::norm(v1) * cv::norm(v2));  // cos of the angle
      if (val > cmp_val)
        cmp_val = val, id = i;
    }
    out_approx.erase(out_approx.begin() + id);
  }
#if ENABLE_VIRTUALIZATION
  auto approx_copy = out_approx;
  for (auto& item : approx_copy)
  {
    item.x += 40;
    item.y += 20;
  }
  cv::polylines(*img_to_draw_, approx_copy, true, cv::Scalar(0, 255, 255), 2);
  // Show if it's sorted correctly. (the following 3 lines)
  cv::circle(*img_to_draw_, approx_copy[0], 4, cv::Scalar(255, 0, 0), -1);
  cv::circle(*img_to_draw_, approx_copy[1], 4, cv::Scalar(0, 255, 0), -1);
  cv::circle(*img_to_draw_, approx_copy[2], 4, cv::Scalar(0, 0, 255), -1);
  for (size_t i = 3; i < approx_copy.size(); ++i)
    cv::circle(*img_to_draw_, approx_copy[i], 4, cv::Scalar(238, 122, 233), -1);
#endif
}

inline void WindmillRecognizerBase::getHammerDescription(int length)
{
  auto center = this->r_icon_position_.front();
  auto direction_vec = this->r_icon_position_.back() * length / 2.;
  decltype(center) normal_vec{ direction_vec.y, -direction_vec.x };  // normal vector
  auto pt1 = center + direction_vec + normal_vec / 2.;
  auto pt4 = center - direction_vec + normal_vec / 2.;
  auto pt2 = pt1 - normal_vec;
  auto pt3 = pt4 - normal_vec;
  // Easy to prove: pt1, pt2, pt3, pt4 are in clockwise order.
  // Now let the four points and the hammer be arranged as follows.
  // 4                      1
  //               [[[[[[]
  //  [[[[[[[[[[[[[[[ * ]]      Remember: hammer_description_[0] is the position of *
  //               [[[[[[]
  // 3                      2
  if ((pt1 - hammer_description_[0]).dot(pt1 - hammer_description_[0]) >
      (pt3 - hammer_description_[0]).dot(pt3 - hammer_description_[0]))
  {
    std::swap(pt1, pt3);
    std::swap(pt2, pt4);
  }
  hammer_description_[0] = pt1;
  hammer_description_[1] = pt2;
  hammer_description_[2] = pt3;
  hammer_description_[3] = pt4;
#if ENABLE_VIRTUALIZATION
  cv::circle(*img_to_draw_, hammer_description_[0], 3, cv::Scalar(255, 0, 255), -1);
  cv::circle(*img_to_draw_, hammer_description_[1], 3, cv::Scalar(0, 0, 255), -1);
  cv::circle(*img_to_draw_, hammer_description_[2], 3, cv::Scalar(0, 255, 255), -1);
  cv::circle(*img_to_draw_, hammer_description_[3], 3, cv::Scalar(0, 255, 0), -1);
#endif
}

void WindmillRecognizerBase::locateCenterIconR(external_contours_type& ext_contours, hammer_type& hammer,
                                               fans_type& fans)
{
  bool is_found = false;
  type_2D r_center_1(-1, -1), r_center_2(-1, -1);
  for (auto& item : ext_contours)
  {
    // Previously, any contour which is possible to be a fan or hammer has already had its first element marked.
    // Just skip it.
    if (item[0].x == -1)
      continue;
    auto bounding_rect = cv::boundingRect(item);
    // The center of 'R' icon (only possible).
    // tl: top left; br: bottom right
    type_2D r_center = (bounding_rect.tl() + bounding_rect.br()) / 2;
    //
    //               * R('R' icon)
    //             ,* O
    //            / /
    //           / /
    //      /`-./*/C(center)
    //     /      `-_
    //     `-.      /
    //           `-`
    //
    //        -->
    // vec1 = RC  = R - C
    // distance = |RC|
    auto vec1 = r_center - this->r_icon_position_[0];
    // Angle between line RC and OC.
    // ---------------------------------------
    // The distance from point R to line OC does in some way indicate whether R is close to line OC,
    // but it is not scale-invariant.
    // The angle is scale-invariant, translation-invariant, and rotation-invariant,
    // which optimizes the defect of using distance judgment.
    // Remember: |r_icon_position_[1]| = 1.
    double theta = abs(asin(vec1.cross(r_icon_position_[1]) / norm(vec1)));
    // distance = |RC|
    // It is used to limit the position of R.
    double distance = sqrt(pow(r_center.x - r_icon_position_[0].x, 2) + pow(r_center.y - r_icon_position_[0].y, 2));
    if (theta < 4 * CV_PI / 180           // theta / CV_PI * 180 < 4 degrees
        && distance > 0.5 * hammer.first  // distance is at least half the length of the hammer
        && distance < hammer.first        // distance is at most the length of the hammer
    )
    {
      r_center_1 = r_center;
      is_found = true;
      break;
    }
  }

  // Fans is not empty, the intersection of the central axis of the fan and the hammer
  // can further confirm the position of the 'R' icon.
  if (!fans.empty())
  {
    size_t i = 0;
    for (; i < fans.size(); ++i)
    {
      type_2D vec = std::get<1>(fans[i]);
      double val1 = vec.cross(r_icon_position_[1]);
      double val2 = abs(val1) / cv::norm(vec);
      constexpr double rad_66_deg = 66. / 180. * CV_PI;
      constexpr double rad_78_deg = 78. / 180. * CV_PI;
      constexpr double rad_30_deg = 30. / 180. * CV_PI;
      constexpr double rad_42_deg = 42. / 180. * CV_PI;
      if ((val2 < sin(rad_78_deg) && val2 > sin(rad_66_deg)) || (val2 < sin(rad_42_deg) && val2 > sin(rad_30_deg)))
      {
        // Calculate the intersection.
        // TODO: comments of math!
        double t1 = std::get<1>(fans[i]).cross(r_icon_position_[0] - std::get<0>(fans[i])) / val1;
        r_center_2 = r_icon_position_[0] - t1 * r_icon_position_[1];
#if ENABLE_VIRTUALIZATION
        cv::line(*img_to_draw_, std::get<0>(fans[i]), std::get<0>(fans[i]) + std::get<1>(fans[i]), cv::Scalar(0, 255),
                 2);
#endif
        break;
      }
      else
        continue;
    }
  }

  if (is_found)
  {
    if ((r_center_2.x > 0 && r_center_2.y > 0 &&
         (pow(r_center_2.x - r_center_1.x, 2) + pow(r_center_2.y - r_center_1.y, 2)) < 100.) ||
        fans.empty())
    {
      // Checked.
      // Tell the analyzer that r_icon_position is really trustable.
      r_icon_position_[1].x = static_cast<real_type>(RIconPositionConfidence::REALLY_TRUSTABLE);
    }
    else
    {
      r_icon_position_[1].x = static_cast<real_type>(RIconPositionConfidence::NOT_REALLY_TRUSTABLE);
    }
    r_icon_position_[0] = r_center_1;
  }
  else
  {
    r_icon_position_[1].x = static_cast<real_type>(RIconPositionConfidence::NOT_TRUSTABLE);
    return;
  }

#if ENABLE_VIRTUALIZATION
  cv::circle(*img_to_draw_, r_icon_position_[0], 5, cv::Scalar(0, 255, 255), -1);
  cv::putText(*img_to_draw_,
              std::to_string(static_cast<int>(floor(r_icon_position_[0].x))) + ", " +
                  std::to_string(static_cast<int>(floor(r_icon_position_[0].y))) +
                  ((RIconPositionConfidence)(r_icon_position_[1].x) == RIconPositionConfidence::REALLY_TRUSTABLE ?
                       " trustable" :
                       ""),
              cv::Point(200, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(160, 20, 255), 2);
#endif
}

bool WindmillRecognizerBase::locateNotTriggeredFan(cv::Mat1b& sel)
{
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

  rotation_inverse_.clear();
  hammer_type hammer{};
  fans_type fans{};
  findHammerAndFans(ext_contours, hammer, &fans);
  if (hammer.first == 0)
    return false;
  locateCenterIconR(ext_contours, hammer, fans);
  fans_ = std::move(fans);
  rotation_inverse_.push_back(cv::Point2f(sel.rows / 2, sel.cols / 2));
  rot_inverse_ = (cv::Mat1d(2, 3, 0.) << rotation_inverse_[0].x, rotation_inverse_[0].y, rotation_inverse_[2].x,
      rotation_inverse_[1].x, rotation_inverse_[1].y, rotation_inverse_[2].y);

  extractHammerRoi(hammer.first);
  return true;
}

}  // namespace rm_windmill_vision