//
// Created by kaxns on 3/6/22.
//

#include "windmill_dev/windmill_analyzer.h"

namespace rm_windmill_vision
{
void WindmillAnalyzerBase::modelWindmill(cv::Mat1d& points_3d, int amount)
{
  // The model coordinate is initially defined as bellow:
  //
  //     *------> +x         1   2
  //     |       origin      |```|
  //     |        o==========|   |  not triggered
  //     v +y     *\*        |___|
  //               *\*       4   3
  //                *\*    1               /
  //      triggered  *\*_-`\             _'
  //               4.-`    .* 2        _`
  //                 `.,'`          _``
  //                  3       <<-'`
  //                        + \theta
  //
  using type_2D = cv::Point2d;
  // Get the original direction vector.
  // [ r00 r01 ]   [ 1 ]   _         [r00]
  // [         ] * [   ] = R * n_x = [   ]
  // [ r10 r11 ]   [ 0 ]             [r10]
  // _
  // R : Rotation part of inverse_transform
  // !!! NOTE:
  // The original direction vector must have the same direction as (1, 0) after affine transformation.
  // This can easily be done in WindmillRecognizer with the help of the position of the "R" icon.
  auto get_original_dir_vec = [this](size_t id) {
    return type_2D(inverse_trans_[id].at<double>(0, 0), inverse_trans_[id].at<double>(1, 0));
  };
  auto ref_vec = get_original_dir_vec(0);
  auto ref_vec_len = cv::norm(ref_vec);
#define epsilon (10)  // angle error limit
  auto calc_angle = [&ref_vec_len, &ref_vec](const type_2D& vec) {
    // around 72deg or (180 - 2 * 72)deg or -72deg or -(180 - 2 * 72)deg
    // angle > 0 => \theta > 0; angle < 0 => \theta < 0
    double angle = asin(vec.cross(ref_vec) / (cv::norm(vec) * ref_vec_len)) * 360 / CV_2PI;
    int sgn = angle > 0 ? 1 : (angle = -angle, -1);
    if (angle > 72 - epsilon && angle < 72 + epsilon)  // => \theta = \pm 72deg
      return 72 * sgn;
    else if (angle > (180 - 2 * 72) - epsilon && angle < (180 - 2 * 72) + epsilon)  // => \theta = \pm 72 * 2deg
      return 72 * 2 * sgn;
    else
      return -1;  // This is an error condition, and when it occurs, the corresponding ROI modeling must be disabled.
  };

  static int angle_array[4];
  if (amount > 1)
  {
    auto amt = amount;
    for (int i = 1 /* start from 1 to skip the hammer */, j = 0; i < amt; ++i)
    {
      auto val = calc_angle(get_original_dir_vec(i));
      if (val != -1)
      {
        angle_array[j] = val;
        ++j;
      }
      else
      {
        actual_roi_size_[i].width = 0;  // This one should be deprecated, use actual_roi_size_ to mark it.
        --amount;                       // Make sure that the following points_3dim takes right number of 3D points.
      }
    }
  }

  // Each valid ROI provides 4 points.
  points_3d.create(amount * 4, 3);
  for (int i = 0; i < 4; ++i)
  {
    /* x */ points_3d(i, 0) = this->angle_LUT_.find(0)->second[i].x;
    /* y */ points_3d(i, 1) = this->angle_LUT_.find(0)->second[i].y;
    /* z */ points_3d(i, 2) = 0.;
  }
  cv::Mat1d hammer_3d(4, 3, 0.);
  points_3d(cv::Rect(0, 0, 3, 4)).copyTo(hammer_3d);
  hammer_3d_ = hammer_3d;

  for (int i = 0, start = 4; i < amount - 1; ++i, start += 4)
  {
    for (int j = 0; j < 4; ++j)
    {
      /* x */ points_3d(start + j, 0) = this->angle_LUT_.find(angle_array[i])->second[j].x;
      /* y */ points_3d(start + j, 1) = this->angle_LUT_.find(angle_array[i])->second[j].y;
      /* z */ points_3d(start + j, 2) = 0.;
    }
  }
}

void WindmillAnalyzerBase::constructLUT(double big_armor_width, double big_armor_height, double distance)
{
  // The model coordinate is initially defined as bellow:
  //
  //     *------> +x         1   2
  //     |       origin      |```|
  //     |        o==========|   |  not triggered
  //     v +y     *\*        |___|
  //               *\*       4   3
  //                *\*    1               /
  //      triggered  *\*_-`\             _'
  //               4.-`    .* 2        _`
  //                 `.,'`          _``
  //                  3       <<-'`
  //                        + \theta
  //

  auto rotation = [](double angle) {
    double cos_a = cos(angle);
    double sin_a = sin(angle);
    // clang-format off
    return cv::Matx22d{ cos_a, -sin_a,
                        sin_a,  cos_a };
    // clang-format on
  };
  const std::array<cv::Point2d, 4> four_vertices{
      /* 1 */ cv::Point2d(distance + big_armor_height / 2., +big_armor_width / 2.),
      /* 2 */ cv::Point2d(distance - big_armor_height / 2., +big_armor_width / 2.),
      /* 3 */ cv::Point2d(distance - big_armor_height / 2., -big_armor_width / 2.),
      /* 4 */ cv::Point2d(distance + big_armor_height / 2., -big_armor_width / 2.)
  };
  this->angle_LUT_.clear();
  this->angle_LUT_[0] = four_vertices;
  auto rot = rotation(72);
  this->angle_LUT_[72] = { rot * four_vertices[0], rot * four_vertices[1], rot * four_vertices[2],
                           rot * four_vertices[3] };
  rot = rotation(72 * 2);
  this->angle_LUT_[72 * 2] = { rot * four_vertices[0], rot * four_vertices[1], rot * four_vertices[2],
                               rot * four_vertices[3] };
  rot = rotation(-72);
  this->angle_LUT_[-72] = { rot * four_vertices[0], rot * four_vertices[1], rot * four_vertices[2],
                            rot * four_vertices[3] };
  rot = rotation(-72 * 2);
  this->angle_LUT_[-72 * 2] = { rot * four_vertices[0], rot * four_vertices[1], rot * four_vertices[2],
                                rot * four_vertices[3] };
}

__attribute__((unused)) void WindmillAnalyzerBase::cvtColorZZJ(cv::Mat3b& src)
{
  std::vector<cv::Mat1b> zzj(3);
  cv::split(src, zzj);
  cv::Mat1b blue = zzj[0] - zzj[2];
  cv::threshold(blue, blue, 0, 255, cv::THRESH_BINARY);
  cv::Mat3b hsv;
  cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(hsv, cv::Scalar(0, 0, 150), cv::Scalar(255, 255, 255), zzj[1]);
  cv::bitwise_and(zzj[1], blue, blue);
}

template <const int Reserved, const int Subtracted>
void WindmillAnalyzerBase::cvtColor(cv::Mat3b& src, cv::Mat1b& bin, int thresh)
{
  static_assert(Reserved >= 0 && Reserved <= 2, "");
  static_assert(Subtracted >= 0 && Subtracted <= 2, "");
  static_assert(Reserved != Subtracted, "");
  std::vector<cv::Mat1b> bgr(3);
  cv::split(src, bgr);
  cv::Mat1b diff = bgr[Reserved] - bgr[Subtracted];
  cv::medianBlur(diff, diff, 3);
  cv::threshold(diff, bin, thresh, 1, cv::THRESH_BINARY);
}

void WindmillAnalyzerBase::setColorMethod(const std::string& color)
{
  if (color == "red")
    convert_color_ = cvtColor<2, 1>;
  else if (color == "blue")
    convert_color_ = cvtColor<0, 2>;
  else
    ROS_ASSERT_MSG(false, "Invalid windmill color mode!");
}

int WindmillAnalyzerBase::getHammerFourVertices(double* vertices)
{
  static image_transport::Publisher head_pub = it_->advertise("head_roi", 1);
  cv::Mat1d r20c2;
  r20c2.create(roi_amount_ * 4, 2);
  //Used fo store five armor plates' vertices.
  //The first four rows of the matrix store vertices on hammer.
  for(int k = 0; k < roi_amount_; ++k)
  {
#if ENABLE_VIRTUALIZATION
    // @name da cha (Chinese PinYing)
    // @brief Marks the result is incorrect.
    // ___________
    // |*       *|
    // | `*   *` |
    // |   `*`   |
    // |  *` `*  |
    // |*`     `*|
    // ```````````
    auto da_cha = [](cv::Mat3b& img) {
      cv::line(img, cv::Point(0, 0), cv::Point(img.cols - 1, img.rows - 1), cv::Scalar(0, 0, 255), 2);
      cv::line(img, cv::Point(0, img.rows - 1), cv::Point(img.cols - 1, 0), cv::Scalar(0, 0, 255), 2);
    };
#endif

    cv::Mat3b head =
        roi_array_[k](cv::Rect(concave_position_[k].x, 0, roi_array_[k].cols - concave_position_[k].x, roi_array_[k].rows));
    cv::Mat1b binary, bin_copy;
    (*convert_color_)(head, binary, threshold_);
    bin_copy = binary.clone();
    // for Morphological gradient
    static const auto KERNEL = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat1b eroded;
    cv::erode(binary, eroded, KERNEL);
    cv::bitwise_xor(binary, eroded, binary);  // !!! FORBIDDEN: binary -= eroded;  // slower than bit operation

#if ENABLE_VIRTUALIZATION
    cv::Mat3b debug(head.rows * 1, head.cols * 2);
    head.copyTo(debug(cv::Rect(0, 0, head.cols * 1, head.rows * 1)));
    cv::cvtColor(binary * 255, head, cv::COLOR_GRAY2BGR);
#endif

    cv::Mat_<int> proj;  // use projection method
    cv::reduce(binary, proj, 0 /* reduced to a single row */, cv::REDUCE_SUM, CV_32S);

#if ENABLE_VIRTUALIZATION
    // Draw the reduced result.
    for (int i = 0; i < proj.cols; ++i)
      if (proj(0, i))
        cv::line(head, cv::Point(i, head.rows - 1 - proj(0, i)), cv::Point(i, head.rows - 1), cv::Scalar(0, 255, 0));
#endif

    cv::Scalar mean;
    cv::Scalar stddev;
    cv::Mat1b proj_mask;

    cv::meanStdDev(proj, mean, stddev, proj != 0 /* only nonzero element counts */);
    double coefficient_of_variation = stddev[0] / mean[0];

#if ENABLE_VIRTUALIZATION
    ROS_INFO("mean: %f; stddev: %f, cv: %f\n", mean[0], stddev[0], coefficient_of_variation);
    // Draw the average line.
    cv::line(head, cv::Point(0, head.rows - 1 - mean[0]), cv::Point(head.cols - 1, head.rows - 1 - mean[0]),
             cv::Scalar(0, 255, 255));
#endif

    if (!(coefficient_of_variation > coefficient_of_variation_typ_ || stddev[0] > stddev_typ_))
#if !ENABLE_VIRTUALIZATION
      return FAILED;
#else
    {
      da_cha(head);
      head.copyTo(debug(cv::Rect(head.cols * 1, 0, head.cols * 1, head.rows * 1)));
      head_pub.publish(cv_bridge::CvImage(sync_, "bgr8", debug).toImageMsg());
      index_ = k;
      return FAILED;
    }
#endif

    int count = 0;
    int thresh = static_cast<int>(mean[0]);
    std::map<int /* index */, int /* projection value */> peak_map{};
    for (int i = 0; i < proj.cols; ++i)
    {
      if (proj(0, i) > thresh)
        ++count;
      else
      {
        if (count == 0)
          continue;
        else if (count == 1)
          peak_map[i - 1] = proj(0, i - 1);
        else
        {
          int max = 0;
          int index = 0;
          // find the max and its index
          for (int j = 1; j <= count; ++j)
            if (proj(0, i - j) > max)
              max = ((index = i - j), proj(0, i - j));
          // check the magnitude
          for (int j = 1; j <= count; ++j)
            if (proj(0, i - j) < static_cast<int>(max * 0.8))
            {
              peak_map[index] = max;
              goto end;  // break and reset
            }
          peak_map[i - count / 2] = max;
        }
        end:
        count = 0;
      }
    }

    std::vector<std::pair<int, int>> peak_vec(peak_map.begin(), peak_map.end());
    if (peak_vec.size() == 4)  // that's ok
      ;
    else if (peak_vec.size() > 4)  // select the largest 4 elements
    {
      std::sort(peak_vec.begin(), peak_vec.end(),
                [](const std::pair<int, int>& lhs, const std::pair<int, int>& rhs) -> bool {  // compare by value
                  return lhs.second > rhs.second;
                });
#if ENABLE_VIRTUALIZATION
      while (peak_vec.size() > 4)  // remove the excess
        peak_vec.pop_back();
#endif
      std::sort(peak_vec.begin(), peak_vec.begin() + 4,
                [](const std::pair<int, int>& lhs, const std::pair<int, int>& rhs) -> bool {  // compare by key
                  return lhs.first < rhs.first;
                });
    }
    else  // peak_vec.size() < 4: fuck it off
    {
#if !ENABLE_VIRTUALIZATION
      return FAILED;
#else
      da_cha(head);
      for (auto& item : peak_vec)
        cv::line(head, cv::Point(item.first, 0), cv::Point(item.first, 4), cv::Scalar(255, 0, 255), 1);
      head.copyTo(debug(cv::Rect(head.cols * 1, 0, head.cols * 1, head.rows * 1)));
      head_pub.publish(cv_bridge::CvImage(sync_, "bgr8", debug).toImageMsg());
      index_ = k;
      return FAILED;
#endif
    }

    if ((peak_vec[2].first - peak_vec[1].first) <
        (peak_vec[1].first - peak_vec[0].first + peak_vec[3].first - peak_vec[2].first))  // thickness is too large
    {
#if !ENABLE_VIRTUALIZATION
      return FAILED;
#else
      da_cha(head);
      for (auto& item : peak_vec)
        cv::line(head, cv::Point(item.first, 0), cv::Point(item.first, 4), cv::Scalar(0, 255, 255), 1);
      head.copyTo(debug(cv::Rect(head.cols * 1, 0, head.cols * 1, head.rows * 1)));
      head_pub.publish(cv_bridge::CvImage(sync_, "bgr8", debug).toImageMsg());
      index_ = k;
      return FAILED;
#endif
    }

    int adjust_horizontal = 1;  // TODO : adjustable
    int x_tangent_1 = peak_vec[1].first + adjust_horizontal;
    int x_tangent_2 = peak_vec[2].first - adjust_horizontal;

#if ENABLE_VIRTUALIZATION
    // Mark the peaks with short lines.
    for (auto& item : peak_vec)
      cv::line(head, cv::Point(item.first, 0), cv::Point(item.first, 4), cv::Scalar(0, 0, 255), 1);
    // Draw two inner tangents:
    //         t1|     |t2
    //           |     |
    //         []|[[[]]|[]
    //         []|     |[]
    // [[[[[[[[[]|     |[]
    //         []|     |[]
    //         []|[[[]]|[]
    //           |     |
    cv::line(debug, cv::Point(x_tangent_1, 0), cv::Point(x_tangent_1, debug.rows - 1), cv::Scalar(0, 0, 255), 1);
    cv::line(debug, cv::Point(x_tangent_2, 0), cv::Point(x_tangent_2, debug.rows - 1), cv::Scalar(0, 0, 255), 1);
#endif

    cv::Mat1b horizontal_margin_removed =
        bin_copy(cv::Rect(peak_vec[1].first, 0, peak_vec[2].first - peak_vec[1].first, binary.rows));
    cv::reduce(horizontal_margin_removed, proj, 1 /* reduced to a single col */, cv::REDUCE_SUM, CV_32S);
    thresh = cv::mean(proj, proj != 0)[0];
#if ENABLE_VIRTUALIZATION
    // Draw the reduced result.
    for (int i = 0; i < proj.rows; ++i)
      if (proj(i, 0))
        cv::line(head, cv::Point(head.cols - 1 - proj(i, 0), i), cv::Point(head.cols - 1, i), cv::Scalar(150, 200, 0));
    // Draw the threshold.
    cv::line(head, cv::Point(head.cols - 1 - thresh, 0), cv::Point(head.cols - 1 - thresh, head.rows - 1),
             cv::Scalar(0, 255, 255));
#endif

    int y_tangent_1{}, y_tangent_2{};
    for (int i = concave_position_[k].y; i > 0; --i)
      if (proj(i, 0) > thresh)
      {
        y_tangent_1 = i;
        break;
      }
    for (int i = concave_position_[k].y; i < proj.rows; ++i)
      if (proj(i, 0) > thresh)
      {
        y_tangent_2 = i;
        break;
      }

    int adjust_vertical_1 = 1;  // TODO : adjustable
    int adjust_vertical_2 = 2;  // TODO : naive method must be replaced.
    y_tangent_1 -= adjust_vertical_1;
    y_tangent_2 += adjust_vertical_2;

#if ENABLE_VIRTUALIZATION
    // Draw two inner tangents:
    //         [][[[[[]]]][]
    //     ------------------- t3
    //         []         []
    // [[[[[[[[[]         []
    //         []         []
    //     ------------------- t4
    //         [][[[[[]]]][]
    cv::line(debug, cv::Point(0, y_tangent_1), cv::Point(debug.cols - 1, y_tangent_1), cv::Scalar(0, 0, 255));
    cv::line(debug, cv::Point(0, y_tangent_2), cv::Point(debug.cols - 1, y_tangent_2), cv::Scalar(0, 0, 255));
#endif

    // clang-format off
    x_tangent_1 += concave_position_[k].x;  // offset the bias
    x_tangent_2 += concave_position_[k].x;  // offset the bias
    // 4x2, 2D positions of vertices in the original image
    cv::Mat r4c2 = (cv::Mat_<double>(4, 3) << x_tangent_2, y_tangent_1, 1,
        x_tangent_1, y_tangent_1, 1,
        x_tangent_1, y_tangent_2, 1,
        x_tangent_2, y_tangent_2, 1) * (inverse_trans_[k].t());
    // clang-format on
    r4c2.copyTo(r20c2(cv::Rect(0, k * 4, r4c2.cols, r4c2.rows)));
    if(k==0)
      r4c2.copyTo(hammer_2d_);

#if ENABLE_VIRTUALIZATION
    head.copyTo(debug(cv::Rect(head.cols * 1, 0, head.cols * 1, head.rows * 1)));
    head_pub.publish(cv_bridge::CvImage(sync_, "bgr8", debug).toImageMsg());
#endif
  }
  index_ = roi_amount_;
  memcpy(vertices, r20c2.data, roi_amount_ * 4 * 2 * sizeof(double));
  return SUCCESS;
}

int WindmillAnalyzerBase::getWindmillVertices(cv::Mat1d& points_2d)
{
  points_2d.create(roi_amount_ * 4, 2);
  if(getHammerFourVertices(reinterpret_cast<double*>(points_2d.data)) != SUCCESS &&
     flags_.flags_.has(scope::TREAT_HAMMER_ONLY))
    return FAILED;
  // TODO : treat fans
  else
    return SUCCESS;
}

void WindmillAnalyzerBase::sendPoints(cv::Mat1d &points_2d)
{
    cv::Mat1d points_2d_cut = points_2d(cv::Rect(0, 0, 2, 4));
  rm_msgs::TargetDetection one_target;
  one_target.id = 1;

  int32_t temp[8]{};
  for(int i = 0; i < 4; ++i)
  {
    temp[2 * i] = static_cast<int32_t>(points_2d_cut.at<double>(i, 0));
    temp[2 * i + 1] = static_cast<int32_t>(points_2d_cut.at<double>(i, 1));
  }
  memcpy(&one_target.pose.orientation.x, &temp[2], sizeof(int32_t) * 2);
  memcpy(&one_target.pose.orientation.y, &temp[0], sizeof(int32_t) * 2);
  memcpy(&one_target.pose.orientation.z, &temp[6], sizeof(int32_t) * 2);
  memcpy(&one_target.pose.orientation.w, &temp[4], sizeof(int32_t) * 2);
  target_.detections.emplace_back(one_target);
//  if (target_.detections.empty())
//    one_target.id = 0;
  target_.header.stamp = this->sync_.stamp;
  target_.header.seq = this->sync_.seq;
  target_pub_.publish(target_);
}

void WindmillAnalyzerBase::solvePose(cv::Mat1d& points_3d, cv::Mat1d& points_2d)
{
  for(int i = 0; i < index_; i++)
  {
    cv::Mat1d points_3d_cut = points_3d(cv::Rect(0, i * 4, 3, 4));
    cv::Mat1d points_2d_cut = points_2d(cv::Rect(0, i * 4, 2, 4));
    cv::Mat_<double> r_vec(3, 1), t_vec(3, 1);
    cv::solvePnP(points_3d_cut, points_2d_cut, cam_intrinsic_, dist_coefficients_, r_vec, t_vec, false, cv::SOLVEPNP_AP3P);
    if (r_icon_confidence_ == RIconPositionConfidence::REALLY_TRUSTABLE)  // Use R icon to check and rectify the result.
    {
      cv::Mat img_point(1, 1, CV_64FC2);
      cv::projectPoints(cv::Mat1d(3, 1, 0.), r_vec, t_vec, cam_intrinsic_, dist_coefficients_, img_point);
      double error = pow(*(double*)img_point.data - r_icon_position_.x, 2) +
          pow(*((double*)img_point.data + 1) - r_icon_position_.y, 2);
#if ENABLE_VIRTUALIZATION
      ROS_INFO("error_before: %f", error);
#endif
      if (error < 4.)  // tolerable error: radius < 2
        goto end;
      points_3d_cut.push_back(cv::Mat1d(1, 3, 0.));
      points_2d_cut.push_back([this]() -> cv::Mat1d {
        cv::Mat1d temp(1, 2);
        *(double*)temp.data = r_icon_position_.x;
        *((double*)temp.data + 1) = r_icon_position_.y;
        return temp;
      }());
      cv::solvePnP(points_3d_cut, points_2d_cut, cam_intrinsic_, dist_coefficients_, r_vec, t_vec, true, cv::SOLVEPNP_ITERATIVE);
      cv::projectPoints(cv::Mat1d(3, 1, 0.), r_vec, t_vec, cam_intrinsic_, dist_coefficients_, img_point);
#if ENABLE_VIRTUALIZATION
      error = pow(*(double*)img_point.data - r_icon_position_.x, 2) +
          pow(*((double*)img_point.data + 1) - r_icon_position_.y, 2);
      ROS_INFO("error_after: %f", error);
#endif
      end:;
    }

    cv::Mat1d rotation;
    cv::Rodrigues(r_vec, rotation);
    //                         1     2
    //    .------- frame1       |`````|
    //    |          o==========|  o <|--- frame2(origin is armor's center)
    //    |          ^          |_____|       |
    // T1 |          |          4     3       |
    //    |          '.______________________.'
    //    |                     T2
    //    v
    //    o
    //  camera_frame
    //
    //  frame2 ==> camera_frame = T1 * T2
    //  camera_frame ==> frame2 = (T1 * T2).inv()
    //
    //      ___         ___
    //      |         | t1|
    //  T = |  R_3x3  | t2|
    //      |_________| t3|
    //      | 0  0  0   1 |
    //      ```         ```
    // clang-format off
    static cv::Mat1d transform2 = (cv::Mat1d(4, 4) << 1, 0, 0, distance_,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1);
    // clang-format on
    cv::Mat1d transform1(4, 4, 0.);
    rotation.copyTo(transform1(cv::Rect(0, 0, 3, 3)));
    memcpy(transform1.data + 3 * transform1.step, t_vec.data, 3 * sizeof(double));
    transform1 = transform1.t();
    transform1(3, 3) = 1.;
    // The transform is transposed to get the rotation from camera_frame to frame2.
    // After transposing, the translation vector changes from one column to one row, which can be easily recorded with memcpy.
    // The translation vector is exactly the coordinate of armor center under camera frame.
    cv::Mat1d transform = (transform1 * transform2).t();
    rotation = transform(cv::Rect(0, 0, 3, 3));
    memcpy(t_vec.data, transform.data + transform.step * 3, 3 * sizeof(double));

    cv::Rodrigues(rotation, r_vec);
#if CV_MAJOR_VERSION == 4
    trans_vec_ = t_vec.reshape(1, 1);
#elif CV_MAJOR_VERSION == 3
    trans_vec_ = { t_vec(0, 0), t_vec(1, 0), t_vec(2, 0) };
#endif
    getQuaternion(r_vec);

    rm_msgs::TargetDetection one_target;
    one_target.id = 9;  // TODO : id for windmill = ?
    one_target.pose.position.x = trans_vec_[0];
    one_target.pose.position.y = trans_vec_[1];
    one_target.pose.position.z = trans_vec_[2];
    auto& quaternion = one_target.pose.orientation;
    quaternion.x = quat_vec_[0];
    quaternion.y = quat_vec_[1];
    quaternion.z = quat_vec_[2];
    quaternion.w = quat_vec_[3];
    position_z_.emplace_back(trans_vec_[2]);
    if(i == 0)
      target_.detections.emplace_back(one_target);
  }
  if(flags_.flags_.has(scope::TREAT_ALL))
  {
    for(auto position_z : position_z_)
    {
      if(position_z < 6.5 || position_z > 7.5)
      {
        flags_.flags_.analysisError();
        mark_ = FAILED;
        return;
      }
    }
  }
}

void WindmillAnalyzerBase::getQuaternion(cv::Mat_<double>& r_vec)
{
  double phi = sqrt(r_vec.dot(r_vec));
  double w = 1, x = 0, y = 0, z = 0;
  if (phi > 1e-6)
  {
    w = cos(phi / 2.);
    r_vec *= (sin(phi / 2.) / phi);  // (r_vec /= phi) *= sin(phi / 2.)
    x = r_vec(0, 0);
    y = r_vec(1, 0);
    z = r_vec(2, 0);
  }
  this->quat_vec_ = { x, y, z, w };
}

void WindmillAnalyzerBase::send2TF()
{
  this->broadcaster_.sendTransform(tf::StampedTransform(
      tf::Transform(tf::Quaternion(quat_vec_[0], quat_vec_[1], quat_vec_[2], quat_vec_[3]),
                    tf::Vector3(trans_vec_[0], trans_vec_[1], trans_vec_[2])),
      sync_.stamp, sync_.frame_id.empty() ? "camera_optical_frame" : sync_.frame_id, "windmill_target"));
}

void WindmillAnalyzerBase::send2Ctrl()
{
  if (target_.detections.empty())
    return;
  target_.header.stamp = this->sync_.stamp;
  target_.header.seq = this->sync_.seq;
  target_pub_.publish(target_);
}

void WindmillAnalyzerBase::rotationPrediction()
{
  rm_msgs::TargetDetection one_target;
  one_target.id = 10;

  cv::Mat inverse_rotation = (cv::Mat1d(2, 2, 0.) << rotation_inverse_(0, 0), rotation_inverse_(0, 1),
      rotation_inverse_(1, 0), rotation_inverse_(1, 1));
  cv::Point2f move = cv::Point2f(rotation_inverse_(0, 2) - r_icon_position_.x,
                                 rotation_inverse_(1, 2) - r_icon_position_.y);

  cv::Mat vec_2d = (cv::Mat1d(2, 4, 0.) << hammer_2d_(0, 0) + move.x - rotation_inverse_(0, 2),
      hammer_2d_(1, 0) + move.x - rotation_inverse_(0, 2),
      hammer_2d_(2, 0) + move.x - rotation_inverse_(0, 2),
      hammer_2d_(3, 0) + move.x - rotation_inverse_(0, 2),
      hammer_2d_(0, 1) + move.y - rotation_inverse_(1, 2),
      hammer_2d_(1, 1) + move.y - rotation_inverse_(1, 2),
      hammer_2d_(2, 1) + move.y - rotation_inverse_(1, 2),
      hammer_2d_(3, 1) + move.y - rotation_inverse_(1, 2));
  vec_2d = inverse_rotation * vec_2d;
  cv::Mat points_2d = (cv::Mat1d(2, 4, 0.) << vec_2d.at<double>(0, 0) + rotation_inverse_(0, 2),
      vec_2d.at<double>(0, 1) + rotation_inverse_(0, 2),
      vec_2d.at<double>(0, 2) + rotation_inverse_(0, 2),
      vec_2d.at<double>(0, 3) + rotation_inverse_(0, 2),
      vec_2d.at<double>(1, 0) + rotation_inverse_(1, 2),
      vec_2d.at<double>(1, 1) + rotation_inverse_(1, 2),
      vec_2d.at<double>(1, 2) + rotation_inverse_(1, 2),
      vec_2d.at<double>(1, 3) + rotation_inverse_(1, 2));
  int back = static_cast<int>((points_2d.at<double>(0, 0) + points_2d.at<double>(0, 1)) / 2 - rotation_inverse_(0, 2));
  points_2d = points_2d - (cv::Mat1d(2, 4, 0.) << back, back, back, back,
      0, 0, 0, 0);
  cv::Mat hammer_3d = hammer_3d_ - (cv::Mat1d(4, 3, 0.) << distance_, 0, 0,
      distance_, 0, 0,
      distance_, 0, 0,
      distance_, 0, 0);
//  ROS_INFO_STREAM("\n" << hammer_3d);

  cv::Mat_<double> r_vec(3, 1), t_vec(3, 1);
  cv::solvePnP(hammer_3d, points_2d.t(), cam_intrinsic_, dist_coefficients_, r_vec, t_vec, false, cv::SOLVEPNP_AP3P);

  cv::Mat1d rotation;
  cv::Rodrigues(r_vec, rotation);

  //2D --> 3D
  //
  //R^(-1) * (M^(-1) * zc * point_pixel - T) = point_world
  //
  //R:rot  M:cam_intrinsic_  T:vec
  cv::Mat point_pixel = (cv::Mat1d(3, 1, 0.) << points_2d.at<double>(0, 0),
      points_2d.at<double>(1, 0),
      1);
  cv::Mat point_world = (cv::Mat1d(3, 1, 0.) << hammer_3d.at<double>(0, 0),
      hammer_3d.at<double>(0, 1),
      hammer_3d.at<double>(0, 2));

  cv::Mat1d tes(3, 5, 0.);
  for(int i = 0; i < 5; i++)
  {
    t_vec.copyTo(tes(cv::Rect(i, 0, 1, 3)));
  }
  cv::Mat m1 = rotation.inv() * cam_intrinsic_.inv() * point_pixel;
  cv::Mat m2 = rotation.inv() * t_vec;
  double zc = (point_world.at<double>(2, 0) + m2.at<double>(2, 0)) / m1.at<double>(2, 0);

  cv::Mat points_pixel =(cv::Mat1d(3, 5, 0.) <<
      hammer_2d_(0, 0) * zc,
      hammer_2d_(1, 0) * zc,
      hammer_2d_(2, 0) * zc,
      hammer_2d_(3, 0) * zc,
      r_icon_position_.x * zc,
      hammer_2d_(0, 1) * zc,
      hammer_2d_(1, 1) * zc,
      hammer_2d_(2, 1) * zc,
      hammer_2d_(3, 1) * zc,
      r_icon_position_.y * zc,
      zc, zc, zc, zc, zc);
  cv::Mat points_world = rotation.inv() * (cam_intrinsic_.inv() * points_pixel - tes);

  float temp_3d[6]{};
  for(int i = 0; i < 3; i++)
  {
    temp_3d[2 * i] = static_cast<float>(points_world.at<double>(i, 4));
    temp_3d[2 * i + 1] = static_cast<float>((points_world.at<double>(i, 0) + points_world.at<double>(i, 2)) / 2);
  }
  float move_x = -(temp_3d[0]);
  float move_y = -(temp_3d[2]);
  float move_z = -(temp_3d[4]);
  temp_3d[0] = 0;
  temp_3d[2] = 0;
  temp_3d[4] = 0;
  temp_3d[1] += move_x;
  temp_3d[3] += move_y;
  temp_3d[5] += move_z;
  float radius = sqrt(pow(temp_3d[1], 2) + pow(temp_3d[3], 2) + pow(temp_3d[5], 2));

  cv::Point3f vec1(temp_3d[1], temp_3d[3], temp_3d[5]);
  cv::Point3f vec2(0, 1, 0);
  float costheta = vec1.dot(vec2) / (cv::norm(vec1) * cv::norm(vec2));
  float theta = acos(costheta);
  if(temp_3d[1] >= 0)
    theta = CV_PI / 2 - theta;
  else
    theta = CV_PI / 2 + theta;
  temp_3d[1] = radius * cos(theta);
  temp_3d[3] = radius * sin(theta);
  temp_3d[5] = 0;

//  float radius = sqrt(pow(temp_3d[0] - temp_3d[1], 2) + pow(temp_3d[2] - temp_3d[3], 2) + pow(temp_3d[4] - temp_3d[5], 2));

//  ROS_INFO_STREAM(rotation_inverse_);
//  ROS_INFO_STREAM("points_world:\n" << points_world);
//  ROS_INFO_STREAM("zc" << zc);
//  ROS_INFO_STREAM("R icon:\n");
//  ROS_INFO_STREAM("x: " << temp_3d[0]);
//  ROS_INFO_STREAM("y: " << temp_3d[2]);
//  ROS_INFO_STREAM("z: " << temp_3d[4]);
//  ROS_INFO_STREAM("armor center:\n");
//  ROS_INFO_STREAM("x: " << temp_3d[1]);
//  ROS_INFO_STREAM("y: " << temp_3d[3]);
//  ROS_INFO_STREAM("z: " << temp_3d[5]);
//  ROS_INFO_STREAM("distance:" << radius);

  memcpy(&one_target.pose.position.x, &temp_3d[0], sizeof(float) * 2);
  memcpy(&one_target.pose.position.y, &temp_3d[2], sizeof(float) * 2);
  memcpy(&one_target.pose.position.z, &temp_3d[4], sizeof(float) * 2);

  prediction_target_.detections.emplace_back(one_target);
  if (prediction_target_.detections.empty())
    return;
  prediction_target_.header.stamp = this->sync_.stamp;
  prediction_target_.header.seq = this->sync_.seq;
  prediction_pub_.publish(prediction_target_);
}

int WindmillAnalyzerBase::initCameraInfo(ros::NodeHandle& nh)
{
  camera_info_manager::CameraInfoManager info_manager{ nh };
  info_manager.loadCameraInfo("");
  sensor_msgs::CameraInfo info = info_manager.getCameraInfo();
  dist_coefficients_ = info.D;
  cam_intrinsic_.create(3, 3);
  memcpy(cam_intrinsic_.data, info.K.data(), 9 * sizeof(double));
  if (cv::determinant(cam_intrinsic_) == 0)
  {
    ROS_INFO("Init camera info failed");
    return FAILED;
  }
  ROS_INFO("Load camera info succeeded");
  ROS_INFO_STREAM(cam_intrinsic_);
  return SUCCESS;
}

void WindmillAnalyzerBase::execute(cv::Mat3b& src)
{
  mark_ = SUCCESS;
  position_z_.clear();
  index_ = 0;
  target_.detections.clear();       // clear the previous data
  prediction_target_.detections.clear();
  roi_amount_ = getRoiAmount(src);  // roi_amount <= 5 is guaranteed by recognizer.
  getRIconPosition(src);
  getActualRoiSize(src, 0);
  getInverseRotation(src);
  splitROIs(src);
  for (int id = 0; id < roi_amount_; ++id)
  {
    getConcavePosition(roi_array_[id], id);
    getInverseTransform(roi_array_[id], id);
  }
  cv::Mat1d points_3dim;
  modelWindmill(points_3dim, roi_amount_);  // Should be called first after all information has been cached.
  cv::Mat1d points_2dim;
  if (getWindmillVertices(points_2dim) != SUCCESS)
  {
      rm_msgs::TargetDetection one_target;
      one_target.id = 0;
      target_.detections.emplace_back(one_target);
      target_.header.stamp = this->sync_.stamp;
      target_.header.seq = this->sync_.seq;
      target_pub_.publish(target_);
      return;
  }
  sendPoints(points_2dim);
//  solvePose(points_3dim, points_2dim);
//  if(r_icon_confidence_==RIconPositionConfidence::REALLY_TRUSTABLE)
  rotationPrediction();
//  if(mark_==SUCCESS)
//  {
//    send2Ctrl();
//    send2TF();
//  }
//  send2Ctrl();
//  send2TF();
}

void WindmillAnalyzerBase::reConfigCB(Config& config, uint32_t __attribute__((unused)) level)
{
  constructLUT(config.armor_width, config.armor_height, config.distance);
  this->distance_ = config.distance;
  this->threshold_ = config.bin_threshold;
  this->stddev_typ_ = config.stddev_typ;
  this->coefficient_of_variation_typ_ = config.cv_typ;
}

void WindmillAnalyzerBase::initReConfig(const ros::NodeHandle& nh)
{
  analyzer_cfg_srv_ = std::make_unique<dynamic_reconfigure::Server<Config>>(nh);
  analyzer_cfg_cb_ = boost::bind(&WindmillAnalyzerBase::reConfigCB, this, _1, _2);
  analyzer_cfg_srv_->setCallback(analyzer_cfg_cb_);
}

cv::Mat_<double> WindmillAnalyzerBase::cam_intrinsic_{};
std::vector<double> WindmillAnalyzerBase::dist_coefficients_{};

}  // namespace rm_windmill_vision
