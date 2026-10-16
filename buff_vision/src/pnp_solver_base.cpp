//
// Created by koonyu on 2023/1/10.
//

#include "../include/buff_vision/pnp_solver.h"

namespace rm_buff_vision
{
void PnpSolverBase::execute(cv::Mat3b& src)
{
  prediction_target_.detections.clear();
  vectors_.clear();

  setColorMethod(target_is_red_);
  roi_amount_ = getRoiAmount(src);
  getRIconPosition(src);
  getImageCebter(src);
  static const int HEIGHT_OF_EACH_ROI = src.rows / 5;
  for (int id = 0; id < roi_amount_; ++id)
  {
    cv::Mat3b roi_with_id =
        src.operator()(cv::Rect(cv::Point(0, HEIGHT_OF_EACH_ROI * id), cv::Size(src.cols, HEIGHT_OF_EACH_ROI)));
    getInverseTransform(roi_with_id, id);
    getActualRoiSize(roi_with_id, id);
  }
  splitROIs(src);
  cv::Mat1d points_2dim;
  if (getBuffVertices(points_2dim) != SUCCESS)
  {
    prediction_target_.header.stamp = this->sync_.stamp;
    prediction_target_.header.seq = this->sync_.seq;
    prediction_pub_.publish(prediction_target_);

    return;
  }
  spanner_2d_ = points_2dim(cv::Rect(0, 0, 2, 4));
  for (int id = 0; id < roi_amount_; ++id)
  {
    cv::Mat1f pts = points_2dim(cv::Rect(0, id, 2, 4));
    vectors_.push_back({(pts(1, 0) + pts(2, 0)) / 2 - r_icon_position_.x, (pts(1, 1) + pts(2, 1)) / 2 - r_icon_position_.y});
  }
  cv::Mat1d points_3dim;
  modelWindmill(points_3dim, roi_amount_);
  rotationPrediction(spanner_2d_);
//  solvePnp(points_3dim, points_2dim);
//  send2Ctrl();
//  send2TF();
}

void PnpSolverBase::modelWindmill(cv::Mat1d& points_3d, int amount)
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
  // Get the original direction vector.
  // [ r00 r01 ]   [ 1 ]   _         [r00]
  // [         ] * [   ] = R * n_x = [   ]
  // [ r10 r11 ]   [ 0 ]             [r10]
  // _
  // R : Rotation part of inverse_transform
  // !!! NOTE:
  // The original direction vector must have the same direction as (1, 0) after affine transformation.
  // This can easily be done in WindmillRecognizer with the help of the position of the "R" icon.
  auto get_original_dir_vec = [this](size_t id) { return type_2D(vectors_[id][0], vectors_[id][1]); };
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
        --amount;                       // Make sure that the following points_3dim takes right number of 3D points.
      }
    }
  }

  points_3d.create(amount * 4, 3);
  for (int i = 0; i < 4; ++i)
  {
    /* x */ points_3d(i, 0) = this->angle_LUT_.find(0)->second[i].x;
    /* y */ points_3d(i, 1) = this->angle_LUT_.find(0)->second[i].y;
    /* z */ points_3d(i, 2) = 0.;
  }
  cv::Mat1d spanner_3d(4, 3, 0.);
  points_3d(cv::Rect(0, 0, 3, 4)).copyTo(spanner_3d);
  spanner_3d_ = spanner_3d;

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

void PnpSolverBase::constructLUT(double big_armor_width, double big_armor_height, double distance)
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

int PnpSolverBase::getBuffVertices(cv::Mat1d& points_2d)
{
  points_2d.create(roi_amount_ * 4, 2);
  if (getHammerFourVertices(reinterpret_cast<double*>(points_2d.data)) != SUCCESS)
    return FAILED;
  // TODO : treat fans
  return SUCCESS;
}

int PnpSolverBase::getHammerFourVertices(double *vertices)
{
#if ENABLE_VIRTUALIZATION
  static image_transport::Publisher hammer_pub = it_->advertise("hammer_roi", 1);
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

  cv::Mat3b hammer = roi_array_[0];
  cv::Mat1b binary, bin_copy;
  (*convert_color_)(hammer, binary, threshold_);
  bin_copy = binary.clone();
  // for Morphological gradient
  static const auto KERNEL = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::Mat1b eroded;
  cv::erode(binary, eroded, KERNEL);
//  cv::bitwise_xor(binary, eroded, binary);  // !!! FORBIDDEN: binary -= eroded;  // slower than bit operation

#if ENABLE_VIRTUALIZATION
  cv::Mat3b debug(hammer.rows * 2, hammer.cols * 1);
  hammer.copyTo(debug(cv::Rect(0, 0, hammer.cols * 1, hammer.rows * 1)));
  cv::cvtColor(binary * 255, hammer, cv::COLOR_GRAY2BGR);
#endif

  cv::normalize(binary, binary, cv::NORM_MINMAX);
  cv::Mat_<int> proj;  // use projection method
  cv::reduce(binary, proj, 0 /* reduced to a single row */, cv::REDUCE_SUM, CV_32S);

#if ENABLE_VIRTUALIZATION
  // Draw the reduced result.
  for (int i = 0; i < proj.cols; ++i)
    if (proj(0, i))
      cv::line(hammer, cv::Point(i, hammer.rows - 1 - proj(0, i)), cv::Point(i, hammer.rows - 1), cv::Scalar(0, 255, 0));
#endif

  cv::Scalar mean;
  cv::Scalar stddev;
  cv::Mat1b proj_mask;

  cv::meanStdDev(proj, mean, stddev, proj != 0 /* only nonzero element counts */);
  double coefficient_of_variation = stddev[0] / mean[0];

#if ENABLE_VIRTUALIZATION
  ROS_INFO("mean: %f; stddev: %f, cv: %f\n", mean[0], stddev[0], coefficient_of_variation);
  // Draw the average line.
  cv::line(hammer, cv::Point(0, hammer.rows - 1 - mean[0]), cv::Point(hammer.cols - 1, hammer.rows - 1 - mean[0]),
           cv::Scalar(0, 255, 255));
#endif

  if (!(coefficient_of_variation > coefficient_of_variation_typ_ || stddev[0] > stddev_typ_))
#if !ENABLE_VIRTUALIZATION
    return FAILED;
#else
  {
    da_cha(hammer);
    hammer.copyTo(debug(cv::Rect(0, hammer.rows * 1, hammer.cols * 1, hammer.rows * 1)));
    hammer_pub.publish(cv_bridge::CvImage(sync_, "bgr8", debug).toImageMsg());
    return FAILED;
  }
#endif

  int x_tangent;
  for (int i = proj.cols / 4; i < proj.cols; ++i)
    if (proj(0, i) > mean[0])
    {
      x_tangent = i;
      break;
    }
  x_tangent -= 1;

  cv::Mat r4c2 = (cv::Mat_<double>(4, 3) << hammer.cols, 0, 1,
      x_tangent, 0, 1,
      x_tangent, hammer.rows, 1,
      hammer.cols, hammer.rows, 1) * (inverse_trans_[0].t());

  memcpy(vertices, r4c2.data, 8 * sizeof(double));

#if ENABLE_VIRTUALIZATION
  hammer.copyTo(debug(cv::Rect(0, hammer.rows * 1, hammer.cols * 1, hammer.rows * 1)));
  hammer_pub.publish(cv_bridge::CvImage(sync_, "bgr8", debug).toImageMsg());
#endif
  return SUCCESS;
}

//void PnpSolverBase::solvePnp(cv::Mat1d& points_3d, cv::Mat1d& points_2d)
//{
//  for(size_t i = 0; i < armor_points_.size(); ++i)
//  {
//    cv::Mat1d points_3d_cut = points_3d(cv::Rect(0, i * 4, 3, 4));
//    cv::Mat1d points_2d_cut = points_2d(cv::Rect(0, i * 4, 2, 4));
//    cv::Mat_<double> r_vec(3, 1), t_vec(3, 1);
//    cv::solvePnP(points_3d_cut, points_2d_cut, cam_intrinsic_, dist_coefficients_, r_vec, t_vec, false, cv::SOLVEPNP_AP3P);
//    if(r_center_.z==static_cast<int>(RIconPositionConfidence::REALLY_TRUSTABLE)) // Use R icon to check and rectify the result.
//    {
//      cv::Mat img_point(1, 1, CV_64FC2);
//      cv::projectPoints(cv::Mat1d(3, 1, 0.), r_vec, t_vec, cam_intrinsic_, dist_coefficients_, img_point);
//      double error = pow(*(double*)img_point.data - r_center_.x, 2) +
//          pow(*((double*)img_point.data + 1) - r_center_.y, 2);
//#if ENABLE_VIRTUALIZATION
//      ROS_INFO("error_before: %f", error);
//#endif
//      if (error < 4.)  // tolerable error: radius < 2
//        goto end;
//      points_3d_cut.push_back(cv::Mat1d(1, 3, 0.));
//      points_2d_cut.push_back([this]() -> cv::Mat1d{
//        cv::Mat1d temp(1, 2);
//        *(double*)temp.data = r_center_.x;
//        *((double*)temp.data + 1) = r_center_.y;
//        return temp;
//      }());
//      cv::solvePnP(points_3d_cut, points_2d_cut, cam_intrinsic_, dist_coefficients_, r_vec, t_vec, true, cv::SOLVEPNP_ITERATIVE);
//      cv::projectPoints(cv::Mat1d(3, 1, 0.), r_vec, t_vec, cam_intrinsic_, dist_coefficients_, img_point);
//#if ENABLE_VIRTUALIZATION
//      error = pow(*(double*)img_point.data - r_center_.x, 2) + pow(*((double*)img_point.data + 1) - r_center_.y, 2);
//      ROS_INFO("error_after: %f", error);
//#endif
//      end:;
//    }
//
//    cv::Mat1d rotation;
//    cv::Rodrigues(r_vec, rotation);
//    //                         1     2
//    //    .------- frame1       |`````|
//    //    |          o==========|  o <|--- frame2(origin is armor's center)
//    //    |          ^          |_____|       |
//    // T1 |          |          4     3       |
//    //    |          '.______________________.'
//    //    |                     T2
//    //    v
//    //    o
//    //  camera_frame
//    //
//    //  frame2 ==> camera_frame = T1 * T2
//    //  camera_frame ==> frame2 = (T1 * T2).inv()
//    //
//    //      ___         ___
//    //      |         | t1|
//    //  T = |  R_3x3  | t2|
//    //      |_________| t3|
//    //      | 0  0  0   1 |
//    //      ```         ```
//    // clang-format off
//    static cv::Mat1d transform2 = (cv::Mat1d(4, 4) << 1, 0, 0, distance_,
//        0, 1, 0, 0,
//        0, 0, 1, 0,
//        0, 0, 0, 1);
//    // clang-format on
//    cv::Mat1d transform1(4, 4, 0.);
//    rotation.copyTo(transform1(cv::Rect(0, 0, 3, 3)));
//    memcpy(transform1.data + 3 * transform1.step, t_vec.data, 3 * sizeof(double));
//    transform1 = transform1.t();
//    transform1(3, 3) = 1.;
//    // The transform is transposed to get the rotation from camera_frame to frame2.
//    // After transposing, the translation vector changes from one column to one row, which can be easily recorded with memcpy.
//    // The translation vector is exactly the coordinate of armor center under camera frame.
//    cv::Mat1d transform = (transform1 * transform2).t();
//    rotation = transform(cv::Rect(0, 0, 3, 3));
//    memcpy(t_vec.data, transform.data + transform.step * 3, 3 * sizeof(double));
//    cv::Rodrigues(rotation, r_vec);
//
//#if CV_MAJOR_VERSION == 4
//    trans_vec_ = t_vec.reshape(1, 1);
//#elif CV_MAJOR_VERSION == 3
//    trans_vec_ = { t_vec(0, 0), t_vec(1, 0), t_vec(2, 0) };
//#endif
//    getQuaternion(r_vec);
//
//    rm_msgs::TargetDetection one_target;
//    one_target.id = 9;  // TODO : id for windmill = ?
//    one_target.pose.position.x = trans_vec_[0];
//    one_target.pose.position.y = trans_vec_[1];
//    one_target.pose.position.z = trans_vec_[2];
//    auto& quaternion = one_target.pose.orientation;
//    quaternion.x = quat_vec_[0];
//    quaternion.y = quat_vec_[1];
//    quaternion.z = quat_vec_[2];
//    quaternion.w = quat_vec_[3];
//    position_z_.emplace_back(trans_vec_[2]);
//  }
//}
//
//void PnpSolverBase::getQuaternion(cv::Mat_<double>& r_vec)
//{
//  double phi = sqrt(r_vec.dot(r_vec));
//  double w = 1, x = 0, y = 0, z = 0;
//  if (phi > 1e-6)
//  {
//    w = cos(phi / 2.);
//    r_vec *= (sin(phi / 2.) / phi);  // (r_vec /= phi) *= sin(phi / 2.)
//    x = r_vec(0, 0);
//    y = r_vec(1, 0);
//    z = r_vec(2, 0);
//  }
//  this->quat_vec_ = { x, y, z, w };
//}
//
//void PnpSolverBase::send2TF()
//{
//  this->broadcaster_.sendTransform(tf::StampedTransform(
//      tf::Transform(tf::Quaternion(quat_vec_[0], quat_vec_[1], quat_vec_[2], quat_vec_[3]),
//                    tf::Vector3(trans_vec_[0], trans_vec_[1], trans_vec_[2])),
//      sync_.stamp, sync_.frame_id.empty() ? "camera_optical_frame" : sync_.frame_id, "windmill_target"));
//}
//
//void PnpSolverBase::send2Ctrl()
//{
//  if (target_.detections.empty())
//    return;
//  target_.header.stamp = this->sync_.stamp;
//  target_.header.seq = this->sync_.seq;
//  target_pub_.publish(target_);
//}

void PnpSolverBase::rotationPrediction(cv::Mat1d& points_2d)
{
  cv::Mat1d pts = points_2d;
  rm_msgs::TargetDetection one_target;
  one_target.id = 10;

  // o----------------------> +x
  // |
  // |                * concave
  // |               '
  // |              '
  // |             * r_icon_position_
  // v +y
  //
  cv::Point2d bc_vec = cv::Point((pts(0, 0) + pts(2, 0)) / 2, (pts(0, 1) + pts(2, 1)) / 2) - r_icon_position_;
  cv::Point2d x_vec = cv::Point2d(1, 0);
  double theta_1 = asin(x_vec.cross(bc_vec) / cv::norm(bc_vec));
  if(bc_vec.x<=0)
  {
   if(bc_vec.y>=0)
     theta_1 = CV_PI - theta_1;
   else
     theta_1 = -CV_PI - theta_1;
  }

  cv::Mat rotation = (cv::Mat1d(2, 2, 0.) << cos(theta_1), -sin(theta_1),
      sin(theta_1), cos(theta_1));

  cv::Mat vec_2d = (cv::Mat1d(2, 4, 0.) << pts(0, 0) - r_icon_position_.x, pts(1, 0) - r_icon_position_.x,
      pts(2, 0) - r_icon_position_.x, pts(3, 0) - r_icon_position_.x,
      pts(0, 1) - r_icon_position_.y, pts(1, 1) - r_icon_position_.y,
      pts(2, 1) - r_icon_position_.y, pts(3, 1) - r_icon_position_.y);

  vec_2d = rotation.inv() * vec_2d;
  cv::Mat spanner_2d = (cv::Mat1d(2, 4, 0.) << (vec_2d.at<double>(0, 0) + vec_2d.at<double>(0, 3)) / 2 + r_icon_position_.x,
      (vec_2d.at<double>(0, 1) + vec_2d.at<double>(0, 2)) / 2 + r_icon_position_.x,
      (vec_2d.at<double>(0, 1) + vec_2d.at<double>(0, 2)) / 2 + r_icon_position_.x,
      (vec_2d.at<double>(0, 0) + vec_2d.at<double>(0, 3)) / 2 + r_icon_position_.x,
      (vec_2d.at<double>(1, 0) + vec_2d.at<double>(1, 1)) / 2 + r_icon_position_.y,
      (vec_2d.at<double>(1, 0) + vec_2d.at<double>(1, 1)) / 2 + r_icon_position_.y,
      (vec_2d.at<double>(1, 2) + vec_2d.at<double>(1, 3)) / 2 + r_icon_position_.y,
      (vec_2d.at<double>(1, 2) + vec_2d.at<double>(1, 3)) / 2 + r_icon_position_.y);
//  int move_x = static_cast<int>((spanner_2d.at<double>(0, 0) + spanner_2d.at<double>(0, 1)) / 2 - img_center_.x);
//  int move_y = static_cast<int>((spanner_2d.at<double>(1, 1) + spanner_2d.at<double>(1, 2)) / 2 - img_center_.y);
//  spanner_2d = spanner_2d - (cv::Mat1d(2, 4, 0.) << move_x, move_x, move_x, move_x,
//      move_y, move_y, move_y, move_y);
//  cv::Mat spanner_3d = spanner_3d_ - (cv::Mat1d(4, 3, 0.) << distance_, 0, 0,
//      distance_, 0, 0,
//      distance_, 0, 0,
//      distance_, 0, 0);

//  cv::Mat_<double> r_vec(3, 1), t_vec(3, 1);
//  cv::solvePnP(spanner_3d, spanner_2d.t(), cam_intrinsic_, dist_coefficients_, r_vec, t_vec, false, cv::SOLVEPNP_AP3P);
//
//  cv::Mat1d rot;
//  cv::Rodrigues(r_vec, rot);
//
//  //2D --> 3D
//  //
//  //R^(-1) * (M^(-1) * zc * point_pixel - T) = point_world
//  //
//  //R:rot  M:cam_intrinsic_  T:vec
//  cv::Mat point_pixel = (cv::Mat1d(3, 1, 0.) << spanner_2d.at<double>(0, 0),
//      spanner_2d.at<double>(1, 0),
//      1);
//  cv::Mat point_world = (cv::Mat1d(3, 1, 0.) << spanner_3d.at<double>(0, 0),
//      spanner_3d.at<double>(0, 1),
//      spanner_3d.at<double>(0, 2));
//
//  cv::Mat1d tes(3, 5, 0.);
//  for(int i = 0; i < 5; i++)
//  {
//    t_vec.copyTo(tes(cv::Rect(i, 0, 1, 3)));
//  }
//  cv::Mat m1 = rot.inv() * cam_intrinsic_.inv() * point_pixel;
  //  cv::Mat m2 = rot.inv() * t_vec;
  //  double zc = (point_world.at<double>(2, 0) + m2.at<double>(2, 0)) / m1.at<double>(2, 0);
  //
  //  cv::Mat points_pixel =(cv::Mat1d(3, 5, 0.) << pts(0, 0) * zc,
  //      pts(1, 0) * zc,
  //      pts(2, 0) * zc,
  //      pts(3, 0) * zc,
  //      r_icon_position_.x * zc,
  //      pts(0, 1) * zc,
  //      pts(1, 1) * zc,
  //      pts(2, 1) * zc,
  //      pts(3, 1) * zc,
  //      r_icon_position_.y * zc,
  //      zc, zc, zc, zc, zc);
  //  cv::Mat points_world = rot.inv() * (cam_intrinsic_.inv() * points_pixel - tes);

  //  float temp_2d[6]{};
  //  for(int i = 0; i < 3; i++)
  //  {
  //    temp_2d[2 * i] = static_cast<float>(points_world.at<double>(i, 4));
  //    temp_2d[2 * i + 1] = static_cast<float>((points_world.at<double>(i, 0) + points_world.at<double>(i, 2)) / 2);
  //  }
  //  temp_2d[1] -= temp_2d[0];
  //  temp_2d[3] -= temp_2d[2];
  //  temp_2d[5] -= temp_2d[4];
  //  temp_2d[0] = 0;
  //  temp_2d[2] = 0;
  //  temp_2d[4] = 0;
  //  float radius = sqrt(pow(temp_2d[1], 2) + pow(temp_2d[3], 2) + pow(temp_2d[5], 2));

  //  float temp_2d[6]{};
  //  cv::Mat x_target = (cv::Mat1d(2, 1, 0.) << (spanner_2d.at<double>(0, 0) + spanner_2d.at<double>(0, 2)) / 2,
  //      (spanner_2d.at<double>(1, 0) + spanner_2d.at<double>(1, 2)) / 2);
  //  vec_2d = (cv::Mat1d(2, 1, 0.) << x_target.at<double>(0, 0) - r_icon_position_.x,
  //      x_target.at<double>(1, 0) - r_icon_position_.y);
  //  vec_2d = rotation * vec_2d;
  //  x_target = (cv::Mat1d(2, 1, 0.) << vec_2d.at<double>(0, 0) + r_icon_position_.x,
  //      vec_2d.at<double>(1, 0) + r_icon_position_.y);
  //  temp_2d[1] = x_target.at<double>(0, 0);
  //  temp_2d[3] = x_target.at<double>(1, 0);
  //  temp_2d[5] = 0;
  //  temp_2d[0] = r_icon_position_.x;
  //  temp_2d[2] = r_icon_position_.y;
  //  temp_2d[4] = 0;

  //  int32_t temp_2d[2]{};
  //  temp_2d[0] = static_cast<int32_t>(r_icon_position_.x);
  //  temp_2d[1] = static_cast<int32_t>(r_icon_position_.y);

  float temp_2d[6]{};
  cv::Mat x_target = (cv::Mat1d(2, 1, 0.) << (spanner_2d.at<double>(0, 0) + spanner_2d.at<double>(0, 2)) / 2,
                      (spanner_2d.at<double>(1, 0) + spanner_2d.at<double>(1, 2)) / 2);
  vec_2d = (cv::Mat1d(2, 1, 0.) << x_target.at<double>(0, 0) - r_icon_position_.x,
            x_target.at<double>(1, 0) - r_icon_position_.y);
  vec_2d = rotation * vec_2d;
  x_target = (cv::Mat1d(2, 1, 0.) << vec_2d.at<double>(0, 0) + r_icon_position_.x,
              vec_2d.at<double>(1, 0) + r_icon_position_.y);
  temp_2d[1] = x_target.at<double>(0, 0);
  temp_2d[3] = x_target.at<double>(1, 0);
  temp_2d[5] = 0;
  temp_2d[0] = r_icon_position_.x;
  temp_2d[2] = r_icon_position_.y;
  temp_2d[4] = 0;

  //  for(int i = 0; i < 3; i++)
  //  {
  //    temp_2d[2 * i] = static_cast<float>(points_world.at<double>(i, 4));
  //    temp_2d[2 * i + 1] = static_cast<float>((points_world.at<double>(i, 0) + points_world.at<double>(i, 2)) / 2);
  //  }
  //  temp_2d[1] -= temp_2d[0];
  //  temp_2d[3] -= temp_2d[2];
  //  temp_2d[5] -= temp_2d[4];
  //  temp_2d[0] = 0;
  //  temp_2d[2] = 0;
  //  temp_2d[4] = 0;
  //  float radius = sqrt(pow(temp_2d[1], 2) + pow(temp_2d[3], 2) + pow(temp_2d[5], 2));
  //
  //  cv::Point3f vec1(temp_2d[1], temp_2d[3], temp_2d[5]);
  //  cv::Point3f vec2(0, 1, 0);
  //  float theta_2 = acos(vec1.dot(vec2) / cv::norm(vec1));
  //  if(temp_2d[1] >= 0)
  //    theta_2 = CV_PI / 2 - theta_2;
  //  else
  //    theta_2 = CV_PI / 2 + theta_2;
  //  temp_2d[1] = radius * cos(theta_2);
  //  temp_2d[3] = radius * sin(theta_2);
  //  temp_2d[5] = 0;

  memcpy(&one_target.pose.position.x, &temp_2d[0], sizeof(float) * 2);
  memcpy(&one_target.pose.position.y, &temp_2d[2], sizeof(float) * 2);
  memcpy(&one_target.pose.position.z, &temp_2d[4], sizeof(float) * 2);
  //  one_target.pose.position.x = temp_2d[0];
  //  one_target.pose.position.y = temp_2d[1];

  int32_t temp[8]{};
  for (int i = 0; i < 4; ++i)
  {
    temp[2 * i] = static_cast<int32_t>(pts.at<double>(i, 0));
    temp[2 * i + 1] = static_cast<int32_t>(pts.at<double>(i, 1));
  }
  memcpy(&one_target.pose.orientation.x, &temp[0], sizeof(int32_t) * 2);
  memcpy(&one_target.pose.orientation.y, &temp[2], sizeof(int32_t) * 2);
  memcpy(&one_target.pose.orientation.z, &temp[4], sizeof(int32_t) * 2);
  memcpy(&one_target.pose.orientation.w, &temp[6], sizeof(int32_t) * 2);

  prediction_target_.detections.emplace_back(one_target);
  if (prediction_target_.detections.empty())
    return;
  prediction_target_.header.stamp = this->sync_.stamp;
  prediction_target_.header.seq = this->sync_.seq;
  prediction_pub_.publish(prediction_target_);
}

template <const int Reserved, const int Subtracted>
void PnpSolverBase::cvtColor(cv::Mat3b& src, cv::Mat1b& bin, int thresh)
{
//  static_assert(Reserved >= 0 && Reserved <= 2, "");
//  static_assert(Subtracted >= 0 && Subtracted <= 2, "");
//  static_assert(Reserved != Subtracted, "");
//  std::vector<cv::Mat1b> bgr(3);
//  cv::split(src, bgr);
//  cv::Mat1b diff = bgr[Reserved] - bgr[Subtracted];
//  cv::medianBlur(diff, diff, 3);
//  cv::threshold(diff, bin, thresh, 1, cv::THRESH_BINARY);

  static_assert(Reserved >= 0 && Reserved <= 2, "");
  cv::Mat3b in_out_src = src.clone();
  cv::medianBlur(src, in_out_src, 3);
  typedef cv::Vec3b Pixel;
  in_out_src.template forEach([](Pixel& px, __attribute__((unused)) const int* position) -> void {
    // min = min(b, g, r)
    uchar min = px(1) < px(2) ? px(1) : px(2);
    min = min < px(0) ? min : px(0);
    constexpr int a = (2 - Reserved) * (Reserved != 1);
    constexpr int b = (a > 0) && (Reserved != 1) ? a - 1 : 2 - (Reserved == 2);
    px(a) = 0;
    px(b) = 0;
    px(Reserved) -= min;
    // px -= Pixel(255, 255, min);  // do not use. Relatively slow because calling constructor everytime.
  });
  cv::bitwise_not(in_out_src, in_out_src);

  cv::Mat3b hsv(in_out_src.size());
  cv::cvtColor(in_out_src, hsv, cv::COLOR_BGR2HSV);
  if(Reserved==0)
    cv::inRange(hsv, cv::Scalar{ 26, 50, 50 }, cv::Scalar{ 34, 255, 255 }, bin);
  else
    cv::inRange(hsv, cv::Scalar{ 78, 80, 80 }, cv::Scalar{ 99, 255, 255 }, bin);
  cv::filter2D(bin, bin, bin.depth(), cv::Mat_<uchar>(cv::Size(3, 3), 1));
}

void PnpSolverBase::setColorMethod(int color)
{
  if (color == 1)
    convert_color_ = cvtColor<2, 1>;
  else if (color == 0)
    convert_color_ = cvtColor<0, 2>;
  else
    ROS_ASSERT_MSG(false, "Invalid windmill color mode!");
}

void PnpSolverBase::reConfigCB(Config &config, uint32_t __attribute__((unused)) level)
{
  this->distance_ = config.distance;
  this->threshold_ = config.bin_threshold;
  this->stddev_typ_ = config.stddev_typ;
  this->coefficient_of_variation_typ_ = config.cv_typ;
  this->armor_width_ = config.armor_width;
  this->armor_height_ = config.armor_height;
  constructLUT(armor_width_, armor_height_, distance_);
}

void PnpSolverBase::initReConfig(const ros::NodeHandle& nh)
{
  pnp_solver_cfg_srv_ = std::make_unique<dynamic_reconfigure::Server<Config>>(nh);
  pnp_solver_cfg_cb_ = boost::bind(&PnpSolverBase::reConfigCB, this, _1, _2);
  pnp_solver_cfg_srv_->setCallback(pnp_solver_cfg_cb_);
}

int PnpSolverBase::initCameraInfo(ros::NodeHandle& nh)
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

cv::Mat_<double> PnpSolverBase::cam_intrinsic_{};
std::vector<double> PnpSolverBase::dist_coefficients_{};

} // namespace rm_buff_vision
