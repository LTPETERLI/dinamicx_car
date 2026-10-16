//
// Created by kaxns on 10/23/21.
//

#pragma once
#define ENABLE_POSE_SOLVER_LOGGING_AND_VISUALIZATION 0
#define ENABLE_POSE_SOLVER_TF_OUTPUT 0

#include <rm_vision/vision_base/posesolver_interface.h>
#include <rm_vision/vision_base/processor_interface.h>
#include <rm_msgs/TargetDetectionArray.h>
#include <rm_msgs/TargetDetection.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <string>
#include <vector>
#include <array>

namespace rm_pose_solver
{
/**
 * @brief
 * Pose solver.
 * @note
 * Method used here is PnP, more precisely, P3P methods (4.2.0: SOLVEPNP_AP3P, 3.2.0: SOLVEPNP_P3P)
 * Need 4 input points to return a unique solution.
 * The reason for choosing SOLVEPNP_AP3P: https://www.youtube.com/watch?v=X5h_4okDzyI
 */
template <typename Targets, typename RuntimeInfo, typename Img = cv_bridge::CvImagePtr&>
class PnpPoseSolverCore : public rm_vision::PoseSolverInterface<Targets, RuntimeInfo, Img>
{
public:
  PnpPoseSolverCore() : cam_intrinsic_mat_k_(3, 3, double{ 0 })
  {
    this->target_array_.header.frame_id = "camera_optical_frame";
  }
  ~PnpPoseSolverCore() override = default;

  void initializeOnlyOnce(ros::NodeHandle& outside_nh, sensor_msgs::CameraInfoConstPtr& cam_info) override;
  void initializeOnlyOnce(ros::NodeHandle& outside_nh, sensor_msgs::CameraInfo& cam_info) override
  {
    auto cam_info_ptr = boost::make_shared<const sensor_msgs::CameraInfo>(cam_info);
    initializeOnlyOnce(outside_nh, cam_info_ptr);
  }
  void paramReConfig() override;

protected:
  void initializeOnlyOnce(ros::NodeHandle& outside_nh) final;
  void poseSolver() override;
  typedef Img ImgType;
  typedef Targets TargetsType;
  typedef RuntimeInfo RuntimeInfoType;
  typedef typename std::remove_reference<Img>::type img_type;
  typedef typename std::remove_reference<Targets>::type targets_type;
  typedef typename std::remove_reference<RuntimeInfo>::type runtime_info_type;
  img_type img_{};
  ros::NodeHandle parent_nh_;
  ros::NodeHandle armor_size_nh_;  // node handle for taking armor_size
  rm_msgs::TargetDetectionArray target_array_{};
  bool is_large_armor_;
  double distance_to_centre_;

  /**
   * @note
   * The detected_targets.label has the following correspondence:
   * @var [1 --> hero]
   * @var [2 --> engineer]
   * @var [3~5 --> infantry]
   * @var [6 --> outpost]
   * @var [7 --> sentry]
   * @var [8 --> base]
   */
  virtual void run(TargetsType detected_targets) = 0;
  template <typename T>
  void initialize(std::vector<T>& points_2dim, double target_h, double target_w);
  void initialize() override;
  void send2Ctl() override;
  void executeSingleTarget(const int& target_label);

private:
  // variables that may be frequently modified
  cv::Matx<double, 4, 3> points_3d_mat_;  // 4rows, 3cols, each row represents a 3D point (Nx3 1-channel)
  cv::Matx<double, 4, 2> points_2d_mat_;  // 2D points, calculated by processor (Nx2 1-channel)
  std::array<double, 4> quat_vec_;        // quaternion
  std::array<double, 3> trans_vec_;       // translation vector
  // end variables that may be frequently modified

  // variables that will hardly change
  tf::TransformBroadcaster broadcaster_;
#if ENABLE_POSE_SOLVER_LOGGING_AND_VISUALIZATION
  ros::Publisher pnp_result_pub_;
  std::vector<cv::Point3f> test_3D_;  // 3D points for testing the result of solvePnP
#endif
  cv::Mat_<double> cam_intrinsic_mat_k_;
  std::vector<double> dist_coefficients_;
  ros::Publisher detection_pub_;
  int axis_mapping_[2];  // basis transformation descriptor. details in initialize()
  int mapping_2D_[4];    // Map the input 2D points to points_2d_mat_ to ensure correspondence between 2D and 3D.
  // end variables that will hardly change

  virtual void visualizeResult(cv::Mat_<double>& K, cv::Mat_<double>& r_vec, cv::Mat_<double>& t_vec);
  void getQuaternion(cv::Mat_<double>& r_vec);
  void send2TF(const int& target_label);
  void append2TargetArray(const int& target_label);
};

}  // namespace rm_pose_solver

namespace rm_pose_solver
{
/**
 * @brief This is a template function which prepares four 2D points and actual height and width of a target.
 * @type T type of each point
 *
 * @param points_2dim 2D input points
 * @param target_h actual height of a target
 * @param target_w actual width of a target
 *
 * @note runtime_info_ will also be used here to adapt tracking mode.
 * @note Reference: http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/RegionOfInterest.html
 */
template <typename Targets, typename RuntimeInfo, typename Img>
template <typename T>
inline void PnpPoseSolverCore<Targets, RuntimeInfo, Img>::initialize(std::vector<T>& points_2dim, double target_h,
                                                                     double target_w)
{
  ROS_ASSERT_MSG(points_2dim.size() == 4, "Invalid size: Size of the input 2D points != 4");
#if ENABLE_POSE_SOLVER_LOGGING_AND_VISUALIZATION
  ROS_INFO("target_h, target_w: %f, %f", target_h, target_w);
#endif
  double half_y = target_h / 2;
  double half_x = target_w / 2;
  //       +z.
  //  1-----/-4
  //  |    /  |
  // ||   o---||----> x
  //  |   |   |
  //  2---|---3
  //      |
  //      v +y
  // 2D points has its definition above.
  // !!! FORBIDDEN: 2D points may have another definition shown as follow:
  // A = (P1 + P4) / 2; B = (P1 + P2) / 2; C = (P2 + P3) / 2; D = (P3 + P4) / 2
  // Because projective geometry does not satisfy midpoint invariance, A, B, C, and D are not true midpoints.

  // clang-format off
  this->points_3d_mat_ = cv::Matx43d(-half_x, -half_y, 0,    // first 3D point
                                     -half_x,  half_y, 0,    // second 3D point
                                      half_x,  half_y, 0,    // third 3D point
                                      half_x, -half_y, 0);  // fourth 3D point

  this->points_2d_mat_ = cv::Matx<double, 4, 2>(points_2dim[mapping_2D_[0]].x, points_2dim[mapping_2D_[0]].y,
                                                points_2dim[mapping_2D_[1]].x, points_2dim[mapping_2D_[1]].y,
                                                points_2dim[mapping_2D_[2]].x, points_2dim[mapping_2D_[2]].y,
                                                points_2dim[mapping_2D_[3]].x, points_2dim[mapping_2D_[3]].y);
  // clang-format on
  initialize();  // modifies the points_3d_mat_ according to axis_mapping_
}

/**
 * This is a overwritten function.
 * @brief Change the definition of the world coordinate system, and then modify the points_3d_mat_.
 *
 * Applies a basis transformation to all 3D points if they should be
 * expressed in a particular Cartesian coordinate system which is different
 * from the original one.
 * @note Since the last row of points_3d_mat_.t() contains only zeros,
 * whatever the third column of the basis-transformation is will not affect the final result.
 * Therefore, the size of axis_mapping_ is only 2.
 *
 * @example
 * @code
 * // mapping:
 * // original  --->  dst
 * // +x(1) ---> +y(2)
 * // +y(2) ---> -z(-3)
 * //
 * //   origin                            dst
 * //        +z.                             ^ +z
 * //  1------/--4                      1----|----4
 * //  |     /   |                      |    |    |
 * //  |    o---------> +x  ------>     |    o--------> +y
 * //  |    |    |                      |   /     |
 * //  2----|----3                      2--/------3
 * //       |                             .+x
 * //       |
 * //       v +y
 * @endcode
 */
template <typename Targets, typename RuntimeInfo, typename Img>
inline void PnpPoseSolverCore<Targets, RuntimeInfo, Img>::initialize()
{
  if (axis_mapping_[0] == 1 && axis_mapping_[1] == 2)
    return;

  cv::Matx33d basis_transformation{ 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  // Modifies the basis-transformation matrix according to axis_mapping_.
  basis_transformation(abs(axis_mapping_[0]) - 1, 0) = (axis_mapping_[0] > 0 ? 1. : -1.);
  basis_transformation(abs(axis_mapping_[1]) - 1, 1) = (axis_mapping_[1] > 0 ? 1. : -1.);

  this->points_3d_mat_ = (basis_transformation * points_3d_mat_.t()).t();
}

/**
 * Will be executed only if ENABLE_POSE_SOLVER_LOGGING_AND_VISUALIZATION defined and not equals to 0
 * @brief Visualize the result of solvePnP.
 *
 * @enum color of x axis: RED
 * @enum color of y axis: GREEN
 * @enum color of z axis: BLUE
 */
template <typename Targets, typename RuntimeInfo, typename Img>
void PnpPoseSolverCore<Targets, RuntimeInfo, Img>::visualizeResult(cv::Mat_<double>& K, cv::Mat_<double>& r_vec,
                                                                   cv::Mat_<double>& t_vec)
{
#if ENABLE_POSE_SOLVER_LOGGING_AND_VISUALIZATION
  std::vector<cv::Point2f> test_2d;  // to be projected from test_3D_
  cv::projectPoints(test_3D_, r_vec, t_vec, K, dist_coefficients_, test_2d);
  // Draw X and Y.
  cv::line(img_->image, test_2d[0], test_2d[1], cv::Scalar(0, 0, 255), 2);
  cv::putText(img_->image, "X", test_2d[1], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255));
  cv::line(img_->image, test_2d[0], test_2d[2], cv::Scalar(0, 255, 0), 2);
  cv::putText(img_->image, "Y", test_2d[2], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0));

  if (abs(axis_mapping_[0] * axis_mapping_[1]) >= 3
      // The absolute value of one of axis_mapping_[0] and axis_mapping_[1]
      // is equal to 3, so the Z axis is parallel to the plane.
      || ((abs(axis_mapping_[0]) == 1) && (axis_mapping_[0] * axis_mapping_[1]) < 0)
      // X and Y not exchanged, but one of them is mirrored,
      // so the Z axis points to us and is visible.
      || ((abs(axis_mapping_[0]) == 2) && (axis_mapping_[0] * axis_mapping_[1]) > 0)
      // X and Y exchanged and both mirrored or not mirrored, so the Z axis points to us and is visible.
  )
  {
    cv::line(img_->image, test_2d[0], test_2d.back(), cv::Scalar(255, 0, 0), 2);
    cv::putText(img_->image, "Z", test_2d.back(), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0));
  }
  else
  // The Z axis is normal to the plane and points behind the plane and is not actually visible, so give it a perspective effect.
  {
    // Ref: https://docs.opencv.org/4.2.0/dc/dd2/classcv_1_1LineIterator.html
    cv::LineIterator itr{ img_->image, test_2d[0], test_2d.back(), 8 };
    float step = 150. / itr.count;
    for (int i = 0; i < itr.count; ++i, ++itr)
    {
      auto blue = static_cast<uchar>(255 - (i * step));
      auto red = static_cast<uchar>(static_cast<int>(blue) > 150 ? blue - 100 : 50);
      auto value = cv::Vec3b{ blue, static_cast<uchar>(255 - blue), red };
      *reinterpret_cast<cv::Vec3b*>(*itr) = value;
      // Draw multiple adjacent points of itr to get a thicker line.
      img_->image.template at<cv::Vec3b>(itr.pos().y + 1, itr.pos().x) = value;
      img_->image.template at<cv::Vec3b>(itr.pos().y - 1, itr.pos().x) = value;
      img_->image.template at<cv::Vec3b>(itr.pos().y, itr.pos().x + 1) = value;
      img_->image.template at<cv::Vec3b>(itr.pos().y, itr.pos().x - 1) = value;
    }
    cv::putText(img_->image, "z", test_2d.back(), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0));
  }
  this->pnp_result_pub_.publish(img_->toImageMsg());
#endif
}

/**
 * @brief Convert rotation vector to quaternion
 * @ref https://en.wikipedia.org/wiki/Euler%E2%80%93Rodrigues_formula
 */
template <typename Targets, typename RuntimeInfo, typename Img>
void PnpPoseSolverCore<Targets, RuntimeInfo, Img>::getQuaternion(cv::Mat_<double>& r_vec)
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

template <typename Targets, typename RuntimeInfo, typename Img>
void PnpPoseSolverCore<Targets, RuntimeInfo, Img>::poseSolver()
{
  // The estimated pose is thus the rotation (r_vec) and the translation (t_vec) vectors.
  cv::Mat_<double> r_vec(3, 1), t_vec(3, 1);
// For different OpenCV versions, dispatch different PnP methods.
#if CV_MAJOR_VERSION == 4
  // clang-format off
  cv::solvePnP(points_3d_mat_, points_2d_mat_, cam_intrinsic_mat_k_,
               dist_coefficients_, r_vec, t_vec, false,cv::SOLVEPNP_AP3P);
  // clang-format on
#elif CV_MAJOR_VERSION == 3
  // ROS_INFO("3.2.0");
  // Note: OpenCV 3.2.0 does not has the method cv::SOLVEPNP_AP3P
  // Ref: https://docs.opencv.org/3.2.0/d9/d0c/group__calib3d.html
  // After trying several methods, the following is valid for the 3.2.0 version.

  std::vector<cv::Point3f> obj_points = {};
  std::vector<cv::Point2f> img_points = {};
  for (int i = 0; i < 4; ++i)
  {
    obj_points.emplace_back(
        cv::Point3f(points_3d_mat_.val[0 + i * 3], points_3d_mat_.val[1 + i * 3], points_3d_mat_.val[2 + i * 3]));
    img_points.emplace_back(cv::Point2f(points_2d_mat_.val[0 + i * 2], points_2d_mat_.val[1 + i * 2]));
  }

  cv::solvePnP(obj_points, img_points, cam_intrinsic_mat_k_, dist_coefficients_, r_vec, t_vec, false, cv::SOLVEPNP_P3P);
#endif
#if ENABLE_POSE_SOLVER_LOGGING_AND_VISUALIZATION
  ROS_INFO("r_vec = (%f, %f, %f)", r_vec(0, 0), r_vec(1, 0), r_vec(2, 0));
  ROS_INFO("t_vec = (%f, %f, %f)", t_vec(0, 0), t_vec(1, 0), t_vec(2, 0));
  ROS_INFO("------------------------");
  visualizeResult(cam_intrinsic_mat_k_, r_vec, t_vec);
#endif

#if CV_MAJOR_VERSION == 4
  trans_vec_ = t_vec.reshape(1, 1);
#elif CV_MAJOR_VERSION == 3
  trans_vec_ = { t_vec(0, 0), t_vec(1, 0), t_vec(2, 0) };
#endif
  getQuaternion(r_vec);
}

template <typename Targets, typename RuntimeInfo, typename Img>
void PnpPoseSolverCore<Targets, RuntimeInfo, Img>::send2TF(const int& target_label)
{
  std::string child_frame_id("target_ID:" + std::to_string(target_label));
  this->broadcaster_.sendTransform(
      tf::StampedTransform(tf::Transform(tf::Quaternion(quat_vec_[0], quat_vec_[1], quat_vec_[2], quat_vec_[3]),
                                         tf::Vector3(trans_vec_[0], trans_vec_[1], trans_vec_[2])),
                           this->sync_.stamp, this->sync_.frame_id, child_frame_id));
  // !!! FORBIDDEN:        this->img_->header.stamp, this->img_->header.frame_id, child_frame_id));
  // When ENABLE_POSE_SOLVER_LOGGING_AND_VISUALIZATION is set to 0, img_ is a nullptr.
}

/**
 * @brief executes poseSolver() and send2TF(target_label), then appends the position info to msg
 */
template <typename Targets, typename RuntimeInfo, typename Img>
void PnpPoseSolverCore<Targets, RuntimeInfo, Img>::executeSingleTarget(const int& target_label)
{
  poseSolver();
#if ENABLE_POSE_SOLVER_TF_OUTPUT
  send2TF(target_label);
#endif
  append2TargetArray(target_label);
}

template <typename Targets, typename RuntimeInfo, typename Img>
void PnpPoseSolverCore<Targets, RuntimeInfo, Img>::append2TargetArray(const int& target_label)
{
  rm_msgs::TargetDetection one_target;
  one_target.id = target_label;
  one_target.is_large_armor = is_large_armor_;
  one_target.distance_to_image_center = distance_to_centre_;

  // @todo one_target.confidence = should be implemented?
  one_target.pose.position.x = trans_vec_[0];
  one_target.pose.position.y = trans_vec_[1];
  one_target.pose.position.z = trans_vec_[2];

  auto& quaternion = one_target.pose.orientation;
  quaternion.x = quat_vec_[0];
  quaternion.y = quat_vec_[1];
  quaternion.z = quat_vec_[2];
  quaternion.w = quat_vec_[3];

  this->target_array_.detections.emplace_back(one_target);
}

template <typename Targets, typename RuntimeInfo, typename Img>
void PnpPoseSolverCore<Targets, RuntimeInfo, Img>::send2Ctl()
{
  this->target_array_.header.stamp = this->sync_.stamp;
  this->target_array_.header.seq = this->sync_.seq;
  detection_pub_.publish(this->target_array_);
}

template <typename Targets, typename RuntimeInfo, typename Img>
void PnpPoseSolverCore<Targets, RuntimeInfo, Img>::paramReConfig()
{
#if ENABLE_POSE_SOLVER_LOGGING_AND_VISUALIZATION
  float vector_length = this->template getParam<double>(parent_nh_, "vector_length", 0);
  test_3D_ = {
    cv::Point3f(0, 0, 0),
    cv::Point3f(vector_length, 0, 0),
    cv::Point3f(0, vector_length, 0),
    cv::Point3f(0, 0, vector_length),
  };
#endif
  ros::NodeHandle axis_mapping_nh(parent_nh_, "axis_mapping");
  axis_mapping_[0] = this->template getParam<int>(axis_mapping_nh, "x", 1);
  axis_mapping_[1] = this->template getParam<int>(axis_mapping_nh, "y", 2);

  ros::NodeHandle mapping_2d_nh(parent_nh_, "input_mapping");
  // The awkward-looking default values 1, 0, 3 and 2 currently matches the order of points defined in Processor.
  mapping_2D_[0] = this->template getParam<int>(mapping_2d_nh, "first", 1);
  mapping_2D_[1] = this->template getParam<int>(mapping_2d_nh, "second", 0);
  mapping_2D_[2] = this->template getParam<int>(mapping_2d_nh, "third", 3);
  mapping_2D_[3] = this->template getParam<int>(mapping_2d_nh, "fourth", 2);

  target_array_.header.frame_id =
      this->template getParam<std::string>(parent_nh_, "camera_coordinate", "camera_optical_frame");
}

template <typename Targets, typename RuntimeInfo, typename Img>
void PnpPoseSolverCore<Targets, RuntimeInfo, Img>::initializeOnlyOnce(ros::NodeHandle& outside_nh)
{
  parent_nh_ = outside_nh;
  armor_size_nh_ = ros::NodeHandle(parent_nh_, "armor_size");
  detection_pub_ = parent_nh_.advertise<rm_msgs::TargetDetectionArray>("/detection", 10);
#if ENABLE_POSE_SOLVER_LOGGING_AND_VISUALIZATION
  ros::NodeHandle pose_solver_test(parent_nh_, "rm_pose_solver");
  pnp_result_pub_ = pose_solver_test.advertise<sensor_msgs::Image>("pnp_result", 10);
#endif
  paramReConfig();
}

template <typename Targets, typename RuntimeInfo, typename Img>
void PnpPoseSolverCore<Targets, RuntimeInfo, Img>::initializeOnlyOnce(ros::NodeHandle& outside_nh,
                                                                      sensor_msgs::CameraInfoConstPtr& cam_info)
{
  ROS_ASSERT(cam_info != nullptr);
  memcpy(cam_intrinsic_mat_k_.data, cam_info->K.data(), 9 * sizeof(double));
  ROS_ASSERT(cv::determinant(cam_intrinsic_mat_k_) != 0);
  dist_coefficients_ = cam_info->D;
  initializeOnlyOnce(outside_nh);
}

}  // namespace rm_pose_solver
