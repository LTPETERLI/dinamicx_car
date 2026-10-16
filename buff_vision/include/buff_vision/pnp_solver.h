//
// Created by koonyu on 2023/1/10.
//

#pragma once

#define ENABLE_VIRTUALIZATION 1

#include <algorithm>
#include <array>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <nodelet/nodelet.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>
#include <rm_msgs/TargetDetection.h>
#include <rm_msgs/TargetDetectionArray.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Int32.h>
#include "buff_vision/PnpSolverConfig.h"
#include "buff_vision/ImageConverterConfig.h"
#include "r_icon_position_confidence.h"
#include "bit_flags.h"

namespace rm_buff_vision
{
class PnpSolverBase
{
 public:
  PnpSolverBase() : angle_LUT_{}
  {
    target_.header.frame_id = "camera_optical_frame";
  };
  ~PnpSolverBase(){};

  int target_is_red_;

 protected:
#if ENABLE_VIRTUALIZATION
  cv::Mat3b* img_to_draw_ = nullptr;
#endif

  typedef cv::Point2f type_2D;
  typedef decltype(std::declval<type_2D>().x) real_type;

  std::shared_ptr<image_transport::ImageTransport> it_;

  std_msgs::Header sync_;

  tf::TransformBroadcaster broadcaster_;
  rm_msgs::TargetDetectionArray target_;
  ros::Publisher prediction_pub_;

#define _stddev_typ (8.)                     // stddev has a typical value which is great than 8
#define _coefficient_of_variation_typ (0.9)  // coefficient_of_variation has a typical value which is great than 0.9
  double stddev_typ_ = _stddev_typ;
  double coefficient_of_variation_typ_ = _coefficient_of_variation_typ;

  typedef buff_vision::PnpSolverConfig Config;
  void reConfigCB(Config& config, uint32_t level);
  void initReConfig(const ros::NodeHandle& nh);
  std::unique_ptr<dynamic_reconfigure::Server<Config>> pnp_solver_cfg_srv_;
  dynamic_reconfigure::Server<Config>::CallbackType pnp_solver_cfg_cb_;

  std::vector<std::array<int32_t, 8>> armor_points_;
  std::vector<std::array<float, 2>> vectors_;

  void execute(cv::Mat3b& src);
  int initCameraInfo(ros::NodeHandle& nh);
  void setColorMethod(int color);
  int threshold_;
  /**
   * @param big_armor_width The length of the long side of the armor.
   * @param big_armor_height The length of the short side of the armor.
   * @param distance The distance between the center of armor and the center('R' icon) of the windmill.
   */
  void constructLUT(double big_armor_width, double big_armor_height, double distance);
  double armor_width_;
  double armor_height_;
  double distance_;  // The distance between the center of armor and the center('R' icon) of the windmill.
  std::map<int, std::array<cv::Point2d, 4>> angle_LUT_;

 private:
  int getRoiAmount(cv::Mat3b& src)
  {
    return *reinterpret_cast<int*>(src.data + static_cast<int>(RoiInfoEncodingProtocol::ROI_AMOUNT) * src.step);
  }
  int roi_amount_;

  void getInverseTransform(cv::Mat3b& roi, size_t id)
  {
    cv::Mat inverse_tran(2, 3, CV_64FC1);
    memcpy(inverse_tran.data, roi.data + static_cast<int>(RoiInfoEncodingProtocol::INVERSE_TRANSFORM) * roi.step,
           2 * 3 * sizeof(double));
    inverse_trans_[id] = inverse_tran;
  }
  std::array<cv::Mat, 5> inverse_trans_;

  /**
   * @brief Get the actual size of a roi.
   *
   * A received ROI frame contains up to 5 ROIs. Instead of dividing the size of the roi frame by 5, each of them has an
   * actual size.
   */
  void getActualRoiSize(cv::Mat3b& roi, size_t id)
  {
    int* ptr = reinterpret_cast<int*>(roi.data + static_cast<int>(RoiInfoEncodingProtocol::ACTUAL_ROI_SIZE) * roi.step);
    this->actual_roi_size_[id] = cv::Size(*(ptr + 1), *ptr);
  }
  // According to modelWindmill(), if the width of an element is equal to 0, the element should be discarded.
  std::array<cv::Size, 5> actual_roi_size_;

  void getRIconPosition(cv::Mat3b& src)
  {
    int* ptr = reinterpret_cast<int*>(src.data +
        static_cast<int>(RoiInfoEncodingProtocol::CENTRAL_R_ORIGIN_POSITION) * src.step);
    this->r_icon_position_.x = *ptr;
    this->r_icon_position_.y = *(ptr + 1);
  }
  cv::Point r_icon_position_;

  void getImageCebter(cv::Mat3b& src)
  {
    int* ptr = reinterpret_cast<int*>(src.data +
        static_cast<int>(RoiInfoEncodingProtocol::ORIGINAL_FRAME_CENTER_POSITION) * src.step);
    this->img_center_.x = *ptr;
    this->img_center_.y = *(ptr + 1);
  }
  cv::Point img_center_;

  void modelWindmill(cv::Mat1d& points_3d, int amount);

  void splitROIs(cv::Mat3b& src)
  {
    int height_of_each_roi = src.rows / 5;
    for (int i = 0; i < roi_amount_; ++i)
    {
      auto& size = actual_roi_size_[i];
      if (size.width == 0)
        continue;
      roi_array_[i] = src(cv::Rect(cv::Point(0, height_of_each_roi * i), size));
    }
  }
  std::array<cv::Mat3b, 5> roi_array_;

  template <const int Reserved, const int Subtracted>
  static void cvtColor(cv::Mat3b& src, cv::Mat1b& bin, int thresh);
  void (*convert_color_)(cv::Mat3b&, cv::Mat1b&, int) = nullptr;

#define SUCCESS (0)
#define FAILED (1)

  int getHammerFourVertices(double* vertices);
  int getBuffVertices(cv::Mat1d& points_2d);
  void solvePnp(cv::Mat1d& points_3d, cv::Mat1d& points_2d);
  void send2TF();
  void send2Ctrl();
  void getQuaternion(cv::Mat_<double>& r_vec);
  void rotationPrediction(cv::Mat1d& points_2d);
  cv::Mat1d spanner_3d_;
  cv::Mat1d spanner_2d_;
  cv::Mat1d armor_points;
  std::array<double, 4> quat_vec_;
  std::array<double, 3> trans_vec_;

  rm_msgs::TargetDetectionArray prediction_target_;

  static cv::Mat_<double> cam_intrinsic_;
  static std::vector<double> dist_coefficients_;

  std::vector<double> position_z_{};

  Flags flags_;

  static cv::Scalar upper4blue_;  // for blue, not its exact value
  static cv::Scalar lower4blue_;  // for blue, not its exact value
  static cv::Scalar upper4red_;   // for red, not its exact value
  static cv::Scalar lower4red_;   // for red, not its exact value
};

class PnpSolverNodelet : private PnpSolverBase, public nodelet::Nodelet
{
 public:
  PnpSolverNodelet(){};
  ~PnpSolverNodelet() override{};
  void onInit() override;

 private:
  void callback(const sensor_msgs::ImageConstPtr& src);
  void statusCallback(const rm_msgs::TargetDetectionArray& status);
  image_transport::Subscriber roi_sub_;
  ros::Subscriber status_sub_;

  int armor_type_;
};

} // namespace rm_buff_vision
