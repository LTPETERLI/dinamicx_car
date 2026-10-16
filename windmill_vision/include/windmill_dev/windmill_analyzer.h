//
// Created by kaxns on 3/6/22.
//

#pragma once

#include <map>
#include <array>
#include <string>
#include <vector>
#include <algorithm>
#include <nodelet/nodelet.h>
#include <ros/node_handle.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "windmill_dev/bit_flags.h"
#include <rm_msgs/TargetDetection.h>
#include <tf/transform_broadcaster.h>
#include <rm_msgs/TargetDetectionArray.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include "windmill_dev/roi_info_encoding_protocol.h"
#include "windmill_dev/AnalyzerConfig.h"
#include <dynamic_reconfigure/server.h>
#include <cmath>

namespace rm_windmill_vision
{
class WindmillAnalyzerBase
{
public:
  WindmillAnalyzerBase() : angle_LUT_{}, inverse_trans_(5), roi_array_(5)
  {
    for (auto& item : inverse_trans_)
    {
      item.create(2, 3, CV_64FC1);
    }
    target_.header.frame_id = "camera_optical_frame";
  }
  virtual ~WindmillAnalyzerBase() = default;

protected:
  std::shared_ptr<image_transport::ImageTransport> it_;
  void execute(cv::Mat3b& src);
  /**
   * @param big_armor_width The length of the long side of the armor.
   * @param big_armor_height The length of the short side of the armor.
   * @param distance The distance between the center of armor and the center('R' icon) of the windmill.
   */
  void constructLUT(double big_armor_width, double big_armor_height, double distance);
  std::map<int, std::array<cv::Point2d, 4>> angle_LUT_;
  double distance_;  // The distance between the center of armor and the center('R' icon) of the windmill.

  std_msgs::Header sync_;

  int initCameraInfo(ros::NodeHandle& nh);
  tf::TransformBroadcaster broadcaster_;
  ros::Publisher target_pub_;
  ros::Publisher prediction_pub_;
  void setColorMethod(const std::string& color);
  int threshold_;

  typedef windmill_dev::AnalyzerConfig Config;
  void reConfigCB(Config& config, uint32_t level);
  void initReConfig(const ros::NodeHandle& nh);
  std::unique_ptr<dynamic_reconfigure::Server<Config>> analyzer_cfg_srv_;
  dynamic_reconfigure::Server<Config>::CallbackType analyzer_cfg_cb_;

  void sendPoints(cv::Mat1d& points_2d);

private:
  int getRoiAmount(cv::Mat3b& src)
  {
    return *reinterpret_cast<int*>(src.data + static_cast<int>(RoiInfoEncodingProtocol::ROI_AMOUNT) * src.step);
  }
  int roi_amount_;

  void getInverseTransform(cv::Mat3b& roi, size_t id)
  {
    memcpy(this->inverse_trans_[id].data,
           roi.data + static_cast<int>(RoiInfoEncodingProtocol::INVERSE_TRANSFORM) * roi.step, 2 * 3 * sizeof(double));
  }
  std::vector<cv::Mat> inverse_trans_;

  /**
   * @brief Get the concave position of the hammer.
   *
   * Do not call this function more than once in the callback.
   */
  void getConcavePosition(cv::Mat3b& roi, size_t id)
  {
    double temp[2];
    memcpy(temp, roi.data + static_cast<int>(RoiInfoEncodingProtocol::TRANSFORMED_CONCAVE_POSITION) * roi.step,
           2 * sizeof(double));
    this->concave_position_[id].x = static_cast<int>(temp[0]);
    this->concave_position_[id].y = static_cast<int>(temp[1]);
  }
  std::array<cv::Point, 5> concave_position_;

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
    this->r_icon_confidence_ = static_cast<RIconPositionConfidence>(*(ptr + 2));
  }
  cv::Point r_icon_position_;
  RIconPositionConfidence r_icon_confidence_;

  void getInverseRotation(cv::Mat3b& src)
  {
    rotation_inverse_ = (cv::Mat1d(2, 3, 0.) << 0., 0., 0.,
        0., 0., 0.);
    memcpy(rotation_inverse_.data, src.data + static_cast<int>(RoiInfoEncodingProtocol::INVERSE_ROTATION) * src.step,
           2 * 3 * sizeof(double));
  }
  cv::Mat1d rotation_inverse_;

  Flags flags_;

  /**
   * @brief Create model coordinates for the windmill according to ROIs received.
   *
   * This function will internally check the validity of ROIs and mark the result on actual_roi_size_.
   * @param amount The number of ROIs, including the hammer itself.
   * @param points_3d An Nx3 matrix with each row specifying a 3D point.
   */
  void modelWindmill(cv::Mat1d& points_3d, int amount);
  cv::Mat1d hammer_3d_;

  void splitROIs(cv::Mat3b& src)
  {
    int height_of_each_roi = src.rows / 5;
    for (int i = 0; i < roi_amount_; ++i)
    {
      auto& size = actual_roi_size_[0];
      if (size.width == 0)
        continue;
      roi_array_[i] = src(cv::Rect(cv::Point(0, height_of_each_roi * i), size));
    }
  }
  std::vector<cv::Mat3b> roi_array_;

  __attribute__((unused)) void cvtColorZZJ(cv::Mat3b& src);
  template <const int Reserved, const int Subtracted>
  static void cvtColor(cv::Mat3b& src, cv::Mat1b& bin, int thresh);
  void (*convert_color_)(cv::Mat3b&, cv::Mat1b&, int) = nullptr;

#define SUCCESS (0)
#define FAILED (1)
  int getHammerFourVertices(double* vertices);
#define _stddev_typ (8.)                     // stddev has a typical value which is great than 8
#define _coefficient_of_variation_typ (0.9)  // coefficient_of_variation has a typical value which is great than 0.9
  double stddev_typ_ = _stddev_typ;
  double coefficient_of_variation_typ_ = _coefficient_of_variation_typ;

  int getWindmillVertices(cv::Mat1d& points_2d);

  void solvePose(cv::Mat1d& points_3d, cv::Mat1d& points_2d);
  static cv::Mat_<double> cam_intrinsic_;
  static std::vector<double> dist_coefficients_;
  cv::Mat1d hammer_2d_;

  void send2TF();
  void send2Ctrl();
  void getQuaternion(cv::Mat_<double>& r_vec);
  std::array<double, 4> quat_vec_;
  std::array<double, 3> trans_vec_;
  // Of course there is only one target here, but using rm_msgs::TargetDetectionArray is convenient for controls.
  rm_msgs::TargetDetectionArray target_;
  rm_msgs::TargetDetectionArray prediction_target_;

  void rotationPrediction();

  int index_{};
  // Even if analyzer can get the right image of armor plate, however, morphological operation still hsa a little
  // possible to analyse the right image as wrong result. So index_ used for recording the amount of analysing
  // successfully, especially when code runs at TREAT_ALL and PNP_MODELING_ALL

  std::vector<double> position_z_{};
  int mark_;
};

class WindmillAnalyzerNodelet : private WindmillAnalyzerBase, public nodelet::Nodelet
{
public:
  WindmillAnalyzerNodelet() = default;
  ~WindmillAnalyzerNodelet() override = default;
  void onInit() override;

private:
  void callback(const sensor_msgs::ImageConstPtr& src);
  image_transport::Subscriber roi_sub_;
};

}  // namespace rm_windmill_vision