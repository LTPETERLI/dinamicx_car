//
// Created by koonyu on 2023/1/25.
//

#pragma once
#include <array>
#include <vector>
#include <string>
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/core/mat.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <rm_msgs/TargetDetection.h>
#include <rm_msgs/TargetDetectionArray.h>
#include "bit_flags.h"
#include "r_icon_position_confidence.h"
#include "image_converter.h"
#include "target.h"
#include "buff_vision/BuffRecognizerConfig.h"

namespace rm_buff_vision
{
class BuffRecognizerBase
{
 public:
  BuffRecognizerBase() = default;
  virtual ~BuffRecognizerBase()
  {
  }

//  rm_msgs::TargetDetectionArray target_;
  ros::Publisher prediction_pub_;

 protected:
  typedef cv::Point2f type_2D;
  typedef decltype(std::declval<type_2D>().x) real_type;
  typedef std::vector<cv::Point> external_contour_type;
  typedef std::vector<external_contour_type> external_contours_type;

  bool locateNotTriggeredFan(cv::Mat1b& sel);
  void findHammer(external_contours_type& ext_contours, cv::Mat1b& sel);
  void checkHammerAndLocateRCenter(external_contours_type& ext_contours);
  void fixRectPoints();
  void sendPoints();
  void sendInfo();

  // Fans' components are as follow
  //
  //                              /'''''''''''/
  //                             / __________/
  // |---------------------------  |                 <------ inactive_spanner
  // |---------------------------  |
  //                             \ ''''''''''\
  //                              \___________\
  //
  //   /''''''''''''''''''''''''''/'''''''''''/
  //  /                          / __________/
  // |---------------------------  |                 <------ active_spanner
  // |---------------------------  |
  //  \                          \ ''''''''''\
  //   \__________________________\___________\
  //
  //                                   ''''''''|
  //                                 ''        |
  //                                   ''''    |
  //                                      |    |
  //                                      |    |     <------ hat
  //                                      |    |
  //                                   ''''    |
  //                                 ''        |
  //                                   '''''''''
  //

  std::shared_ptr<image_transport::ImageTransport> it_;
  cv_bridge::CvImageConstPtr cv_img_const_ptr_ = nullptr;

  typedef buff_vision::BuffRecognizerConfig Config;
  void reConfigCB(Config& config, uint32_t level);
  void init(const ros::NodeHandle& nh);
  std::unique_ptr<dynamic_reconfigure::Server<Config>> buff_recognizer_cfg_srv_;
  dynamic_reconfigure::Server<Config>::CallbackType buff_recognizer_cfg_cb_;

  std_msgs::Header sync_;
  rm_msgs::TargetDetectionArray prediction_target_;

  static Target hammer_;
  static std::vector<Target> is_hammer_;
  cv::Point2f r_center_;
  cv::Point img_center_;
  bool is_found_;

  double hammer_min_area_;
  double hammer_max_area_;
  double hammer_min_rect_ratio_;
  double hammer_max_rect_ratio_;
  double min_white_ratio_;
  double max_white_ratio_;

  double r_icon_min_length_;
  double r_icon_max_length_;
  double r_icon_min_area_;
  double r_icon_max_area_;
  double r_icon_min_rect_ratio_;
  double r_icon_max_rect_ratio_;

  double min_rc_distance_;
  double max_rc_distance_;

  int big_rect_size_;
  int small_rect_size_;
  int intensity_;

  double small_rect_move_;

//  int take_photo_;

  Flags flags_;

#if ENABLE_VIRTUALIZATION
  cv::Mat3b* img_to_draw_ = nullptr;
#endif

  virtual void extractHammerRoi(int length) = 0;

  int roi_amount_{};
};

}  // namespace rm_buff_vision

#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <ros/callback_queue.h>
#include <thread>
namespace rm_buff_vision
{
/**
 * @brief
 *
 */
class BuffRecognizerNodelet : private BuffRecognizerBase, public nodelet::Nodelet
{
 public:
  BuffRecognizerNodelet()
  {
  }
  ~BuffRecognizerNodelet() override
  {
    if (my_thread_.joinable())
      my_thread_.join();
  }
  void onInit() override;

//  static int count_;

 private:
  inline void setRoiSize();
  void callback(const sensor_msgs::ImageConstPtr& raw_img, const sensor_msgs::ImageConstPtr& binary_img);
  void statusCallback(const rm_msgs::TargetDetectionArray& status);
  void extractHammerRoi(int length) override;

  image_transport::Publisher roi_pub_{};
  std::thread my_thread_{};
  std::shared_ptr<cv::Mat3b> roi_{ new cv::Mat3b() };
  ros::Subscriber status_sub_;
#if ENABLE_VIRTUALIZATION
  image_transport::Publisher debug_pub_{};
#endif

  int armor_type_;
};

}  // namespace rm_buff_vision