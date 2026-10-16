//
// Created by kaxns on 11/11/21.
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
#include "windmill_dev/bit_flags.h"
#include "windmill_dev/roi_info_encoding_protocol.h"
#include "windmill_dev/image_converter.h"

namespace rm_windmill_vision
{
class WindmillRecognizerBase
{
public:
  WindmillRecognizerBase() = default;
  virtual ~WindmillRecognizerBase()
  {
  }
protected:
  typedef cv::Point2f type_2D;
  typedef decltype(std::declval<type_2D>().x) real_type;
  typedef std::vector<cv::Point> external_contour_type;
  typedef std::vector<external_contour_type> external_contours_type;
  // Specify a fan(Rotated Rect).
  // fan[0]: fan's center
  // fan[1]: fan's direction vector with magnitude = max(fan.height, fan.width)
  // fan[2 : 5]: fan's 4 vertices
  typedef std::array<type_2D, 6> fan_type;
  typedef std::vector<fan_type> fans_type;
  // hammer_type.first: length
  // hammer_type.second: pointer to the contour
  typedef std::pair<int, std::vector<cv::Point>*> hammer_type;

  bool locateNotTriggeredFan(cv::Mat1b& sel);
  bool pcaHammer(hammer_type& hammer, std::vector<cv::Point>& out_approx);
  inline void normApprox(type_2D& center, type_2D& eigenvector, std::vector<cv::Point>& out_approx);
  void findHammerAndFans(external_contours_type& ext_contours, hammer_type& hammer, fans_type* const fans = nullptr);

  std::shared_ptr<image_transport::ImageTransport> it_;
  cv_bridge::CvImageConstPtr cv_img_const_ptr_ = nullptr;

#if ENABLE_VIRTUALIZATION
  cv::Mat3b* img_to_draw_ = nullptr;
#endif

  // Used for storing the average of concave position of the hammer.
  // ----------------------------
  // |                [[[[[[[]  |
  // |                *     []  |
  // |  >>>>>>>>>>>>>>]     []  |
  // |                *     []  |
  // |                [[[[[[[]  |
  // ----------------------------
  // Each '*' denotes a concave position.
  // The running water lamp, denoted by '>>>>>>>>>>>>>>', will unavoidably increase the complexity of processing the
  // image above. In the process of binarization, the running water lamp will eventually get several small blocks,
  // neither logical filtering nor morphology can directly or indirectly remove their influence without increasing the
  // time consumed and reducing the accuracy of finding vertices.
  // By concave_position_ we can easily cut off the part which contains the running water lamp.
  cv::Point concave_position_{};
  // Used for storing 4 vertices of the rotated_rect of hammer(this is the final purpose).
  // The so called 'rotated_rect' is not simply given by cv::minAreaRect(), see getHammerDescription() for more details.
  // Firstly its first element will store the "center" of hammerhead, which is very useful in getHammerDescription().
  std::array<type_2D, 4> hammer_description_{};
  inline void getHammerDescription(int length);
  // Used for storing 2D position of 'R' (this is the final purpose).
  // The first and second element will firstly store the center and eigenvector calculated by PCA, respectively.
  // Then the center and eigenvector will be used to find the 'R' icon.
  // Finally, 'R' icon's position will be stored as the first element.
  // In addition, r_icon_position_[1].x will eventually be used to indicate whether the position can be trusted.
  std::array<type_2D, 2> r_icon_position_{};
  void locateCenterIconR(external_contours_type& ext_contours, hammer_type& hammer, fans_type& fans);
  virtual void extractHammerRoi(int length) = 0;
  int roi_amount_{};
  fans_type fans_{};
  std::vector<cv::Point2f> rotation_inverse_;
  cv::Mat1d rot_inverse_;
};

}  // namespace rm_windmill_vision

#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <ros/callback_queue.h>
#include <thread>
namespace rm_windmill_vision
{
/**
 * @brief
 *
 */
class WindmillRecognizerNodelet : private WindmillRecognizerBase, public nodelet::Nodelet
{
public:
  WindmillRecognizerNodelet()
  {
  }
  ~WindmillRecognizerNodelet() override
  {
    if (my_thread_.joinable())
      my_thread_.join();
  }
  void onInit() override;

private:
  inline void setRoiSize();
  void callback(const sensor_msgs::ImageConstPtr& raw_img, const sensor_msgs::ImageConstPtr& binary_img);
  void extractHammerRoi(int length) override;

  Flags flags_;
  image_transport::Publisher roi_pub_{};
  std::thread my_thread_{};
  std::shared_ptr<cv::Mat3b> roi_{ new cv::Mat3b() };
#if ENABLE_VIRTUALIZATION
  image_transport::Publisher debug_pub_{};
#endif
};

}  // namespace rm_windmill_vision
