//
// Created by kaxns on 3/14/22.
//

#pragma once
#include <string>
#include <nodelet/nodelet.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "buff_vision/ImageConverterConfig.h"
#include <dynamic_reconfigure/server.h>
#include <opencv2/opencv.hpp>
#include <rm_msgs/StatusChange.h>
#include <rm_msgs/TargetDetection.h>
#include <rm_msgs/TargetDetectionArray.h>

namespace rm_buff_vision
{
class ColorProcessor
{
public:
  template <const int Reserved>
  static void colorEnhance(cv::Mat& in_out_src)
  {
    static_assert(Reserved >= 0 && Reserved <= 2, "");
    cv::medianBlur(in_out_src, in_out_src, 3);
    cv::Size origin_size = in_out_src.size();
    cv::pyrDown(in_out_src, in_out_src);
    cv::resize(in_out_src, in_out_src, in_out_src.size() / 2, cv::InterpolationFlags::INTER_AREA);
    typedef cv::Vec3b Pixel;
    // Parallel execution using C++11 lambda
    in_out_src.forEach<Pixel>([](Pixel& px, __attribute__((unused)) const int* position) -> void {
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
    cv::resize(in_out_src, in_out_src, origin_size);
  }

  typedef cv::Scalar Scalar;

  template <Scalar* const LowerBoundary, Scalar* const UpperBoundary>
  static void colorCvt(cv::Mat& src, cv::Mat1b& sel)
  {
    cv::Mat3b hsv(src.size());
    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, *LowerBoundary, *UpperBoundary, sel);
    // brute force dilating
    cv::filter2D(sel, sel, sel.depth(), cv::Mat_<uchar>(cv::Size(filter_size_, filter_size_), filter_step_));
    static cv::Mat const MOR_KERNEL{ cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size_, kernel_size_)) };
    cv::erode(sel, sel, MOR_KERNEL);
  }

  static Scalar upper4blue_;  // for blue, not its exact value
  static Scalar lower4blue_;  // for blue, not its exact value
  static Scalar upper4red_;   // for red, not its exact value
  static Scalar lower4red_;   // for red, not its exact value

  static int filter_size_;
  static int filter_step_;
  static int kernel_size_;

  static int target_is_red_;
};

class ImageConverterBase : public ColorProcessor
{
protected:
  ImageConverterBase()
  {
    cv::setNumThreads(cv::getNumberOfCPUs());
  }
  virtual ~ImageConverterBase() = default;

  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber cam_sub_;

  typedef buff_vision::ImageConverterConfig Config;
  void reConfigCB(Config& config, uint32_t level);
  void init(const ros::NodeHandle& nh);
  std::unique_ptr<dynamic_reconfigure::Server<Config>> image_converter_cfg_srv_;
  dynamic_reconfigure::Server<Config>::CallbackType image_converter_cfg_cb_;
};

class ImageConverterNodelet : private ImageConverterBase, public nodelet::Nodelet
{
public:
  ImageConverterNodelet() = default;
  ~ImageConverterNodelet() override = default;
  void onInit() override
  {
    ros::NodeHandle& nh = this->getPrivateNodeHandle();
    this->status_change_srv_ = nh.advertiseService("/buff_status_switch", &ImageConverterNodelet::changeStatusCB, this);
    ImageConverterBase::init(nh);
    it_ = std::make_shared<image_transport::ImageTransport>(image_transport::ImageTransport(nh));
    std::string base_topic{};
    nh.param("base_topic", base_topic, std::string("/galaxy_camera/image_raw"));
    // Subscribe to a synchronized image & camera info topic pair, version for class member function with bare pointer.
    cam_sub_ = it_->subscribe(base_topic, 1, &ImageConverterNodelet::callback, this);
    converted_img_pub_ = it_->advertise("/buff_vision/bin_image_raw", 1);
    // base_topic is now used for configuring the color mode.
//    nh.param("buff_color", base_topic, std::string("red"));
//    target_is_red_ = nh.param("buff_color", decltype(target_is_red_){});
    status_pub_ = nh.advertise<rm_msgs::TargetDetectionArray>("/status", 1);

    if(!nh.getParam("target_is_red", target_is_red_))
      ROS_WARN("No target_is_red specified");

    if(!nh.getParam("h_min4red", *(lower4red_.val)))
      ROS_WARN("No h_min4red specified");
    if(!nh.getParam("h_max4red", *(upper4red_.val)))
      ROS_WARN("No h_max4red specified");
    if(!nh.getParam("s_min4red", *(lower4red_.val + 1)))
      ROS_WARN("No s_min4red specified");
    if(!nh.getParam("s_max4red", *(upper4red_.val + 1)))
      ROS_WARN("No s_max4red specified");
    if(!nh.getParam("v_min4red", *(lower4red_.val + 2)))
      ROS_WARN("No v_min4red specified");
    if(!nh.getParam("v_max4red", *(upper4red_.val + 2)))
      ROS_WARN("No v_max4red specified");

    if(!nh.getParam("h_min4blue", *(lower4blue_.val)))
      ROS_WARN("No h_min4blue specified");
    if(!nh.getParam("h_max4blue", *(upper4blue_.val)))
      ROS_WARN("No h_max4blue specified");
    if(!nh.getParam("s_min4blue", *(lower4blue_.val + 1)))
      ROS_WARN("No s_min4blue specified");
    if(!nh.getParam("s_max4blue", *(upper4blue_.val + 1)))
      ROS_WARN("No s_max4blue specified");
    if(!nh.getParam("v_min4blue", *(lower4blue_.val + 2)))
      ROS_WARN("No v_min4blue specified");
    if(!nh.getParam("v_max4blue", *(upper4blue_.val + 2)))
      ROS_WARN("No v_max4blue specified");

    if(!nh.getParam("filter_size", filter_size_))
      ROS_WARN("No filter_size specified");
    if(!nh.getParam("filter_step", filter_step_))
      ROS_WARN("No filter_step specified");
    if(!nh.getParam("kernel_size", kernel_size_))
      ROS_WARN("No kernel_size specified");
  }

private:
  void callback(const sensor_msgs::ImageConstPtr& img)
  {
    status_target_.detections.clear();

    rm_msgs::TargetDetection one_target;
    std_msgs::Header sync = img->header;
    one_target.id = 1;
    int32_t temp[2]{};
    temp[0] = target_is_red_;
    temp[1] = armor_type_;
    memcpy(&one_target.pose.orientation.x, &temp[0], sizeof(int32_t));
    memcpy(&one_target.pose.orientation.y, &temp[1], sizeof(int32_t));
    status_target_.detections.push_back(one_target);
    status_target_.header.stamp = sync.stamp;
    status_target_.header.seq = sync.seq;
    status_pub_.publish(status_target_);

    if(!armor_type_)
      return;

    cv_bridge::CvImageConstPtr cv_image_const_ptr = cv_bridge::toCvShare(img, "bgr8");
    if (cv_image_const_ptr == nullptr)
      return;
    // cv::Mat3b input_img = *boost::const_pointer_cast<cv_bridge::CvImage>(cv_image_const_ptr);
    cv::Mat3b input_img = cv_image_const_ptr->image.clone();
    if (target_is_red_ == 0)
    {
      color_enhance_ = ColorProcessor::colorEnhance<0>;
      convert_color_ = ColorProcessor::colorCvt<&ColorProcessor::lower4blue_, &ColorProcessor::upper4blue_>;
    }
    else if (target_is_red_ == 1)
    {
      color_enhance_ = ColorProcessor::colorEnhance<2>;
      convert_color_ = ColorProcessor::colorCvt<&ColorProcessor::lower4red_, &ColorProcessor::upper4red_>;
    }
    else
    {
      NODELET_ERROR("Invalid buff color mode!");
      ros::shutdown();
    }
    (*color_enhance_)(input_img);
    cv::Mat1b bin_img(input_img.size());
    (*convert_color_)(input_img, bin_img);

    sync = cv_image_const_ptr->header;
    converted_img_pub_.publish(cv_bridge::CvImage(sync, "mono8", bin_img).toImageMsg());
  }

  void(*color_enhance_)(cv::Mat&) = nullptr;
  void(*convert_color_)(cv::Mat&, cv::Mat1b&) = nullptr;
  image_transport::Publisher converted_img_pub_;

  bool changeStatusCB(rm_msgs::StatusChange::Request& change, rm_msgs::StatusChange::Response& res)
  {
//    this->target_is_red_ = change.color == 0;
    if(change.target == 0)
      this->armor_type_ = 0;
    else
      this->armor_type_ = 1;
    ROS_INFO("armor type is %d:", armor_type_);
    res.switch_is_success = true;

    return true;
  }
  static int armor_type_;

  ros::ServiceServer status_change_srv_;

  ros::Publisher status_pub_;
  rm_msgs::TargetDetectionArray status_target_;
};

}  // namespace rm_buff_vision