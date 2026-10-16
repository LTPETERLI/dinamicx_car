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
#include "windmill_dev/ImageConverterConfig.h"
#include <dynamic_reconfigure/server.h>
#include <opencv2/opencv.hpp>

namespace rm_windmill_vision
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
    cv::filter2D(sel, sel, sel.depth(), cv::Mat_<uchar>(cv::Size(3, 3), 1));
    static cv::Mat const MOR_KERNEL{ cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)) };
    cv::erode(sel, sel, MOR_KERNEL);
  }

  static Scalar upper4blue_;  // for blue, not its exact value
  static Scalar lower4blue_;  // for blue, not its exact value
  static Scalar upper4red_;   // for red, not its exact value
  static Scalar lower4red_;   // for red, not its exact value
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

  typedef windmill_dev::ImageConverterConfig Config;
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
    ImageConverterBase::init(nh);
    it_ = std::make_shared<image_transport::ImageTransport>(image_transport::ImageTransport(nh));
    std::string base_topic{};
    nh.param("base_topic", base_topic, std::string("/galaxy_camera/image_raw"));
    // Subscribe to a synchronized image & camera info topic pair, version for class member function with bare pointer.
    cam_sub_ = it_->subscribe(base_topic, 1, &ImageConverterNodelet::callback, this);
    converted_img_pub_ = it_->advertise("/windmill_vision/bin_image_raw", 1);
    // base_topic is now used for configuring the color mode.
    nh.param("windmill_color", base_topic, std::string("red"));
    if (base_topic == "blue")
    {
      color_enhance_ = ColorProcessor::colorEnhance<0>;
      convert_color_ = ColorProcessor::colorCvt<&ColorProcessor::lower4blue_, &ColorProcessor::upper4blue_>;
    }
    else if (base_topic == "red")
    {
      color_enhance_ = ColorProcessor::colorEnhance<2>;
      convert_color_ = ColorProcessor::colorCvt<&ColorProcessor::lower4red_, &ColorProcessor::upper4red_>;
    }
    else
    {
      NODELET_ERROR("Invalid windmill color mode!");
      ros::shutdown();
    }
  }

private:
  void callback(const sensor_msgs::ImageConstPtr& img)
  {
    cv_bridge::CvImageConstPtr cv_image_const_ptr = cv_bridge::toCvShare(img, "bgr8");
    if (cv_image_const_ptr == nullptr)
      return;
    // cv::Mat3b input_img = *boost::const_pointer_cast<cv_bridge::CvImage>(cv_image_const_ptr);
    cv::Mat3b input_img = cv_image_const_ptr->image.clone();
    (*color_enhance_)(input_img);
    cv::Mat1b bin_img(input_img.size());
    (*convert_color_)(input_img, bin_img);
    converted_img_pub_.publish(cv_bridge::CvImage(cv_image_const_ptr->header, "mono8", bin_img).toImageMsg());
  }

  void(*color_enhance_)(cv::Mat&) = nullptr;
  void(*convert_color_)(cv::Mat&, cv::Mat1b&) = nullptr;
  image_transport::Publisher converted_img_pub_;
};

}  // namespace rm_windmill_vision