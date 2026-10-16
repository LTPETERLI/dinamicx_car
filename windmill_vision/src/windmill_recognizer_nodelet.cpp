//
// Created by kaxns on 3/15/22.
//

#include "windmill_dev/windmill_recognizer.h"
#include <string>

namespace rm_windmill_vision
{
/**
 * @ref https://zhuanlan.zhihu.com/p/48980327
 */
inline void WindmillRecognizerNodelet::setRoiSize()
{
  ros::NodeHandle& nh = this->getPrivateNodeHandle();
  // Unit: mm
  double object_distance{}, focal_length{};
  // Unit: mm
  double object_min_area_bounding_rect_max_len{}, object_min_area_bounding_rect_min_len{};
  // Unit: um  The physical size of a pixel on CMOS.
  double cell_size{};
  nh.param("object_distance", object_distance, 6439.);
  nh.param("cam_focus", focal_length, 8.);
  nh.param("object_min_area_bounding_rect_max_len", object_min_area_bounding_rect_max_len, 720.);
  nh.param("object_min_area_bounding_rect_min_len", object_min_area_bounding_rect_min_len,
           object_min_area_bounding_rect_max_len / 2.);
  nh.param("cmos_cell_size", cell_size, 4.8);
  NODELET_INFO_STREAM("Get object distance: " << object_distance << "mm");
  NODELET_INFO_STREAM("Get focal length: " << focal_length << "mm");
  NODELET_INFO_STREAM("Get object bounding rect max length: " << object_min_area_bounding_rect_max_len << "mm");
  NODELET_INFO_STREAM("Get object bounding rect min length: " << object_min_area_bounding_rect_min_len << "mm");
  ROS_ASSERT(object_distance != focal_length && focal_length > 1. && object_distance * focal_length > focal_length);
  //                    1     1     1
  // Gaussian formula: --- + --- = ---  (l', l, f > 0)
  //                    l'    l     f
  // l': image distance; l: object distance; f: focal length
  double image_distance = 1. / (1. / focal_length - 1. / object_distance);
  //          l'
  // \beta = ---
  //          l
  double beta = image_distance / object_distance;
  int image_max_len = static_cast<int>(object_min_area_bounding_rect_max_len * beta * 1000 / cell_size);
  int image_min_len = static_cast<int>(object_min_area_bounding_rect_min_len * beta * 1000 / cell_size);
  NODELET_INFO_STREAM("The max length of object on image = " << image_max_len);
  NODELET_INFO_STREAM("The min length of object on image = " << image_min_len);
  this->roi_->create(image_min_len * 5, image_max_len + 5);
}

void WindmillRecognizerNodelet::onInit()
{
  setRoiSize();
  ros::NodeHandle& nh = this->getPrivateNodeHandle();
  static ros::CallbackQueue my_queue;
  nh.setCallbackQueue(&my_queue);
  this->it_ = std::make_shared<image_transport::ImageTransport>(image_transport::ImageTransport(nh));
  static message_filters::Subscriber<sensor_msgs::Image> raw_img_sub(nh, "/galaxy_camera/image_raw", 1);
  static message_filters::Subscriber<sensor_msgs::Image> bin_img_sub(nh, "/windmill_vision/bin_image_raw", 1);
  this->roi_pub_ = it_->advertise("/windmill_vision/roi_img", 1);
#if ENABLE_VIRTUALIZATION
  this->debug_pub_ = it_->advertise("/windmill_vision/recognizer/debug", 1);
#endif
  my_thread_ = std::thread([this]() {
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(raw_img_sub, bin_img_sub, 10);
    sync.registerCallback(boost::bind(&WindmillRecognizerNodelet::callback, this, _1, _2));
    ros::SingleThreadedSpinner spinner;
    spinner.spin(&my_queue);
  });
  // !!! FORBIDDEN: Any code that prohibits onInit() from exiting, such as ros::spin().
}

void WindmillRecognizerNodelet::callback(const sensor_msgs::ImageConstPtr& raw_img,
                                         const sensor_msgs::ImageConstPtr& binary_img)
{
  roi_amount_ = 0;
  this->cv_img_const_ptr_ = cv_bridge::toCvShare(raw_img, "bgr8");
#if ENABLE_VIRTUALIZATION
  cv::Mat3b img_to_draw = cv_img_const_ptr_->image.clone();
  this->img_to_draw_ = &img_to_draw;
#endif
  auto bin_img_const_ptr = cv_bridge::toCvShare(binary_img, "mono8");
  cv::Mat1b bin_img = bin_img_const_ptr->image;
  locateNotTriggeredFan(bin_img);

  if (roi_amount_ > 0)
  {
    // TODO: make roi amount realistic
    *reinterpret_cast<int*>(roi_->data + static_cast<int>(RoiInfoEncodingProtocol::ROI_AMOUNT) * roi_->step) =
        roi_amount_;
    this->roi_pub_.publish(cv_bridge::CvImage(binary_img->header, "bgr8", *roi_).toImageMsg());
  }
#if ENABLE_VIRTUALIZATION
  this->debug_pub_.publish(cv_bridge::CvImage(binary_img->header, "bgr8", *img_to_draw_).toImageMsg());
#endif
}

void WindmillRecognizerNodelet::extractHammerRoi(int length)
{
  if (length > this->roi_->cols)
  {
    NODELET_ERROR("The length of the ROI will exceed the limit of roi_.");
    return;
  }
  float height = length / 2.;
  float width = length;
  cv::Size sz(static_cast<int>(width), static_cast<int>(height));
  cv::Point2f* src = hammer_description_.data();
  cv::Point2f dst[4] = { cv::Point2f(width, 0), cv::Point2f(width, height), cv::Point2f(0, height), cv::Point2f(0, 0) };
  // !!! FORBIDDEN: Using the following two lines will give a poor quality ROI.
  // cv::Mat trans = cv::getAffineTransform(dst, src);
  // cv::warpAffine(cv_img_const_ptr_->image, roi, trans, cv::Size(dist2, dist1), cv::WARP_INVERSE_MAP);

  cv::Mat transform = cv::getAffineTransform(src, dst);  // type: CV_64F
  cv::Mat transformed_concave_position =
      transform * cv::Matx31d(this->concave_position_.x, this->concave_position_.y, 1);
  if (!(transformed_concave_position.at<double>(0, 0) > 0 && transformed_concave_position.at<double>(0, 0) < width &&
        transformed_concave_position.at<double>(1, 0) > 0 && transformed_concave_position.at<double>(1, 0) < height))
  {
    NODELET_ERROR("The transformed concave_position is outside the roi!");
    return;
  }
  cv::Mat3b roi(sz);
  cv::warpAffine(cv_img_const_ptr_->image, roi, transform, sz);
  // To convert the position of the point from the ROI back to the Src, use cv::invertAffineTransform() to get the
  // inverse of the transform and apply it to the (x, y) of the ROI.
  // hammer_description_ is now useless and can be used to store other data.
  cv::invertAffineTransform(transform, transform);
  // roi will be transferred to the geometry analyzer, therefore the inverse transform can be encoded in the roi.
  // Remember that the hammer head is on the right side, so there is no need to worry about destroying the effective area.
  memcpy(roi.data + static_cast<int>(RoiInfoEncodingProtocol::INVERSE_TRANSFORM) * roi.step, transform.data,
         2 * 3 * sizeof(double));
  // Encode the transformed_concave_position in the roi.
  memcpy(roi.data + static_cast<int>(RoiInfoEncodingProtocol::TRANSFORMED_CONCAVE_POSITION) * roi.step,
         transformed_concave_position.data, 2 * sizeof(double));
  // Encode the size of roi in roi.
  int* ptr = reinterpret_cast<int*>(roi.data + static_cast<int>(RoiInfoEncodingProtocol::ACTUAL_ROI_SIZE) * roi.step);
  *ptr = sz.height;
  *(ptr + 1) = sz.width;
  // Encode the 'R' icon position and its confidence.
  ptr = reinterpret_cast<int*>(roi.data +
      static_cast<int>(RoiInfoEncodingProtocol::CENTRAL_R_ORIGIN_POSITION) * roi.step);
  *ptr = static_cast<int>(floor(r_icon_position_[0].x));
  *(ptr + 1) = static_cast<int>(floor(r_icon_position_[0].y));
  *(ptr + 2) = static_cast<int>(r_icon_position_[1].x);
  // Encode the rotation_inverse
  memcpy(roi.data + static_cast<int>(RoiInfoEncodingProtocol::INVERSE_ROTATION) * roi.step,
         rot_inverse_.data, 2 * 3 * sizeof(double));
  // Copy the roi to roi_. Finally, roi_ will be transferred to the geometry analyzer.
  roi.copyTo(cv::Mat(roi_->operator()(cv::Rect(cv::Point(0, 0), sz))));
  ++roi_amount_;

  if (fans_.size() >= 2 && fans_.size() <= 4 && flags_.flags_.condition())
  {
    flags_.flags_.onlyToAll();
    for (unsigned long i = 0; i < fans_.size(); ++i)
    {
      if(cv::norm(fans_[i][2] - this->r_icon_position_[0]) > cv::norm(fans_[i][0] - this->r_icon_position_[0]))
      {
        fans_[i][1] = -fans_[i][1];
      }

      cv::Point2f normal_vec{fans_[i][1].y, -fans_[i][1].x};
      cv::Point2f p1 = fans_[i][0] + fans_[i][1] * 1.05 / 2 + normal_vec  / 4;
      cv::Point2f p2 = fans_[i][0] + fans_[i][1] * 1.05 / 2 - normal_vec  / 4;
      cv::Point2f p3 = p2 - fans_[i][1];
      cv::Point2f p4 = p1 - fans_[i][1];
      cv::Point2f concave_position_of_hammer = fans_[i][0] + 8.4 * fans_[i][1] / 45;
      if ((p1 - concave_position_of_hammer).dot(p1 - concave_position_of_hammer) >
          (p3 - concave_position_of_hammer).dot(p3 - concave_position_of_hammer))
      {
        std::swap(p1, p3);
        std::swap(p2, p4);
      }
      cv::Point2f src_from_fan[4] = {p1, p2, p3, p4};

      cv::Mat transform_from_fan = cv::getAffineTransform(src_from_fan, dst);
      cv::Mat transformed_concave_position_of_fan =
          transform_from_fan * cv::Matx31d(concave_position_of_hammer.x, concave_position_of_hammer.y, 1);
      if (!(transformed_concave_position_of_fan.at<double>(0, 0) > 0
          && transformed_concave_position_of_fan.at<double>(0, 0) < width &&
          transformed_concave_position_of_fan.at<double>(1, 0) > 0
          && transformed_concave_position_of_fan.at<double>(1, 0) < height)) {
        NODELET_ERROR("The transformed concave_position is outside the roi!");
        return;
      }

      cv::warpAffine(cv_img_const_ptr_->image, roi, transform_from_fan, sz);
      cv::invertAffineTransform(transform_from_fan, transform_from_fan);

      memcpy(roi.data + static_cast<int>(RoiInfoEncodingProtocol::INVERSE_TRANSFORM) * roi.step,
             transform_from_fan.data, 2 * 3 * sizeof(double));

      memcpy(roi.data + static_cast<int>(RoiInfoEncodingProtocol::TRANSFORMED_CONCAVE_POSITION) * roi.step,
             transformed_concave_position_of_fan.data, 2 * sizeof(double));

      roi.copyTo(cv::Mat(roi_->operator()(cv::Rect(cv::Point(0, this->roi_->rows * (i + 1) / 5), sz))));
      ++roi_amount_;
    }
  }
}
}  // namespace rm_windmill_vision

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rm_windmill_vision::WindmillRecognizerNodelet, nodelet::Nodelet)