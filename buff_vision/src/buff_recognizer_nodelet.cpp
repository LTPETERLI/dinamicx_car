//
// Created by kaxns on 3/15/22.
//

#include "../include/buff_vision/buff_recognizer.h"
#include <string>

namespace rm_buff_vision {
void BuffRecognizerNodelet::onInit() {
  setRoiSize();
  ros::NodeHandle &nh = this->getPrivateNodeHandle();
  BuffRecognizerBase::init(nh);
  static ros::CallbackQueue my_queue;
  nh.setCallbackQueue(&my_queue);
  this->it_ = std::make_shared<image_transport::ImageTransport>(image_transport::ImageTransport(nh));
  static message_filters::Subscriber<sensor_msgs::Image> raw_img_sub(nh, "/galaxy_camera/image_raw", 1);
  static message_filters::Subscriber<sensor_msgs::Image> bin_img_sub(nh, "/buff_vision/bin_image_raw", 1);
  this->status_sub_ = nh.subscribe("/status", 1, &BuffRecognizerNodelet::statusCallback, this);
  this->roi_pub_ = it_->advertise("/buff_vision/roi_img", 1);
#if ENABLE_VIRTUALIZATION
  this->debug_pub_ = it_->advertise("/buff_vision/recognizer/debug", 1);
#endif
  my_thread_ = std::thread([this]() {
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(raw_img_sub, bin_img_sub, 10);
    sync.registerCallback(boost::bind(&BuffRecognizerNodelet::callback, this, _1, _2));
    ros::SingleThreadedSpinner spinner;
    spinner.spin(&my_queue);
  });
  // !!! FORBIDDEN: Any code that prohibits onInit() from exiting, such as ros::spin().

  prediction_pub_ = nh.advertise<rm_msgs::TargetDetectionArray>("/prediction", 1);

  if(!nh.getParam("hammer_min_area", hammer_min_area_))
    ROS_WARN("No hammer_min_area specified");
  if(!nh.getParam("hammer_max_area", hammer_max_area_))
    ROS_WARN("No hammer_max_area specified");
  if(!nh.getParam("hammer_min_rect_ratio", hammer_min_rect_ratio_))
    ROS_WARN("No hammer_min_rect_ratio specified");
  if(!nh.getParam("hammer_max_rect_ratio", hammer_max_rect_ratio_))
    ROS_WARN("No hammer_max_rect_ratio specified");
  if(!nh.getParam("min_white_ratio", min_white_ratio_))
    ROS_WARN("No min_white_ratio specified");
  if(!nh.getParam("max_white_ratio", max_white_ratio_))
    ROS_WARN("No max_white_ratio specified");
  if(!nh.getParam("r_icon_min_length", r_icon_min_length_))
    ROS_WARN("No r_icon_min_length specified");
  if(!nh.getParam("r_icon_max_length", r_icon_max_length_))
    ROS_WARN("No r_icon_max_length specified");
  if(!nh.getParam("r_icon_min_area", r_icon_min_area_))
    ROS_WARN("No r_icon_min_area specified");
  if(!nh.getParam("r_icon_max_area", r_icon_max_area_))
    ROS_WARN("No r_icon_max_area specified");
  if(!nh.getParam("r_icon_min_rect_ratio", r_icon_min_rect_ratio_))
    ROS_WARN("No r_icon_min_rect_ratio specified");
  if(!nh.getParam("r_icon_max_rect_ratio", r_icon_max_rect_ratio_))
    ROS_WARN("No r_icon_max_rect_ratio specified");
  if(!nh.getParam("min_rc_distance", min_rc_distance_))
    ROS_WARN("No min_rc_distance specified");
  if(!nh.getParam("max_rc_distance", max_rc_distance_))
    ROS_WARN("No max_rc_distance specified");

  if(!nh.getParam("big_rect_size", big_rect_size_))
    ROS_WARN("No big_rect_size specified");
  if(!nh.getParam("small_rect_size", small_rect_size_))
    ROS_WARN("No small_rect_size specified");
  if(!nh.getParam("intensity", intensity_))
    ROS_WARN("No intensity specified");
  if(!nh.getParam("small_rect_move", small_rect_move_))
    ROS_WARN("No small_rect_move specified");
}

inline void BuffRecognizerNodelet::setRoiSize() {
  ros::NodeHandle &nh = this->getPrivateNodeHandle();
  // Unit: mm
  double object_distance{}, focal_length{};
  // Unit: mm
  double object_min_area_bounding_rect_max_len{}, object_min_area_bounding_rect_min_len{};
  // Unit: um  The physical size of a pixel on CMOS.
  double cell_size{};
  nh.param("object_distance", object_distance, 6439.);
  nh.param("cam_focus", focal_length, 6.);
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

void BuffRecognizerNodelet::statusCallback(const rm_msgs::TargetDetectionArray &status)
{
  int32_t temp[1]{};
  for (const auto &detection : status.detections)
    memcpy(&temp[0], &detection.pose.orientation.y, sizeof(int32_t));

  armor_type_ = temp[0];
}

void BuffRecognizerNodelet::callback(const sensor_msgs::ImageConstPtr &raw_img,
                                     const sensor_msgs::ImageConstPtr &binary_img){
  if (!armor_type_)
    return;
  roi_amount_ = 0;
  this->sync_ = binary_img->header;
  this->cv_img_const_ptr_ = cv_bridge::toCvShare(raw_img, "bgr8");
#if ENABLE_VIRTUALIZATION
  cv::Mat3b img_to_draw = cv_img_const_ptr_->image.clone();
  this->img_to_draw_ = &img_to_draw;
#endif
  auto bin_img_const_ptr = cv_bridge::toCvShare(binary_img, "mono8");
  cv::Mat1b bin_img = bin_img_const_ptr->image;

//  if(take_photo_)
//  {
//    std::string str;
//    str = std::to_string(count_);
//    cv::Mat cv_img1;
//    cv_img_const_ptr_->image.copyTo(cv_img1);
//    cv::imwrite("/media/koonyu/KESU/buff_dataset/blue/"+str+".jpg",cv_img1);
//    count_++;
//  }

  img_center_ = cv::Point(img_to_draw_->cols / 2, img_to_draw_->rows / 2);
  locateNotTriggeredFan(bin_img);

#if ENABLE_VIRTUALIZATION
  if (roi_amount_ > 0) {
    // TODO: make roi amount realistic
    *reinterpret_cast<int *>(roi_->data + static_cast<int>(RoiInfoEncodingProtocol::ROI_AMOUNT) * roi_->step) =
        roi_amount_;
    this->roi_pub_.publish(cv_bridge::CvImage(binary_img->header, "bgr8", *roi_).toImageMsg());
  }
  else
  {
    prediction_target_.header.stamp = this->sync_.stamp;
    prediction_target_.header.seq = this->sync_.seq;
    prediction_pub_.publish(prediction_target_);
  }
#endif

#if ENABLE_VIRTUALIZATION
  this->debug_pub_.publish(cv_bridge::CvImage(binary_img->header, "bgr8", *img_to_draw_).toImageMsg());
#endif
}

void BuffRecognizerBase::init(const ros::NodeHandle &nh) {
  buff_recognizer_cfg_srv_ = std::make_unique<dynamic_reconfigure::Server<Config>>(nh);
  buff_recognizer_cfg_cb_ = boost::bind(&BuffRecognizerBase::reConfigCB, this, _1, _2);
  buff_recognizer_cfg_srv_->setCallback(buff_recognizer_cfg_cb_);
}

void BuffRecognizerBase::reConfigCB(Config &config, uint32_t level) {
  hammer_min_area_ = config.hammer_min_area;
  hammer_max_area_ = config.hammer_max_area;
  hammer_min_rect_ratio_ = config.hammer_min_rect_ratio;
  hammer_max_rect_ratio_ = config.hammer_max_rect_ratio;
  min_white_ratio_ = config.min_white_ratio;
  max_white_ratio_ = config.max_white_ratio;

  r_icon_min_area_ = config.r_icon_min_area;
  r_icon_max_area_ = config.r_icon_max_area;
  r_icon_min_length_ = config.r_icon_min_length;
  r_icon_max_length_ = config.r_icon_max_length;
  r_icon_min_rect_ratio_ = config.r_icon_min_rect_ratio;
  r_icon_max_rect_ratio_ = config.r_icon_max_rect_ratio;

  min_rc_distance_ = config.min_rc_distance;
  max_rc_distance_ = config.max_rc_distance;

  small_rect_size_ = config.small_rect_size;
  big_rect_size_ = config.big_rect_size;
  intensity_ = config.intensity;

  small_rect_move_ = config.small_rect_move;

//  take_photo_ = config.take_photo;
}

void BuffRecognizerNodelet::extractHammerRoi(int length)
{
  if (length > this->roi_->cols)
  {
    NODELET_ERROR("The length of the ROI will exceed the limit of roi_.");
    return;
  }
  float height = length / 2.;
  float width = length;
  cv::Size sz(static_cast<int>(width), static_cast<int>(height));
  std::array<cv::Point2f, 4> rect_points;
  hammer_.returnRectPoints(rect_points);
  cv::Point2f src[4] = {rect_points[0], rect_points[1], rect_points[2], rect_points[3]};
  cv::Point2f dst[4] = { cv::Point2f(width, 0), cv::Point2f(width, height), cv::Point2f(0, height), cv::Point2f(0, 0) };
  // !!! FORBIDDEN: Using the following two lines will give a poor quality ROI.
  // cv::Mat trans = cv::getAffineTransform(dst, src);
  // cv::warpAffine(cv_img_const_ptr_->image, roi, trans, cv::Size(dist2, dist1), cv::WARP_INVERSE_MAP);

  cv::Mat transform = cv::getAffineTransform(src, dst);  // type: CV_64F
  cv::Point2f x_target;
  hammer_.returnCenter(x_target);
  cv::Mat transformed_concave_position =
      transform * cv::Matx31d(x_target.x, x_target.y, 1);
  if (!(transformed_concave_position.at<double>(0, 0) > 0 && transformed_concave_position.at<double>(0, 0) < width &&
      transformed_concave_position.at<double>(1, 0) > 0 && transformed_concave_position.at<double>(1, 0) < height))
  {
    NODELET_ERROR("The transformed x_target is outside the roi!");
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
  // Encode the size of roi in roi.
  int* ptr = reinterpret_cast<int*>(roi.data + static_cast<int>(RoiInfoEncodingProtocol::ACTUAL_ROI_SIZE) * roi.step);
  *ptr = sz.height;
  *(ptr + 1) = sz.width;
  // Encode the 'R' icon position.
  ptr = reinterpret_cast<int*>(roi.data +
      static_cast<int>(RoiInfoEncodingProtocol::CENTRAL_R_ORIGIN_POSITION) * roi.step);
  *ptr = static_cast<int>(r_center_.x);
  *(ptr + 1) = static_cast<int>(r_center_.y);
  // Encode the center of original frame
  ptr = reinterpret_cast<int*>(roi.data +
      static_cast<int>(RoiInfoEncodingProtocol::ORIGINAL_FRAME_CENTER_POSITION) * roi.step);
  *ptr = static_cast<int>(img_center_.x);
  *(ptr + 1) = static_cast<int>(img_center_.y);
  // Copy the roi to roi_. Finally, roi_ will be transferred to the geometry analyzer.
  roi.copyTo(cv::Mat(roi_->operator()(cv::Rect(cv::Point(0, 0), sz))));
  ++roi_amount_;
}

//int BuffRecognizerNodelet::count_ = 0;
}  // namespace rm_buff_vision

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rm_buff_vision::BuffRecognizerNodelet, nodelet::Nodelet)