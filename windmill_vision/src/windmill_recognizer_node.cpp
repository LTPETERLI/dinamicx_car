////
//// Created by kaxns on 3/15/22.
////
//
//#include "windmill_dev/windmill_recognizer.h"
//
//namespace rm_windmill_vision
//{
//void WindmillRecognizerNode::execute()
//{
//  if (this->cv_img_const_ptr_ == nullptr)
//    return;
//  static cv::Mat3b input_img(cv_img_const_ptr_->image.size());
//  input_img = cv_img_const_ptr_->image.clone();
//#if ENABLE_VIRTUALIZATION
//  img_to_draw_ = &input_img;
//#endif
//  static cv::Mat1b bin_img(input_img.size());
//  if (color_processor_ != nullptr)
//  {
//    color_processor_->colorEnhance(input_img);
//    color_processor_->colorCvt(input_img, bin_img);
//  }
//  if (!locateNotTriggeredFan(bin_img)) {}
//#if ENABLE_VIRTUALIZATION
//  color_image_pub_.publish(cv_bridge::CvImage(cv_img_const_ptr_->header, "bgr8", input_img).toImageMsg());
//  bin_image_pub_.publish(cv_bridge::CvImage(cv_img_const_ptr_->header, "mono8", bin_img).toImageMsg());
//#endif
//}
//
//bool WindmillRecognizerNode::transferRoi(cv::Mat3b*& output_roi)
//{
//  if (this->roi_ != nullptr)
//  {
//    output_roi = this->roi_;
//    this->roi_ = nullptr;
//    return true;
//  }
//  return false;
//}
//
//void WindmillRecognizerNode::extractHammerRoi(int length)
//{
//  float height = length / 2.;
//  float width = length;
//  cv::Size sz(static_cast<int>(width), static_cast<int>(height));
//  cv::Point2f* src = hammer_description_.data();
//  cv::Point2f dst[4] = { cv::Point2f(width, 0), cv::Point2f(width, height), cv::Point2f(0, height), cv::Point2f(0, 0) };
//  // !!! FORBIDDEN: Using the following two lines will give a poor quality roi.
//  // cv::Mat trans = cv::getAffineTransform(dst, src);
//  // cv::warpAffine(cv_img_const_ptr_->image, roi, trans, cv::Size(dist2, dist1), cv::WARP_INVERSE_MAP);
//
//  cv::Mat transform = cv::getAffineTransform(src, dst);  // type: CV_64F
//  cv::Mat transformed_concave_position = transform * cv::Matx31d(this->concave_position_.x, this->concave_position_.y, 1);
//  if (roi_ == nullptr)
//    roi_ = new cv::Mat3b(sz);
//  cv::warpAffine(cv_img_const_ptr_->image, *roi_, transform, sz);
//  // To convert the position of the point from the ROI back to the Src, use cv::invertAffineTransform() to get the
//  // inverse of the transform and apply it to the (x, y) of the ROI.
//  // hammer_description_ is now useless and can be used to store other data.
//  cv::invertAffineTransform(transform, transform);
//  // roi_ will be transferred to the geometry analyzer, therefore the inverse transform can be encoded in the roi_.
//  // Remember that the hammer head is on the right side, so there is no need to worry about destroying the effective area.
//  memcpy(roi_->data, transform.data, 2 * 3 * sizeof(double));
//  // Encodes the transformed_concave_position in the roi_.
//  memcpy(roi_->data + roi_->step, transformed_concave_position.data, 2 * sizeof(double));
//#if ENABLE_VIRTUALIZATION
//  cv::Mat roi = (*img_to_draw_)(cv::Rect(cv::Point(150, 0), roi_->size()));
//  roi_->copyTo(roi);
//#endif
//}
//}  // namespace rm_windmill_vision