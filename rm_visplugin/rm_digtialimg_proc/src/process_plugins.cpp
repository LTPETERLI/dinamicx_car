//
// Created by yichenghe on 2021/11/13.
//

#include "../include/process_plugins.h"
#include <pluginlib/class_list_macros.h>
#include <ros/callback_queue.h>

using namespace cv;
using namespace std;
// using namespace InferenceEngine;

PLUGINLIB_EXPORT_CLASS(rm_digtialimg_proc::Processor, nodelet::Nodelet)

namespace rm_digtialimg_proc
{
void Processor::onInit()
{
  ros::NodeHandle& nh = getMTPrivateNodeHandle();
//  this->status_change_srv_ = nh.advertiseService("status_switch", &Processor::changeStatusCB, this);
  static ros::CallbackQueue my_queue;
  nh.setCallbackQueue(&my_queue);
  initialize(nh);
  my_thread_ = std::thread([]() {
    ros::SingleThreadedSpinner spinner;
    spinner.spin(&my_queue);
  });
}

void Processor::initialize(ros::NodeHandle& nh)
{
  nh_ = ros::NodeHandle(nh, "digtialimg_proc");
  ROS_INFO("to read model");
  net_ = cv::dnn::readNetFromONNX(ros::package::getPath("rm_digtialimg_proc") + "/config/model/mlp.onnx");
  ROS_INFO("reding model done");
  class_num_ = 9;
  /*
//  xml_path_ = ros::package::getPath("rm_digtialimg_proc") + "/config/model/MobileNetV21_3.xml";
//  bin_path_ = ros::package::getPath("rm_digtialimg_proc") + "/config/model/MobileNetV21_3.bin";
//  Core ie;
//  cnn_network_ = ie.ReadNetwork(xml_path_, bin_path_);
//  // setting input information of model
//  InputsDataMap input_info(cnn_network_.getInputsInfo());
//  InputInfo::Ptr& input = input_info.begin()->second;
//  input_name_ = input_info.begin()->first;
//  input->setPrecision(Precision::FP32);
//  input->getInputData()->setLayout(Layout::NCHW);
//  ICNNNetwork::InputShapes input_shapes = cnn_network_.getInputShapes();
//  cnn_network_.reshape(input_shapes);
//  //setting output information
//  output_info_ = OutputsDataMap(cnn_network_.getOutputsInfo());
//  output_info_.begin()->second->setPrecision(Precision::FP32);
//  output_info_.begin()->second->setLayout(InferenceEngine::Layout::NC);
//  network_ = ie.LoadNetwork(cnn_network_, "CPU");
//  infer_request_ = network_.CreateInferRequest();
*/
  result_.first = 0;
  result_.second = 0.0;

  /*
    // weibull model
  //  vector<double> coefficient;
  //  if (!nh.getParam("weibull_coefficient/class1_weibull", coefficient))
  //    ROS_WARN("No weibull coefficient specified");
  //  weibull_model_.push_back(coefficient);
  //  if (!nh.getParam("weibull_coefficient/class2_weibull", coefficient))
  //    ROS_WARN("No weibull coefficient specified");
  //  weibull_model_.push_back(coefficient);
  //  if (!nh.getParam("weibull_coefficient/class3_weibull", coefficient))
  //    ROS_WARN("No weibull coefficient specified");
  //  weibull_model_.push_back(coefficient);
  //  if (!nh.getParam("weibull_coefficient/class4_weibull", coefficient))
  //    ROS_WARN("No weibull coefficient specified");
  //  weibull_model_.push_back(coefficient);
  //  if (!nh.getParam("weibull_coefficient/class5_weibull", coefficient))
  //    ROS_WARN("No weibull coefficient specified");
  //  weibull_model_.push_back(coefficient);
  //  if (!nh.getParam("weibull_coefficient/class6_weibull", coefficient))
  //    ROS_WARN("No weibull coefficient specified");
  //  weibull_model_.push_back(coefficient);
  //  if (!nh.getParam("weibull_coefficient/class7_weibull", coefficient))
  //    ROS_WARN("No weibull coefficient specified");
  //  weibull_model_.push_back(coefficient);
  //  if (!nh.getParam("weibull_coefficient/class8_weibull", coefficient))
  //    ROS_WARN("No weibull coefficient specified");
  //  weibull_model_.push_back(coefficient);
  //
  //  vector<double> mean;
  //  if (!nh.getParam("mean_vector/class1_mean", mean))
  //    ROS_WARN("No weibull coefficient specified");
  //  weibull_mean_.push_back(mean);
  //  if (!nh.getParam("mean_vector/class2_mean", mean))
  //    ROS_WARN("No weibull coefficient specified");
  //  weibull_mean_.push_back(mean);
  //  if (!nh.getParam("mean_vector/class3_mean", mean))
  //    ROS_WARN("No weibull coefficient specified");
  //  weibull_mean_.push_back(mean);
  //  if (!nh.getParam("mean_vector/class4_mean", mean))
  //    ROS_WARN("No weibull coefficient specified");
  //  weibull_mean_.push_back(mean);
  //  if (!nh.getParam("mean_vector/class5_mean", mean))
  //    ROS_WARN("No weibull coefficient specified");
  //  weibull_mean_.push_back(mean);
  //  if (!nh.getParam("mean_vector/class6_mean", mean))
  //    ROS_WARN("No weibull coefficient specified");
  //  weibull_mean_.push_back(mean);
  //  if (!nh.getParam("mean_vector/class7_mean", mean))
  //    ROS_WARN("No weibull coefficient specified");
  //  weibull_mean_.push_back(mean);
  //  if (!nh.getParam("mean_vector/class8_mean", mean))
  //    ROS_WARN("No weibull coefficient specified");
  //  weibull_mean_.push_back(mean);
  //
  //    ROS_INFO("weibull");
  */
  auto armor_params_init = [this, &nh]() {
    ROS_INFO("reading armor param");
    //    bar_br_thresh_ = nh.param("bar_br_thresh", decltype(bar_br_thresh_){});
    /// using to define 'Point2d' in id-classification
    bar_length_in_warp_ = nh.param("bar_length_in_warp", decltype(bar_length_in_warp_){});
    warp_height_ = nh.param("warp_height", decltype(warp_height_){});
    top_light_y_ = nh.param("top_light_y", decltype(top_light_y_){});
    roi_width_ = nh.param("roi_width", decltype(roi_width_){});
    roi_height_ = nh.param("roi_height", decltype(roi_height_){});

    is_bar_debug_ = nh.param("is_bar_debug", decltype(is_bar_debug_){});
    is_armor_debug_ = nh.param("is_armor_debug", decltype(is_armor_debug_){});
    is_classfy_debug_ = nh.param("is_classfy_debug", decltype(is_classfy_debug_){});
    select_bar_ = nh.param("select_bar", decltype(select_bar_){});
    max_angle_ = nh.param("max_angle", decltype(max_angle_){});
    min_lw_ratio_ = nh.param("min_lw_ratio", decltype(min_lw_ratio_){});
    max_lw_ratio_ = nh.param("max_lw_ratio", decltype(max_lw_ratio_){});
    min_pixel_contained_ratio_ = nh.param("min_pixel_contained_ratio", decltype(min_pixel_contained_ratio_){});
    max_bars_ratio_ = nh.param("max_bars_ratio", decltype(max_bars_ratio_){});
    max_bars_distance_ = nh.param("max_bars_distance", decltype(max_bars_distance_){});
    min_bars_distance_ = nh.param("min_bars_distance", decltype(min_bars_distance_){});
    max_bars_angle_ = nh.param("max_bars_angle", decltype(max_bars_angle_){});
    max_bars_y_dis_ = nh.param("max_bars_y_dis", decltype(max_bars_y_dis_){});
    select_by_last_ = nh.param("select_by_last", decltype(select_by_last_){});
    large_armor_ratio_ = nh.param("large_armor_ratio", decltype(large_armor_ratio_){});

    warp_thresh_ = nh.param("warp_thresh", decltype(warp_thresh_){});
    gamma_ = nh.param("gamma", decltype(gamma_){});
    contrast_alpha_ = nh.param("contrast_alpha", decltype(contrast_alpha_){});
    contrast_beta_ = nh.param("contrast_beta", decltype(contrast_beta_){});
    gamma_y_ = nh.param("gamma_y", decltype(gamma_y_){});
    rotate_ = nh.param("rotate", decltype(rotate_){});

    expand_ratio_ = nh.param("expand_ratio", decltype(expand_ratio_){});
    id_confidence_ = nh.param("id_confidence", decltype(id_confidence_){});
    input_shape_ = nh.param("input_shape", decltype(input_shape_){});
    use_id_cls_ = nh.param("use_id_cls", decltype(use_id_cls_){});
    is_large_armor_store_ = nh.param("is_large_armor_store", decltype(is_large_armor_store_){});
    min_id_white_ratio_ = nh.param("min_id_white_ratio", decltype(min_id_white_ratio_){});
    max_id_white_ratio_ = nh.param("max_id_white_ratio", decltype(max_id_white_ratio_){});
    ROS_INFO("Armor params reading done");
  };
  auto pre_process_params_init = [this, &nh]() {
    ROS_INFO("reading pre-process param");
    target_is_red_ = nh.param("target_color", decltype(target_is_red_){});
    preprocess_method_ = nh.param("preprocess_method", decltype(preprocess_method_){});

    red_h_min_low_ = nh.param("red_h_min_low", decltype(red_h_min_low_){});
    red_h_max_low_ = nh.param("red_h_max_low", decltype(red_h_max_low_){});
    red_h_min_high_ = nh.param("red_h_min_high", decltype(red_h_min_high_){});
    red_h_max_high_ = nh.param("red_h_max_high", decltype(red_h_max_high_){});
    red_s_min_ = nh.param("red_s_min", decltype(red_s_min_){});
    red_s_max_ = nh.param("red_s_max", decltype(red_s_max_){});
    red_v_min_ = nh.param("red_v_min", decltype(red_v_min_){});
    red_v_max_ = nh.param("red_v_max", decltype(red_v_max_){});

    blue_h_min_ = nh.param("blue_h_min", decltype(blue_h_min_){});
    blue_h_max_ = nh.param("blue_h_max", decltype(blue_h_max_){});
    blue_s_min_ = nh.param("blue_s_min", decltype(blue_s_min_){});
    blue_s_max_ = nh.param("blue_s_max", decltype(blue_s_max_){});
    blue_v_min_ = nh.param("blue_v_min", decltype(blue_v_min_){});
    blue_v_max_ = nh.param("blue_v_max", decltype(blue_v_max_){});

    binary_thresh_ = nh.param("binary_thresh", decltype(binary_thresh_){});
    morph_type_ = nh.param("morph_type", decltype(morph_type_){});
    binary_element_ = nh.param("binary_element", decltype(binary_element_){});
    ROS_INFO("pre-processing param reding done");
  };
  auto dataset_params_init = [this, &nh]() {
    ROS_INFO("reading dataset config");
    dataset_path_ = nh.param("dataset_path", decltype(dataset_path_){});
    if (*(this->dataset_path_.end() - 1) != '/')
    {
      this->dataset_path_ += "/";
    }
    make_dataset_ = nh.param("make_dataset", decltype(make_dataset_){});
    dataset_split_ = nh.param("dataset_split", decltype(dataset_split_){});
    image_id_ = nh.param("image_id", decltype(image_id_){});
    continute_ = nh.param("continute", decltype(continute_){});
    image_num_ = nh.param("image_num", decltype(image_num_){});
    ROS_INFO("dataset config reding done");
  };
  armor_params_init();
  pre_process_params_init();
  dataset_params_init();

  armor_cfg_srv_ = new dynamic_reconfigure::Server<ArmorConfig>(ros::NodeHandle(nh_, "armor_condition"));
  armor_cfg_cb_ = boost::bind(&Processor::armorconfigCB, this, _1, _2);
  armor_cfg_srv_->setCallback(armor_cfg_cb_);

  preprocess_cfg_srv_ = new dynamic_reconfigure::Server<PreprocessConfig>(ros::NodeHandle(nh_, "preprocess_condition"));
  preprocess_cfg_cb_ = boost::bind(&Processor::preProcessconfigCB, this, _1, _2);
  preprocess_cfg_srv_->setCallback(preprocess_cfg_cb_);

  draw_cfg_srv_ = new dynamic_reconfigure::Server<DrawConfig>(ros::NodeHandle(nh_, "draw_condition"));
  draw_cfg_cb_ = boost::bind(&Processor::drawconfigCB, this, _1, _2);
  draw_cfg_srv_->setCallback(draw_cfg_cb_);

  make_dataset_cfg_srv_ = new dynamic_reconfigure::Server<MakedatasetConfig>(ros::NodeHandle(nh_, "dataset_condition"));
  make_dataset_cfg_cb_ = boost::bind(&Processor::datasetconfigCB, this, _1, _2);
  make_dataset_cfg_srv_->setCallback(make_dataset_cfg_cb_);

  it_ = make_shared<image_transport::ImageTransport>(nh_);
  image_pub_ = it_->advertise("debug_image", 1);
  cam_sub_ = it_->subscribeCamera("/galaxy_camera/image_raw", 1, &Processor::callback, this);
  target_pub_ = nh.advertise<decltype(target_array_)>("/processor/result_msg", 1);
}

void Processor::armorconfigCB(ArmorConfig& config, uint32_t level)
{
  if (!armor_dynamic_reconfig_initialized_)
  {
    // config.bar_br_thresh = bar_br_thresh_;
    config.is_bar_debug = is_bar_debug_;
    config.is_classfy_debug = is_classfy_debug_;
    config.is_armor_debug = is_armor_debug_;
    config.select_bar = select_bar_;
    config.max_angle = max_angle_;
    config.min_lw_ratio = min_lw_ratio_;
    config.max_lw_ratio = max_lw_ratio_;
    config.min_pixel_contained_ratio = min_pixel_contained_ratio_;
    config.max_bars_ratio = max_bars_ratio_;
    config.max_bars_distance = max_bars_distance_;
    config.min_bars_distance = min_bars_distance_;
    config.max_bars_angle = max_bars_angle_;
    config.max_bars_y_dis = max_bars_y_dis_;
    config.select_by_last = select_by_last_;

    config.warp_thresh = warp_thresh_;
    config.gamma = gamma_;
    config.contrast_alpha = contrast_alpha_;
    config.contrast_beta = contrast_beta_;
    config.gamma_y = gamma_y_;
    config.rotate = rotate_;

    config.id_confidence = id_confidence_;
    config.use_id_cls = use_id_cls_;
    config.is_large_armor_store = is_large_armor_store_;
    config.min_id_white_ratio = min_id_white_ratio_;
    config.max_id_white_ratio = max_id_white_ratio_;
    armor_dynamic_reconfig_initialized_ = true;
  }

  // bar_br_thresh_ = config.bar_br_thresh;
  is_bar_debug_ = config.is_bar_debug;
  is_armor_debug_ = config.is_armor_debug;
  is_classfy_debug_ = config.is_classfy_debug;
  select_bar_ = config.select_bar;
  max_angle_ = config.max_angle;
  min_lw_ratio_ = config.min_lw_ratio;
  max_lw_ratio_ = config.max_lw_ratio;
  min_pixel_contained_ratio_ = config.min_pixel_contained_ratio;
  max_bars_ratio_ = config.max_bars_ratio;
  max_bars_distance_ = config.max_bars_distance;
  min_bars_distance_ = config.min_bars_distance;
  max_bars_angle_ = config.max_bars_angle;
  max_bars_y_dis_ = config.max_bars_y_dis;
  select_by_last_ = config.select_by_last;

  warp_thresh_ = config.warp_thresh;
  gamma_ = config.gamma;
  contrast_alpha_ = config.contrast_alpha;
  contrast_beta_ = config.contrast_beta;
  gamma_y_ = config.gamma_y;
  uchar* p = look_up_table_.ptr();
  for (int i = 0; i < 256; ++i)
      p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma_y_) * 255.0);

  rotate_ = config.rotate;

  id_confidence_ = config.id_confidence;
  use_id_cls_ = config.use_id_cls;
  is_large_armor_store_ = config.is_large_armor_store;
  min_id_white_ratio_ = config.min_id_white_ratio;
  max_id_white_ratio_ = config.max_id_white_ratio;
}

void Processor::preProcessconfigCB(PreprocessConfig& config, uint32_t level)
{
  if (!pre_process_dynamic_reconfig_initialized_)
  {
    config.target_color = target_is_red_;
    config.preprocess_method = preprocess_method_;

    config.red_h_min_low = red_h_min_low_;
    config.red_h_max_low = red_h_max_low_;
    config.red_h_min_high = red_h_min_high_;
    config.red_h_max_high = red_h_max_high_;
    config.red_s_min = red_s_min_;
    config.red_s_max = red_s_max_;
    config.red_v_min = red_v_min_;
    config.red_v_max = red_v_max_;

    config.blue_h_min = blue_h_min_;
    config.blue_h_max = blue_h_max_;
    config.blue_s_min = blue_s_min_;
    config.blue_s_max = blue_s_max_;
    config.blue_v_min = blue_v_min_;
    config.blue_v_max = blue_v_max_;

    config.binary_thresh = binary_thresh_;

    config.morph_type = morph_type_;
    config.binary_element = binary_element_;
    pre_process_dynamic_reconfig_initialized_ = true;
  }

  target_is_red_ = config.target_color;
  preprocess_method_ = config.preprocess_method;

  red_h_min_low_ = config.red_h_min_low;
  red_h_max_low_ = config.red_h_max_low;
  red_h_min_high_ = config.red_h_min_high;
  red_h_max_high_ = config.red_h_max_high;
  red_s_min_ = config.red_s_min;
  red_s_max_ = config.red_s_max;
  red_v_min_ = config.red_v_min;
  red_v_max_ = config.red_v_max;

  blue_h_min_ = config.blue_h_min;
  blue_h_max_ = config.blue_h_max;
  blue_s_min_ = config.blue_s_min;
  blue_s_max_ = config.blue_s_max;
  blue_v_min_ = config.blue_v_min;
  blue_v_max_ = config.blue_v_max;

  binary_thresh_ = config.binary_thresh;

  morph_type_ = config.morph_type;
  binary_element_ = config.binary_element;
  if (config.binary_element % 2)
    binary_element_ += 1;
}

void Processor::drawconfigCB(DrawConfig& config, uint32_t level)
{
  draw_type_ = DrawImage(config.draw_type);
  line_width_ = config.line_width;
}

void Processor::datasetconfigCB(MakedatasetConfig& config, uint32_t level)
{
  make_dataset_ = config.make_dataset;
  dataset_split_ = config.dataset_split;
  image_id_ = config.image_id;
  continute_ = config.continute;
  image_num_ = config.image_num;
  this->collected_completely_ = false;
  this->image_index_ = 0;
}

Mat Processor::setElement()
{
  return getStructuringElement(MORPH_ELLIPSE, Size(binary_element_, binary_element_), Point(-1, -1));
}

void Processor::hsvToBinary()
{
  Mat hsv_image;

  cvtColor(this->raw_image_, hsv_image, COLOR_BGR2HSV);
  if (target_is_red_ == 1)
  {
    Mat h_binary_low, h_binary_high;
    inRange(hsv_image, Scalar(red_h_min_low_, red_s_min_, red_v_min_), Scalar(red_h_max_low_, red_s_max_, red_v_max_),
            h_binary_low);
    inRange(hsv_image, Scalar(red_h_min_high_, red_s_min_, red_v_min_), Scalar(red_h_max_high_, red_s_max_, red_v_max_),
            h_binary_high);
    bitwise_or(h_binary_low, h_binary_high, binary_image_);
  }
  else
  {
    inRange(hsv_image, Scalar(blue_h_min_, blue_s_min_, blue_v_min_), Scalar(blue_h_max_, blue_s_max_, blue_v_max_),
            binary_image_);
  }
}

// RGB(single channel)
void Processor::bgrToBinary()
{
  vector<Mat> channels;

  split(this->raw_image_, channels);
  if (target_is_red_ == 1)
    binary_image_ = channels[2] - channels[0];
  else
    binary_image_ = channels[0] - channels[2];
  threshold(binary_image_, binary_image_, binary_thresh_, 255, cv::THRESH_BINARY);
}

void Processor::imageProcess(cv_bridge::CvImagePtr& cv_image)
{
  Mat element = setElement();
  cv_image->image.copyTo(raw_image_);
  if (gamma_)
      cv::LUT(raw_image_,look_up_table_,raw_image_);
  switch (preprocess_method_)
  {
    case PreProcessMethod::HSV:
    {
      hsvToBinary();
      break;
    }
    case PreProcessMethod::SINGLE_CHANNEL:
    {
      bgrToBinary();
      break;
    }
  }
  if (morph_type_ == 0)
    morphologyEx(binary_image_, morpro_image_, MORPH_ERODE, element);
  else if (morph_type_ == 1)
    morphologyEx(binary_image_, morpro_image_, MORPH_DILATE, element);
  else if (morph_type_ == 2)
    morphologyEx(binary_image_, morpro_image_, MORPH_OPEN, element);
  else if (morph_type_ == 3)
    morphologyEx(binary_image_, morpro_image_, MORPH_CLOSE, element);
  else if (morph_type_ == 4)
    morphologyEx(binary_image_, morpro_image_, MORPH_GRADIENT, element);
  else if (morph_type_ == 5)
    morphologyEx(binary_image_, morpro_image_, MORPH_TOPHAT, element);
  else if (morph_type_ == 6)
    morphologyEx(binary_image_, morpro_image_, MORPH_BLACKHAT, element);
  else if (morph_type_ == 7)
    morphologyEx(binary_image_, morpro_image_, MORPH_HITMISS, element);
  else if (morph_type_ == 8)
    binary_image_.copyTo(morpro_image_);
}
/*
ArmorColor Processor::getBarColor(const cv::RotatedRect& rect)
{
  Point2f rect_point[4];
  vector<Point2f> sort_point;
  int br_x;
  int br_y;
  rect.points(rect_point);
  for (const auto& i : rect_point)
    sort_point.emplace_back(i);
  sort(sort_point.begin(), sort_point.end(), [](Point2f& p1, Point2f& p2) { return p1.x < p2.x; });
  Rect roi_rect;
  roi_rect.x = sort_point[0].x;
  br_x = sort_point[3].x;
  sort(sort_point.begin(), sort_point.end(), [](Point2f& p1, Point2f& p2) { return p1.y < p2.y; });
  roi_rect.y = sort_point[0].y;
  br_y = sort_point[3].y;
  roi_rect.height = br_y - roi_rect.y;
  roi_rect.width = br_x - roi_rect.x;
  roi_rect &= Rect(0, 0, raw_image_.cols, raw_image_.rows);
  Mat bar_roi = raw_image_(roi_rect);

  Scalar result = cv::mean(bar_roi);
  if (abs(result(0) - result(2)) > bar_br_thresh_)
  {
    if (result(0) > result(2))
      return ArmorColor(1);
    else
      return ArmorColor(0);
  }
  else
    return ArmorColor(2);
}
*/
bool Processor::isValidBar(const Bar& bar)
{
  if (is_bar_debug_)
  {
    ROS_INFO("angle: %d > %lf", max_angle_, bar.angle_);
    ROS_INFO("lw_ratio: %lf > %lf", max_lw_ratio_, bar.lw_ratio_);
    ROS_INFO("pixel_contained_ratio: %lf < %lf", min_pixel_contained_ratio_, bar.pixel_contained_ratio_);
    ROS_INFO("/////////////////////");
  }

  if (fabs(bar.angle_) > max_angle_)
    return false;

  if (bar.lw_ratio_ >= max_lw_ratio_)
    return false;

  if (bar.lw_ratio_ <= min_lw_ratio_)
    return false;

  if (bar.pixel_contained_ratio_ < min_pixel_contained_ratio_)
    return false;

  return true;
}

bool Processor::isValidArmor(Bar& left_bar, Bar& right_bar)
{
  double distance = pow(pow((right_bar.center_point_.x - left_bar.center_point_.x), 2) +
                            pow((right_bar.center_point_.y - left_bar.center_point_.y), 2),
                        0.5);

  double bars_length = left_bar.length_len_ + right_bar.length_len_;

  if (is_armor_debug_)
  {
    ROS_INFO("max_bars_ratio: %lf > %lf", max_bars_ratio_,
             max(left_bar.length_len_, right_bar.length_len_) / min(left_bar.length_len_, right_bar.length_len_));
    ROS_INFO("distance: %lf > %lf > %lf ", bars_length * max_bars_distance_, distance, bars_length * min_bars_distance_);
    ROS_INFO("left_bar: (%f, %f)", left_bar.center_point_.x, left_bar.center_point_.y);
    ROS_INFO("left_bar: (%f, %f)", right_bar.center_point_.x, right_bar.center_point_.y);
    ROS_INFO("max_bars_angle: %lf > %lf", max_bars_angle_, fabs(left_bar.angle_ - right_bar.angle_));
    ROS_INFO("left_bar_angle: %f", left_bar.angle_);
    ROS_INFO("right_bar_angle: %f", right_bar.angle_);
    ROS_INFO("max_bars_y_dis: %lf > %lf", max_bars_y_dis_,
             fabs(left_bar.center_point_.y - right_bar.center_point_.y) / (bars_length * 0.5));
    ROS_INFO("////////////////////////////////////////////");
  }

  if ((max(left_bar.length_len_, right_bar.length_len_) / min(left_bar.length_len_, right_bar.length_len_) >
       max_bars_ratio_) ||
      (distance <= bars_length * min_bars_distance_ || distance >= bars_length * max_bars_distance_) ||
      (fabs(left_bar.angle_ - right_bar.angle_) > max_bars_angle_))
  {
    return false;
  }

  Armor armor(left_bar, right_bar);
  is_large_armor_ = isLargeArmor(armor);
  if (armor.parallel_dist_ / (bars_length * 0.5) > max_bars_y_dis_)
    return false;
  if (select_by_last_)
  {
    for (auto& last_frame_armor : last_frame_armors_)
    {
      if (pow(pow((armor.center_.x - last_frame_armor.center_.x), 2) +
                  pow((armor.center_.y - last_frame_armor.center_.y), 2),
              0.5) < left_bar.length_len_)
      {
        result_.first = last_frame_armor.id_;
        result_.second = last_frame_armor.confidence_ * 1.5;
        negative_confidence_ = last_frame_armor.negative_confidence_ * 0.5;
        return true;
      }
    }
  }
  //  if (this->make_dataset_ && (armors_.size() != 1))
  //  {
  //    makeDigitalDataset(armor);
  //  }
  /// '* expand ratio' causes stablity loss
  //  armor.bars_inter_4points_[0].y += (armor.bar_left_->length_len_) * expand_ratio_[0];
  //  armor.bars_inter_4points_[1].y -= (armor.bar_left_->length_len_) * expand_ratio_[1];
  //  armor.bars_inter_4points_[2].y -= (armor.bar_left_->length_len_) * expand_ratio_[1];
  //  armor.bars_inter_4points_[3].y += (armor.bar_left_->length_len_) * expand_ratio_[0];

  if (use_id_cls_)
  {
    Mat image;
    if (!warp(armor.bars_inter_4points_, is_large_armor_, image))
      return false;
    /// return result id
    result_.first = fcRecognition(image);
    /// use armor_size_store(only for number of 3,4,5)
    if (result_.second >= id_confidence_) {
        if (is_large_armor_store_) {
            if (
            ///*hero, base is large *///
            ((result_.first == 1 || result_.first == 8) && !is_large_armor_) ||
            ///*engine , outpost and guard is small*///
            ((result_.first == 2 || result_.first == 6 || result_.first == 7) && is_large_armor_))
                return false;
            if (result_.first == 3 || result_.first == 4 || result_.first == 5) {
                // if temp higher than 100 or lower than -100 then keep the status all the game
                if (is_large_armor_ && armor_temp_[result_.first] < 400)
                    armor_temp_[result_.first]++;
                else if (!is_large_armor_ && armor_temp_[result_.first] > -400)
                    armor_temp_[result_.first]--;
                ///*the real-time conflicts with the temp*///
                if (armor_temp_[result_.first] > 0 && !is_large_armor_)
                    return false;
                else if (armor_temp_[result_.first] <= 0 && is_large_armor_)
                    return false;
            }
        }
        else
            memset(armor_temp_, 0, sizeof(armor_temp_));
    }

    if (result_.first == 9)
    {
      return false;  /// trash data
    }
    /// display the confidence
    if (is_classfy_debug_)
        ROS_INFO("classfy result:%d, confidence:%f, negative_confidence:%f", result_.first, result_.second,
                 negative_confidence_);
    return result_.second >= id_confidence_;
  }
  return true;
}

bool Processor::warp(const vector<Point2d>& points, bool is_large, Mat& dst)
{
  Mat gray_image;
  cvtColor(raw_image_, gray_image, cv::COLOR_BGR2GRAY);
  std::vector<Point2d> warp_points(4);
  /// using to define 'Point2d'
  bottom_light_y_ = top_light_y_ + bar_length_in_warp_;
  ///@Note define bar_length = 12
  ///@Note 19 : (warp_height - bar_length)/2 + bar_length
  ///@Note 7 : ( warp_height-bar_length) / 2
  ///@Note deep learning model : 20 width * 28 height
  /// large warp width
  if (is_large)
  {
    warp_width_ = 54;
  }
  /// small warp width
  if (!is_large)
  {
    warp_width_ = 32;
  }
  warp_points[0] = { 0, bottom_light_y_ };
  warp_points[1] = { 0, top_light_y_ };
  warp_points[2] = { double(warp_width_ - 1), top_light_y_ };
  warp_points[3] = { double(warp_width_ - 1), bottom_light_y_ };
  /* before
  //    warp_points[0] = { 0, 19 };
  //    warp_points[1] = { 0, 7 };
  //    warp_points[2] = { 24, 7 };
  //    warp_points[3] = { 24, 19 };
  */
  Mat warp_mat = findHomography(points, warp_points, cv::RANSAC);
  if (warp_mat.rows == 0 || warp_mat.cols == 0)
    return -1;
  try
  {
    warpPerspective(gray_image, dst, warp_mat, cv::Size(warp_width_, warp_height_));
    //    dst = dst(cv::Rect(cv::Point(2, 0), cv::Size(20,28)));
    dst = dst(cv::Rect(cv::Point((warp_width_ - 20) / 2, 0), cv::Size(roi_width_, roi_height_)));
  }
  catch (cv::Exception& e)
  {
    ROS_ERROR("warp err occurred !");
    ROS_ERROR("cv_exception: %s", e.what());
    return -1;
  }
  cv::normalize(dst, dst, 0, 255, cv::NORM_MINMAX);
  //  if (gamma_)
  //  {
  //    // gamma
  //    cv::Mat look_up_table(1, 256, CV_8U);
  //    uchar* p = look_up_table.ptr();
  //    for (int i = 0; i < 256; ++i)
  //      p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma_y_) * 255.0);
  //    cv::LUT(dst, look_up_table, dst);
  //  }
  //  else
  //  {
  //    dst.convertTo(dst, dst.type(), contrast_alpha_, contrast_beta_);
  //  }
  cv::threshold(dst, dst, warp_thresh_, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  if (rotate_)
      cv::flip(dst, dst, -1);
  //  morphologyEx(dst, dst, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(5, 5), Point(-1, -1)));
  //  if (rotate_)
  //  {
  //    cv::rotate(dst, dst, cv::ROTATE_180);
  //  }
  return true;
}

void Processor::armorPrograming(const vector<vector<int>>& armor_programme)
{
  vector<Armor> final_armor;
  // col
  for (int i = 1; i < armor_programme.size(); i++)
  {
    double max_confidence_score = 0;
    int best_idx = -1;
    // row
    double fit_area;
    for (int j = 0; j < i; j++)
    {
      if (armor_programme[j][i] > -1)
      {
        double confidence_score =
            ((1 - armors_[armor_programme[j][i]].negative_confidence_) + (armors_[armor_programme[j][i]].confidence_));
        if (max_confidence_score < confidence_score)
        {
          max_confidence_score = confidence_score;
          best_idx = armor_programme[j][i];
        }
      }
    }
    if (best_idx > -1)
      final_armor.push_back(armors_[best_idx]);
  }
  armors_ = final_armor;
}

vector<Point> Processor::calcPCAPoints(vector<Point>& pts)
{
  int size = static_cast<int>(pts.size());
  Mat data_pts = Mat(size, 2, CV_64FC1);
  for (int i = 0; i < size; i++)
  {
    data_pts.at<double>(i, 0) = pts[i].x;
    data_pts.at<double>(i, 1) = pts[i].y;
  }
  // compute PCA
  PCA pca(data_pts, Mat(), CV_PCA_DATA_AS_ROW, 1);

  Point cnt = Point(static_cast<int>(pca.mean.at<double>(0, 0)), static_cast<int>(pca.mean.at<double>(0, 1)));

  Point2d vec;
  double val;
  val = pca.eigenvalues.at<double>(0, 0);           // eigen value
  vec = Point2d(pca.eigenvectors.at<double>(0, 0),  // eigen vector
                (pca.eigenvectors.at<double>(0, 1)));

  Point p1 = cnt + 0.1 * Point(static_cast<int>(vec.x * val), static_cast<int>(vec.y * val));
  Point p2 = cnt - 0.1 * Point(static_cast<int>(vec.x * val), static_cast<int>(vec.y * val));

  vector<Point> result;
  if (p1.y < p2.y)
  {
    result.emplace_back(p1);
    result.emplace_back(p2);
  }
  else
  {
    result.emplace_back(p2);
    result.emplace_back(p1);
  }
  return result;
}

void Processor::findBars()
{
  bars_.clear();
  vector<vector<cv::Point>> contours;
  findContours(morpro_image_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  cv::RotatedRect rect;
  unsigned long contours_size = contours.size();
  for (size_t i = 0; i < contours_size; i++)
  {
    vector<cv::Point> cluster;
    if (contourArea(contours[i]) < 5)
      continue;
    for (auto& j : contours[i])
    {
      if (binary_image_.at<uchar>(j.y, j.x) > 0)
        cluster.push_back(j);
    }
    if (cluster.empty())
      continue;
    contours_.push_back(cluster);
    rect = cv::minAreaRect(contours_[contours_.size() - 1]);
    /*
    ArmorColor bar_color = getBarColor(rect);
    if ((target_is_red_ && bar_color == ArmorColor::RED) || (!target_is_red_ && bar_color == ArmorColor::BLUE))
    {
      Bar bar(rect, contours_[i], bar_color);
      bool is_valid = true;
      if (select_bar_)
        is_valid = isValidBar(bar);
      if (is_valid)
        bars_.emplace_back(bar);
      }
      */
    ArmorColor bar_color = ArmorColor::BLUE;
    if (target_is_red_ == 1)
      bar_color = ArmorColor::RED;
    Bar bar(rect, contours_[contours_.size() - 1], bar_color);
    bool is_valid = true;
    if (select_bar_ == 1)
      is_valid = isValidBar(bar);
    if (is_valid)
      bars_.emplace_back(bar);
  }
}

void Processor::findArmor()
{
  this->target_array_.detections.clear();
  last_frame_armors_ = armors_;
  armors_.clear();
  findBars();
  if (bars_.size() < 2)
    return;
  sort(bars_.begin(), bars_.end(), [](Bar& bar1, Bar& bar2) { return bar1.center_point_.x < bar2.center_point_.x; });
  int i, j;
  unsigned int bars_size = bars_.size();
  bool is_valid_armor;
  vector<vector<int>> armor_programme(bars_size, vector<int>(bars_size, -1));
  int idx = 0;
  for (i = 0; i < bars_size - 1; i++)
  {
    for (j = i + 1; j < bars_size; j++)
    {
      is_valid_armor = isValidArmor(bars_[i], bars_[j]);
      if (is_valid_armor)
      {
        armors_.emplace_back(bars_[i], bars_[j]);
        armors_[armors_.size() - 1].id_ = result_.first;
        armors_[armors_.size() - 1].confidence_ = result_.second;
        armors_[armors_.size() - 1].is_large_armor_ = is_large_armor_;
        armors_[armors_.size() - 1].warp_white_ratio_ = warp_white_ratio_;
        armors_[armors_.size() - 1].negative_confidence_ = negative_confidence_;
        /*
        vector<Point> left_pca_points = calcPCAPoints(armors_[armors_.size() - 1].bar_left_->contour_);   /////////
        vector<Point> right_pca_points = calcPCAPoints(armors_[armors_.size() - 1].bar_right_->contour_);//////////
        armors_[armors_.size() - 1].bars_PCA_4points_.emplace_back(left_pca_points[0]);
        armors_[armors_.size() - 1].bars_PCA_4points_.emplace_back(left_pca_points[1]);
        armors_[armors_.size() - 1].bars_PCA_4points_.emplace_back(right_pca_points[1]);
        armors_[armors_.size() - 1].bars_PCA_4points_.emplace_back(right_pca_points[0]);


//          result_.first = 0;
//          result_.second = 0.0;
//          negative_confidence_ = 0.0;
          */
        armor_programme[i][j] = idx;
        idx++;
        break;
      }
    }
  }
  armorPrograming(armor_programme);

  for (auto& armor : armors_)
  {
    double x_center =
        (armor.bars_4points_[0].x + armor.bars_4points_[1].x + armor.bars_4points_[2].x + armor.bars_4points_[3].x) / 4;
    double y_center =
        (armor.bars_4points_[0].y + armor.bars_4points_[1].y + armor.bars_4points_[2].y + armor.bars_4points_[3].y) / 4;
    double distance = sqrt((x_center - 720) * (x_center - 720) + (y_center - 512) * (y_center - 512));

    rm_msgs::TargetDetection one_target;
    one_target.id = armor.id_;
    one_target.confidence = armor.confidence_;
    one_target.is_large_armor = armor.is_large_armor_;
    one_target.distance_to_image_center = distance;

    int32_t temp[8];
    for (int k = 0; k < 4; k++)
    {
      temp[k * 2] = static_cast<int32_t>(armor.bars_4points_[k].x);
//      if (!rotate_)
        temp[k * 2 + 1] = static_cast<int32_t>(armor.bars_4points_[k].y);
//      else
//        temp[k * 2 + 1] = static_cast<int32_t>(raw_image_.rows - armor.bars_4points_[k].y - 1);
    }
    memcpy(&one_target.pose.orientation.x, &temp[0], sizeof(int32_t) * 2);
    memcpy(&one_target.pose.orientation.y, &temp[2], sizeof(int32_t) * 2);
    memcpy(&one_target.pose.orientation.z, &temp[4], sizeof(int32_t) * 2);
    memcpy(&one_target.pose.orientation.w, &temp[6], sizeof(int32_t) * 2);

    target_array_.detections.emplace_back(one_target);
  }
}

void Processor::drawBars(Mat& image)
{
  cv::Scalar line_color = Scalar(0, 255, 0);
  for (const auto& bar : bars_)
    for (int j = 0; j < 4; j++)
      cv::line(image, bar.points_[j], bar.points_[(j + 1) % 4], line_color, line_width_);
}

void Processor::drawArmors(Mat& image)
{
  cv::Scalar line_color = Scalar(0, 255, 0);
  for (const auto& armor : armors_)
  {
    for (int j = 0; j < 4; j++)
      cv::line(image, armor.bars_4points_[j], armor.bars_4points_[(j + 1) % 4], line_color, line_width_);
    if (use_id_cls_)
    {
      putText(image, to_string(armor.id_), armor.bars_4points_[0], FONT_HERSHEY_COMPLEX, 3, Scalar(0, 255, 0));
    }
    if (armor.is_large_armor_)
      putText(image, "large", armor.bars_4points_[1], FONT_HERSHEY_COMPLEX, 3, Scalar(0, 255, 0));
    else
      putText(image, "small", armor.bars_4points_[1], FONT_HERSHEY_COMPLEX, 3, Scalar(0, 255, 0));
  }
}
void Processor::drawArmorsVertexes(Mat& image)
{
  cv::Scalar vertex_color = Scalar(0, 255, 0);
  for (const auto& armor : armors_)
  {
    for (int i = 0; i < 4; i++)
    {
      circle(image, armor.bars_4points_[i], 3, vertex_color, 2);
      putText(image, to_string(i), armor.bars_4points_[i], FONT_HERSHEY_COMPLEX, 1.5, vertex_color);
    }
    circle(image, armor.center_, 3, vertex_color, 2);
    putText(image, to_string(5), armor.center_, FONT_HERSHEY_COMPLEX, 1.5, vertex_color);

    line(image, armor.bars_4points_[0], armor.bars_4points_[1], Scalar(0, 255, 0), 2, 8, 0);
    line(image, armor.bars_4points_[2], armor.bars_4points_[3], Scalar(0, 255, 0), 2, 8, 0);
  }
}

bool Processor::drawWarp()
{
  if (armors_.size() != 1)
    return false;
  Armor armor_tmp(armors_[0]);
  /// '* expand_ratio' causes bug (unstable digit recognition)
  //    armor_tmp.bars_inter_4points_[0].y += (armor_tmp.bar_left_->length_len_) * expand_ratio_[0];
  //    armor_tmp.bars_inter_4points_[1].y -= (armor_tmp.bar_left_->length_len_) * expand_ratio_[1];
  //    armor_tmp.bars_inter_4points_[2].y -= (armor_tmp.bar_left_->length_len_) * expand_ratio_[1];
  //    armor_tmp.bars_inter_4points_[3].y += (armor_tmp.bar_left_->length_len_) * expand_ratio_[0];
  warp(armor_tmp.bars_inter_4points_, armor_tmp.is_large_armor_, warp_image_);
  return true;
}

void Processor::draw()
{
  Mat draw_image;
  sensor_msgs::ImagePtr msg;
  if (draw_type_ == DrawImage::DISABLE)
    return;
  else if (draw_type_ == DrawImage::RAW)
  {
    raw_image_.copyTo(draw_image);
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", draw_image).toImageMsg();
  }
  else if (draw_type_ == DrawImage::BINARY)
  {
    binary_image_.copyTo(draw_image);
    msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", draw_image).toImageMsg();
  }
  else if (draw_type_ == DrawImage::MORPHOLOGY)
  {
    morpro_image_.copyTo(draw_image);
    msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", draw_image).toImageMsg();
  }
  else if (draw_type_ == DrawImage::BARS)
  {
    raw_image_.copyTo(draw_image);
    drawBars(draw_image);
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", draw_image).toImageMsg();
  }
  else if (draw_type_ == DrawImage::ARMORS)
  {
    raw_image_.copyTo(draw_image);
    drawArmors(draw_image);
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", draw_image).toImageMsg();
  }
  else if (draw_type_ == DrawImage::ARMORS_VERTEXS)
  {
    raw_image_.copyTo(draw_image);
    drawArmorsVertexes(draw_image);
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", draw_image).toImageMsg();
  }
  else if (draw_type_ == DrawImage::WARP)
  {
    if (!drawWarp())
      return;
    msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", warp_image_).toImageMsg();
  }
  else
  {
    raw_image_.copyTo(draw_image);
    drawArmors(draw_image);
    drawBars(draw_image);
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", draw_image).toImageMsg();
  }
  image_pub_.publish(msg);
}

void Processor::paramReconfig()
{
}

Processor::Object Processor::getObj()
{
  std::lock_guard<std::mutex> guard(this->obj_locker_);
  return this->object_;
}

void Processor::putObj()
{
  std::lock_guard<std::mutex> guard(this->obj_locker_);
  object_.points = points_;
  object_.id = labels_;
  object_.prob = probs_;
}
/*
//  void Processor::cnnInfer(Mat& src) {
//    //Blob is a kind of universal container
//      Blob::Ptr frame_blob = infer_request_.GetBlob(input_name_); // input_name_:string
//      InferenceEngine::LockedMemory<void> blob_mapped = InferenceEngine::as<MemoryBlob>(frame_blob)->wmap();
//      auto *blob_data = blob_mapped.as<float *>();
//
//      Mat_<uchar> padding_img = src;// resizeandpadding-before
//      imshow("after padding",padding_img);
//      waitKey(1);
//
////      cout<<padding_img.size;
//
//      // NCHW
//      size_t p_rows = input_shape_[0];
//      size_t p_cols = input_shape_[1];
//      if (padding_img.isContinuous()) {
//          p_rows = 1;
//          p_cols = input_shape_[0] * input_shape_[1];
//      }
//      for (size_t row = 0; row < p_rows; row++) {
//          uchar *data_ptr = padding_img.ptr<uchar>(row);
//          for (size_t col = 0; col < p_cols; col++) {
//              blob_data[row * p_cols + col] = float(data_ptr[col]);  /// 255.0f;
//          }
//      }
//      // predict
//      infer_request_.StartAsync();
//      infer_request_.Wait(IInferRequest::WaitMode::RESULT_READY);
////      infer_request_.Wait(IInferRequest::WaitMode::RESULT_READY);
//      // infer_request_.Infer();
//      std::vector<double> class_prob;
//      vector<double> recalibrate_score;
//      float exp_sum = 0;
//      // get result
//      ROS_INFO("--------------------------------------------");
//      for (auto &output: output_info_) {
//          auto output_name = output.first;
//          Blob::Ptr blob = infer_request_.GetBlob(output_name);
//          LockedMemory<const void> blob_mapped2 = as<MemoryBlob>(blob)->rmap();
//          const float *output_blob = blob_mapped2.as<float *>();
//          const SizeVector output_dims = blob->getTensorDesc().getDims();
//          for (int i = 0; i < output_dims[1]; i++) {
//              class_prob.push_back(output_blob[i]); ///not abs before
////              ROS_WARN("score: %d , %f", i, output_blob[i]  );
//          }
//
//
//
/////softmax function as directly output
//          softmax_score_.clear();
//          for (size_t i = 0; i < class_prob.size(); i++) {
//          exp_sum += exp(class_prob[i]);
///// bug location : our softmax res too large to calculate (var : exp_sum inf -> firstnet_score_->nan -> result_.first->0)
////          ROS_ERROR("%f", exp_sum); //inf
//      }
//          for (size_t i = 0; i < class_prob.size(); i++) {
//              firstnet_score_ = exp(class_prob[i]) / exp_sum;
//              //ROS_INFO(firstnet_score_);
//              softmax_score_.emplace_back(firstnet_score_);
//              firstnet_score_=0;
//          }
//
//          for(size_t i=0;i<softmax_score_.size();i++)
//          {
//              ROS_WARN("score %d , %f",i+1,softmax_score_[i]);
//          }
//
/////result.first : the number we detected , result.second : the confidence
////          result_.first =
////                  (int) (std::max_element(softmax_score_.begin(), softmax_score_.end()) - (softmax_score_.begin())) + 1;
// result_.first=(int)(std::distance(class_prob.begin(),std::min_element(class_prob.begin(),class_prob.end())))+1;
////          ROS_INFO("result: %d ",result_.first);
//          ROS_INFO("--------------------------------------------");
//            result_.second=1;//TODO
//         // result_.second = softmax_score_[result_.first - 1];
//      }
//  }
//  cv::Mat_<uchar> Processor::resizeAndPadding(cv::Mat& src)
//  {
//    int input_h = input_shape_[0];
//    int input_w = input_shape_[1];
//    float r = std::min(input_w / (src.cols * 1.0), input_h / (src.rows * 1.0));
//    int resize_w = r * src.cols;
//    int resize_h = r * src.rows;
//
//    cv::Mat& resize_img = src;
//    cv::resize(src, resize_img, cv::Size(resize_w, resize_h), cv::INTER_NEAREST);
//
//    int padding_background = 114;
//    cv::Mat_<uchar> padding_img(input_h, input_w, 114);
//    resize_img.copyTo(padding_img(cv::Rect(0, 0, resize_img.cols, resize_img.rows)));
//    return padding_img;
//  }

//  vector<double> Processor::recalibrateScores(const vector<double>& activate_vector)
//  {
//    vector<double> result;
//    vector<int> sort_score = argsort(activate_vector);
//    vector<double> alpha_weights(8, 0);
//    for (int i = 1; i < 4; i++)
//    {
//      alpha_weights[sort_score[i - 1]] = (4 - i) / 3.0;
//    }
//    vector<double> distance = computeDistance(activate_vector);
//    vector<double> wscore = weibullWscore(distance, alpha_weights);
//    for (int i = 0; i < 8; i++)
//    {
//      result.push_back(activate_vector[i] * wscore[i]);
//    }
//    double neg_score = 0;
//    for (int i = 0; i < 8; i++)
//    {
//      neg_score += activate_vector[i] * (1 - wscore[i]);
//    }
//    result.push_back(neg_score);
//    return result;
//  }
*/
/// detect number
int Processor::fcRecognition(Mat& image)
{
  cv::Mat blob;
  // Normalize
  cv::Mat image_nor = image / 255.0;
  cv::dnn::blobFromImage(image_nor, blob);
  net_.setInput(blob);
  // Forward pass the image blob through the model
  cv::Mat outputs = net_.forward();
  // Do softmax
  float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
  cv::Mat softmax_prob;
  cv::exp(outputs - max_prob, softmax_prob);
  float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
  softmax_prob /= sum;
  std::vector<float> res = static_cast<std::vector<float>>(softmax_prob);
  fc_result_ = (int)(std::max_element(res.begin(), res.end()) - (res.begin()));
  // Confidence
  result_.second = res.at(fc_result_);
  return fc_result_ + 1;
}

template <typename T>
std::vector<int> Processor::argsort(const std::vector<T>& array)
{
  const int array_len(array.size());
  std::vector<int> array_index(array_len, 0);
  for (int i = 0; i < array_len; ++i)
    array_index[i] = i;
  std::sort(array_index.begin(), array_index.end(),
            [&array](int pos1, int pos2) { return (array[pos1] > array[pos2]); });
  return array_index;
}
/*
//  vector<double> Processor::computeDistance(const vector<double>& activate_vector)
//  {
//    vector<double> distance;
//    for (int i = 0; i < 8; i++)
//    {
//      double dis = 0.0;
//      for (int j = 0; j < 8; j++)
//      {
//        dis += pow((activate_vector[j] - weibull_mean_[i][j]), 2);
//      }
//      dis = sqrt(dis);
//      distance.push_back(dis);
//    }
//    return distance;
//  }

//  vector<double> Processor::weibullWscore(const vector<double>& distance, const vector<double>& alpha_weights)
//  {
//    vector<double> prob;
//    for (int i = 0; i < 8; i++)
//    {
//      double scale = weibull_model_[i][0];
//      double shape = weibull_model_[i][1];
//      double sign = weibull_model_[i][2];
//      double translate = weibull_model_[i][3];
//      double score = weibull_model_[i][4];
//
//      double temp_val = (distance[i] * sign + translate - score) / scale;
//      if (temp_val > 0)
//      {
//        double temp_val1 = pow(temp_val, shape);
//        double s = alpha_weights[i] * exp(-1 * temp_val1);
//        prob.push_back(s);
//      }
//      else
//        prob.push_back(1);
//    }
//    return prob;
//  }
*/
bool Processor::changeStatusCB(rm_msgs::StatusChange::Request& change, rm_msgs::StatusChange::Response& res)
{
  this->target_is_red_ = change.color == 0;
  this->target_is_armor_ = change.target == 0;
  ROS_INFO("change.target is %d", change.target);
  if (target_is_armor_)
    ROS_INFO("hitting the armor.");
  res.switch_is_success = true;
  return true;
}
void Processor::makeDigitalDataset(Armor armor)
{
  static int loop_times = 0;
  if (make_dataset_ && !collected_completely_)
  {
    //      armor.bars_inter_4points_[0].y += (armor.bar_left_->length_len_) * expand_ratio_[0]; ///TODO:ensure not
    //      problem(about stabiilty) armor.bars_inter_4points_[1].y -= (armor.bar_left_->length_len_) *
    //      expand_ratio_[1]; armor.bars_inter_4points_[2].y -= (armor.bar_left_->length_len_) * expand_ratio_[1];
    //      armor.bars_inter_4points_[3].y += (armor.bar_left_->length_len_) * expand_ratio_[0];

    /// when we decide to make single digit dataset , it 's no need to new a Mat ; if we new a Mat, our dataset will exist flash.

    auto dir_flow = opendir(this->dataset_path_.c_str());
    if (dir_flow == nullptr)
    {
      ROS_ERROR("Dataset dir %s is not exist! Please check dataset_path param.", this->dataset_path_.c_str());
      closedir(dir_flow);
      return;
    }
    closedir(dir_flow);
    if (this->image_index_ == 0)
    {
      ROS_INFO("Begin to make %d dataset...\nDataset path is %s.", this->image_id_, this->dataset_path_.c_str());
    }
    if (loop_times == this->image_num_)
    {
      ROS_INFO("The data with ID %d has been collected completely.", this->image_id_);
      this->collected_completely_ = true;
      loop_times = 0;
      return;
    }
    std::string img_dir_path = this->dataset_path_;
    img_dir_path += this->dataset_split_ ? "test/" : "train/";
    img_dir_path += to_string(image_id_) + "/";
    if (this->continute_ && this->last_img_index_.find(this->image_id_) != this->last_img_index_.end())
    {
      ROS_INFO("Continue to collect image...");
      DIR* root_dir = opendir(this->last_img_index_[this->image_id_].second.c_str());
      struct dirent* file;
      int max_index = 0;
      while ((file = readdir(root_dir)) != nullptr)
      {
        if (int(file->d_type) == DT_REG)
        {
          std::string file_name = std::string(file->d_name);
          std::size_t pos = file_name.find('.');
          std::string sub_str = file_name.substr(0, pos);
          int index = stoi(sub_str);
          if (index > max_index)
          {
            max_index = index;
          }
        }
      }
      this->image_index_ = max_index + 1;
      closedir(root_dir);
      this->continute_ = false;
    }
    std::string image_name = to_string(this->image_index_) + ".png";  // file name format is "id_idx.png"
    std::string command = ("mkdir -p " + img_dir_path);
    system(command.c_str());
    cv::imwrite("/home/kr3hy/ArmorId/train/" + img_dir_path + image_name, this->warp_image_);  // directly save warp_img
    this->last_img_index_.insert(
        std::make_pair(this->image_id_, std::pair<int, std::string>(this->image_index_, img_dir_path)));
    this->image_index_++;
    loop_times++;
  }
  return;
}

bool Processor::isLargeArmor(const Armor& armor)
{
  double distance = pow(pow((armor.bar_right_->center_point_.x - armor.bar_left_->center_point_.x), 2) +
                            pow((armor.bar_right_->center_point_.y - armor.bar_left_->center_point_.y), 2),
                        0.5);

  if (distance / ((armor.bar_left_->length_len_ + armor.bar_right_->length_len_) / 2) >= large_armor_ratio_)
    return true;
  else
    return false;
}

}  // namespace rm_digtialimg_proc
