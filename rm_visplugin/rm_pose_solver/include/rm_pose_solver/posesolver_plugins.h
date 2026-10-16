//
// Created by kaxns on 3/31/22.
//

#pragma once
#include "posesolver_core.h"
using std::string;

namespace rm_pose_solver
{
class PnpPoseSolverPluginInterface
  : public PnpPoseSolverCore<rm_vision::ProcessorInterface::Object&, sensor_msgs::CameraInfoConstPtr&>
{
#if ENABLE_POSE_SOLVER_LOGGING_AND_VISUALIZATION
public:
  void execute(TargetsType detected_targets, RuntimeInfoType info, ImgType img) override = 0;

protected:  // Disallow calls through base class pointers when ENABLE_POSE_SOLVER_LOGGING_AND_VISUALIZATION is enabled.
  void execute(TargetsType detected_targets, RuntimeInfoType info) override = 0;
#endif
};

class PnpPoseSolverPluginImpl final : public PnpPoseSolverPluginInterface
{
public:
  PnpPoseSolverPluginImpl() = default;
  ~PnpPoseSolverPluginImpl() override = default;
  void updateSync(TargetsType targets, RuntimeInfoType info) override;
  void execute(TargetsType detected_targets, RuntimeInfoType info, ImgType img) override;
#if ENABLE_POSE_SOLVER_LOGGING_AND_VISUALIZATION
private:
#endif
  void execute(TargetsType detected_targets, RuntimeInfoType info) override;

private:
  void impl(TargetsType detected_targets, size_t i, const std::string& height, const std::string& width);
  void run(TargetsType detected_targets) override;
  template <typename T>
  void runtimeInfoInterpreter(std::vector<T>& points_2dim);
};

// ********************************************************************************************************************
// START: implementation of PnpPoseSolverPluginImpl
// ********************************************************************************************************************

template <typename T>
void PnpPoseSolverPluginImpl::runtimeInfoInterpreter(std::vector<T>& points_2dim)
{
  T offset(runtime_info_->roi.x_offset, runtime_info_->roi.y_offset);
  for (auto& item : points_2dim)
    item += offset;
}

void PnpPoseSolverPluginImpl::run(TargetsType detected_targets)
{
  size_t number_of_targets = detected_targets.id.size();
  for (size_t i = 0; i < number_of_targets; ++i)
  {
    if (detected_targets.is_large_armor[i])
    {
      impl(detected_targets, i, "large_height", "large_width");
    }
    else
    {
      impl(detected_targets, i, "small_height", "small_width");
    }
  }
  send2Ctl();
}

void PnpPoseSolverPluginImpl::impl(TargetsType detected_targets, size_t i, const string& height, const string& width)
{
  runtimeInfoInterpreter<>(detected_targets.points[i]);
  initialize<>(detected_targets.points[i], this->template getParam<>(armor_size_nh_, height, 0.06),
               this->template getParam(armor_size_nh_, width, 0.123));
  PnpPoseSolverCore::is_large_armor_ = detected_targets.is_large_armor[i];
  PnpPoseSolverCore::distance_to_centre_ = detected_targets.distance_to_image_center;
  PnpPoseSolverCore::executeSingleTarget(detected_targets.id[i]);
}

void PnpPoseSolverPluginImpl::updateSync(TargetsType targets, RuntimeInfoType info)
{
  this->sync_ = info->header;
  this->runtime_info_ = info;
}

/**
 * @brief This function is designed for racing. The image is no longer required as there is no need
 * to drawing on the image for debugging. Hence, image shouldn't be a parameter anymore.
 *
 * @note initializeOnlyOnce() must be called beforehand
 * @warning Do NOT enable ENABLE_POSE_SOLVER_LOGGING_AND_VISUALIZATION
 */
void PnpPoseSolverPluginImpl::execute(TargetsType detected_targets, RuntimeInfoType info)
{
  updateSync(detected_targets, info);      // synchronize timestamp
  this->target_array_.detections.clear();  // release old data
  run(detected_targets);
}

/**
 * @brief Solve PnP, and plot the result on the input image if ENABLE_POSE_SOLVER_RESULT_TEST is active.
 * @note initializeOnlyOnce() must be called beforehand
 */
void PnpPoseSolverPluginImpl::execute(TargetsType detected_targets, RuntimeInfoType info, ImgType img)
{
  this->img_ = img;  // refresh img_
  execute(detected_targets, info);
}

// ********************************************************************************************************************
// END: implementation of PnpPoseSolverPluginImpl
// ********************************************************************************************************************

class PnpPoseSolverMsg : public PnpPoseSolverCore<const rm_msgs::TargetDetectionArray, int*>
{
public:
  PnpPoseSolverMsg()
  {
    using info_type = std::remove_pointer<runtime_info_type>::type;
    static info_type info[2];
    this->runtime_info_ = &info[0];
  }
  ~PnpPoseSolverMsg() override = default;
  void updateSync(TargetsType targets, RuntimeInfoType info) override;
  void execute(TargetsType detected_targets, RuntimeInfoType info) override;
  void execute(TargetsType detected_targets, RuntimeInfoType info, ImgType img) override;  // NOT support

private:
  void impl(const TargetsType::_detections_type::value_type& target, const std::string& height,
            const std::string& width);
  void run(TargetsType detected_targets) override;
  template <typename T>
  void runtimeInfoInterpreter(std::vector<T>& points_2dim);
#if ENABLE_POSE_SOLVER_LOGGING_AND_VISUALIZATION
  void visualizeResult(cv::Mat_<double>& K, cv::Mat_<double>& r_vec, cv::Mat_<double>& t_vec) override
  {
    // Bypass the default implementation because img_ is not synchronized.
  }
#endif
};

// ********************************************************************************************************************
// START: implementation of PnpPoseSolverMsg
// ********************************************************************************************************************

void PnpPoseSolverMsg::updateSync(TargetsType targets, RuntimeInfoType __attribute__((unused)) info)
{
  this->sync_ = targets.header;
}

template <typename T>
void PnpPoseSolverMsg::runtimeInfoInterpreter(std::vector<T>& points_2dim)
{
  T offset(runtime_info_[0], runtime_info_[1]);
  for (auto& item : points_2dim)
    item += offset;
}

void PnpPoseSolverMsg::impl(const TargetsType::_detections_type::value_type& target, const std::string& height,
                            const std::string& width)
{
  runtime_info_[0] = target.pose.position.x;  // get the ROI x_offset
  runtime_info_[1] = target.pose.position.y;  // get the ROI y_offset
  int32_t data[4 * 2];                        // data of 4 2D points
  static_assert(sizeof(decltype(target.pose.orientation.x)) == sizeof(double) &&
                    sizeof(decltype(target.pose.orientation.y)) == sizeof(double) &&
                    sizeof(decltype(target.pose.orientation.z)) == sizeof(double) &&
                    sizeof(decltype(target.pose.orientation.w)) == sizeof(double),
                "Insufficient size!");
  memcpy(&data[0], &target.pose.orientation.x, sizeof(int32_t) * 2);
  memcpy(&data[2], &target.pose.orientation.y, sizeof(int32_t) * 2);
  memcpy(&data[4], &target.pose.orientation.z, sizeof(int32_t) * 2);
  memcpy(&data[6], &target.pose.orientation.w, sizeof(int32_t) * 2);
  std::vector<cv::Point2d> points_2dim(4);
  // Convert the msg type target to std::vector<cv::Point_<>>
  for (size_t i = 0; i < 4; ++i)
  {
    points_2dim[i].x = data[2 * i];
    points_2dim[i].y = data[2 * i + 1];
  }
  runtimeInfoInterpreter<>(points_2dim);
  initialize<>(points_2dim, this->template getParam<>(armor_size_nh_, height, 0.06),
               this->template getParam(armor_size_nh_, width, 0.123));
  PnpPoseSolverCore::is_large_armor_ = target.is_large_armor;
  PnpPoseSolverCore::distance_to_centre_ = target.distance_to_image_center;
  executeSingleTarget(target.id);
}

void PnpPoseSolverMsg::run(TargetsType detected_targets)
{
  for (auto& target : detected_targets.detections)
  {
    if (target.is_large_armor)
    {
      impl(target, "large_height", "large_width");
    }
    else
    {
      impl(target, "small_height", "small_width");
    }
  }
  send2Ctl();
}

void PnpPoseSolverMsg::execute(TargetsType detected_targets, RuntimeInfoType info)
{
  updateSync(detected_targets, info);      // synchronize timestamp
  this->target_array_.detections.clear();  // release old data
  run(detected_targets);
}

void PnpPoseSolverMsg::execute(TargetsType detected_targets, RuntimeInfoType info, ImgType img)
{
  // TODO: Support this method.
}

// ********************************************************************************************************************
// END: implementation of PnpPoseSolverMsg
// ********************************************************************************************************************

/**
 * @brief A ROS node implementation of PnpPoseSolverMsg.
 */
class PnpPoseSolverMsgNode final : private PnpPoseSolverMsg
{
public:
  explicit PnpPoseSolverMsgNode(ros::NodeHandle& nh);
  PnpPoseSolverMsgNode(const PnpPoseSolverMsgNode&) = delete;
  template <typename Any>
  Any operator=(const PnpPoseSolverMsgNode&) = delete;
  ~PnpPoseSolverMsgNode() override = default;

private:
  void callback(TargetsType& targets)
  {
    execute(targets, nullptr);
  }
  ros::Subscriber targets_sub_;
};

/**
 * @brief Any initialization will be done by this constructor.
 * @warning For performance reason, do NOT enable ENABLE_POSE_SOLVER_LOGGING_AND_VISUALIZATION
 */
PnpPoseSolverMsgNode::PnpPoseSolverMsgNode(ros::NodeHandle& nh)
{
  sensor_msgs::CameraInfoConstPtr camera_info{};
  ros::Subscriber camera_info_grabber = nh.subscribe<sensor_msgs::CameraInfo>(
      "/galaxy_camera/camera_info", 10,
      [&camera_info](const sensor_msgs::CameraInfoConstPtr& info) -> void { camera_info = info; });
  while (camera_info == nullptr && ros::ok())
    ros::spinOnce();  // Spin until camera info is grabbed.
  PnpPoseSolverMsg::initializeOnlyOnce(nh, camera_info);
  targets_sub_ = nh.subscribe("/processor/result_msg", 1, &PnpPoseSolverMsgNode::callback, this);
  ROS_INFO("Success");
}

}  // namespace rm_pose_solver