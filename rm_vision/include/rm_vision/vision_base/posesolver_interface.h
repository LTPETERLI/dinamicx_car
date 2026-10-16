//
// Created by heyicheng on 10/6/21.
//

#pragma once

#include <type_traits>
#include <ros/node_handle.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include "rm_vision/vision_base/processor_interface.h"

namespace rm_vision
{
/**
 * @brief template Base class of pose solver.
 *
 * Only pure virtual functions were provided.
 * Any member variables need to be provided by the subclass.
 */
template <typename Targets, typename RuntimeInfo, typename Img>
class PoseSolverInterface
{
public:
  virtual void initializeOnlyOnce(ros::NodeHandle& outside_nh, sensor_msgs::CameraInfoConstPtr& cam_info) = 0;
  virtual void initializeOnlyOnce(ros::NodeHandle& outside_nh, sensor_msgs::CameraInfo& cam_info) = 0;
  virtual void initializeOnlyOnce(ros::NodeHandle& outside_nh) = 0;

  virtual void execute(Targets detected_targets, RuntimeInfo info, Img img) = 0;
  virtual void execute(Targets detected_targets, RuntimeInfo info) = 0;
  /// config parameters dynamically
  virtual void paramReConfig() = 0;
  virtual ~PoseSolverInterface() = default;
  /// prepares variables for poseSolver(), not receiving from previous step!
  virtual void initialize() = 0;
  /// calculates the pose of target by PnP or other methods
  virtual void poseSolver() = 0;
  /// sends the result to rm_control
  virtual void send2Ctl() = 0;

private:
  typedef typename std::remove_reference<Img>::type img_type;
  typedef typename std::remove_reference<Targets>::type targets_type;
  typedef typename std::remove_reference<RuntimeInfo>::type runtime_info_type;

protected:
  PoseSolverInterface() = default;
  std_msgs::Header sync_;
  runtime_info_type runtime_info_;
  virtual void updateSync(Targets targets, RuntimeInfo info) = 0;
  template <typename T>
  inline T getParam(const ros::NodeHandle& nh, const std::string& param_name, const T& default_val)
  {
    T param_val;
    nh.param<T>(param_name, param_val, default_val);
    return param_val;
  }
};

}  // namespace rm_vision
