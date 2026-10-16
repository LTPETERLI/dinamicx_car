//
// Created by luotinkai on 2022/3/17.
//

#pragma once

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <pcl-1.10/pcl/impl/point_types.hpp>
#include <pcl-1.10/pcl/point_cloud.h>
#include <ros/callback_queue.h>
#include <thread>

namespace lidar_repost
{
typedef struct
{
  float x;
  float y;
  float z;
  float reflectivity;
  uint8_t tag;
  uint8_t line;
} LivoxPointXyzrtl;

class LivoxRepost : public nodelet::Nodelet
{
public:
  LivoxRepost();
  ~LivoxRepost() override
  {
    thread_.join();
  };
  void onInit() override;
  void initPointcloud2MsgHeader(sensor_msgs::PointCloud2& cloud);
  void lidarCallBack(const livox_ros_driver2::CustomMsgConstPtr& msg);
  void loop()
  {
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

private:
  std::thread thread_;
  std::string frame_id_{};
  ros::NodeHandle nh_;
  ros::Publisher* publisher_ptr_;
  ros::Subscriber livox_sub_;
};

}  // namespace lidar_repost
