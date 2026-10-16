//
// Created by luotinkai on 2022/3/17.
//
#include "lidar_repost.h"

PLUGINLIB_EXPORT_CLASS(lidar_repost::LivoxRepost, nodelet::Nodelet)

namespace lidar_repost
{
LivoxRepost::LivoxRepost()
{
  ros::NodeHandle root_nh;
  livox_sub_ = root_nh.subscribe<livox_ros_driver2::CustomMsg>("/livox/lidar", 100, &LivoxRepost::lidarCallBack, this);
  publisher_ptr_ = new ros::Publisher;
  *publisher_ptr_ = root_nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar_repost", 100);
}

void LivoxRepost::onInit()
{
  nh_ = getNodeHandle();
  nh_.getParam("frame_id", frame_id_);
  NODELET_INFO_STREAM_ONCE("Initialised.");
  thread_ = std::thread(std::bind(&LivoxRepost::loop, this));
}

void LivoxRepost::initPointcloud2MsgHeader(sensor_msgs::PointCloud2& cloud)
{
  cloud.header.frame_id.assign(frame_id_);
  cloud.height = 1;
  cloud.width = 0;
  cloud.fields.resize(6);
  cloud.fields[0].offset = 0;
  cloud.fields[0].name = "x";
  cloud.fields[0].count = 1;
  cloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  cloud.fields[1].offset = 4;
  cloud.fields[1].name = "y";
  cloud.fields[1].count = 1;
  cloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  cloud.fields[2].offset = 8;
  cloud.fields[2].name = "z";
  cloud.fields[2].count = 1;
  cloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  cloud.fields[3].offset = 12;
  cloud.fields[3].name = "intensity";
  cloud.fields[3].count = 1;
  cloud.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  cloud.fields[4].offset = 16;
  cloud.fields[4].name = "tag";
  cloud.fields[4].count = 1;
  cloud.fields[4].datatype = sensor_msgs::PointField::UINT8;
  cloud.fields[5].offset = 17;
  cloud.fields[5].name = "line";
  cloud.fields[5].count = 1;
  cloud.fields[5].datatype = sensor_msgs::PointField::UINT8;
  cloud.point_step = sizeof(LivoxPointXyzrtl);
}

void LivoxRepost::lidarCallBack(const livox_ros_driver2::CustomMsgConstPtr& msg)
{
  sensor_msgs::PointCloud2 cloud;
  initPointcloud2MsgHeader(cloud);

  cloud.point_step = sizeof(LivoxPointXyzrtl);

  cloud.width = msg->point_num;
  cloud.row_step = cloud.width * cloud.point_step;

  cloud.is_bigendian = false;
  cloud.is_dense = true;

  uint64_t time_stamp = 0;
  if (!msg->points.empty())
  {
    time_stamp = msg->timebase;
  }
  cloud.header.stamp = ros::Time(time_stamp / 1000000000.0);

  std::vector<LivoxPointXyzrtl> points;
  for (size_t i = 0; i < msg->point_num; ++i)
  {
    LivoxPointXyzrtl point;
    point.x = msg->points[i].x;
    point.y = msg->points[i].y;
    point.z = msg->points[i].z;
    point.reflectivity = msg->points[i].reflectivity;
    point.tag = msg->points[i].tag;
    point.line = msg->points[i].line;

    points.push_back(std::move(point));
  }
  cloud.data.resize(msg->point_num * sizeof(LivoxPointXyzrtl));
  memcpy(cloud.data.data(), points.data(), msg->point_num * sizeof(LivoxPointXyzrtl));

  publisher_ptr_->publish(cloud);
}

}  // namespace lidar_repost
