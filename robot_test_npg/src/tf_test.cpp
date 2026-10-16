//
// Created by nepgear on 2023/11/23.
//
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "iostream"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

void sub_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Vector3 aa=tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
    transform.setOrigin(aa);
    tf::Quaternion quat;                                     //定义一个四元数
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    transform.setRotation(quat);
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"/map","/livox_frame"));
    std::cout<<"ss";
}
int main(int argc,char **argv)
{
    ros::init(argc,argv,"tf_vv");
    ros::NodeHandle nh;
    ros::Rate r(5);
    tf::TransformListener t;
    tf::StampedTransform odom2base_link;
    tf::StampedTransform map2odom,map2body;
    ros::Subscriber s =nh.subscribe("/robot/dlio/odom_node/odom",100,sub_callback);
    while (ros::ok())
    {
        r.sleep();
    }
}