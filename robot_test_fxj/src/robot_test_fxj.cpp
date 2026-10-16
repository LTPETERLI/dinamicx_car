//
// Created by fxj on 2023/9/30.
//
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc,char **argv){
    ros::init(argc,argv,"test");
    ros::NodeHandle n;
    ros::Publisher pub=n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
    ros::Rate rate(10);     //发送频率：10Hz
    while (ros::ok()){
        geometry_msgs::Twist msg;   //创建需要发布的消息类型
        msg.linear.x=0.1;
        msg.linear.y=0.1;
        //发布消息
        pub.publish(msg);
        ros::spinOnce();
        //向rosout发送相应消息
        ROS_INFO("linear.x:%f,linear.y:%f",msg.linear.x,msg.linear.y);
        rate.sleep();
    }
}