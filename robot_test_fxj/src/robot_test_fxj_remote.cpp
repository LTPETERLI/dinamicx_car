//
// Created by fxj on 2023/9/30.
//
#include <rm_msgs/DbusData.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

class Listen_Remote
{
private:
//    ros::NodeHandle nh;
    ros::Subscriber sub_;    //订阅者
    ros::Publisher pub_;     //发布者

public:
    Listen_Remote(ros::NodeHandle &nh){
        //sub_ = nh.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &Listen_Remote::callback,this);
        sub_=nh.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &Listen_Remote::callback, this);
        pub_=nh.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
    }
    void callback(const rm_msgs::DbusData::ConstPtr& msg){
        ROS_INFO("get ch_r_x:%f,ch_r_y%f",msg->ch_r_x,msg->ch_r_y);
        //将接受的数值赋值
        geometry_msgs::Twist pub_msg;
        pub_msg.linear.x = msg->ch_r_x;
        pub_msg.linear.y = msg->ch_r_y;
        pub_.publish(pub_msg);
    }

};
int main(int argc,char **argv){
    ros::init(argc,argv,"remote_test");
    ros::NodeHandle nh("~");

    Listen_Remote lr(nh);
    ros::spin();

}