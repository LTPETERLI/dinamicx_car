//
// Created by yawara on 23-9-30.
//

#include "topic_test.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "vel_pub");
    ros::NodeHandle vel_nh;
    vel_pub = vel_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Rate r(10);

    while (ros::ok())
    {
        vel_2D.linear.x = 0.1;
        vel_2D.linear.y = 0.1;

        vel_pub.publish(vel_2D);

        ROS_INFO("linear.x :%f",vel_2D.linear.x);
        ROS_INFO("linear.y :%f",vel_2D.linear.y);

        r.sleep();
        ros::spinOnce();
    }
}

