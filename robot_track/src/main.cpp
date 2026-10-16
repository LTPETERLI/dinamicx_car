//
// Created by yawara on 24-1-9.
//
#include "track.h"

int main(int argc, char** argv) {
    robot_track::Track* robot_control;
    ros::init(argc, argv, "robot_track");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(100);
    robot_control = new robot_track::Track(nh);

    while (ros::ok()) {
        ros::spinOnce();
        robot_control->Run();
        loop_rate.sleep();
    }
    return 0;
}