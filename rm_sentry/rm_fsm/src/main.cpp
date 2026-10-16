//
// Created by luohx on 7/27/20.
//

#include "rm_fsm/state_machine.h"

int main(int argc, char** argv)
{
  std::string robot;
  ros::init(argc, argv, "rm_fsm");
  ros::NodeHandle fsm_nh("~");
  StateMachine sm(fsm_nh);
  ROS_INFO("Running sentry robot.");
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    sm.update();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
