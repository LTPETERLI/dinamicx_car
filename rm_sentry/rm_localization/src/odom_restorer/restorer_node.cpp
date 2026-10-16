//
// Created by luotinkai on 3/12/23.
//

#include "rm_localization/odom_restorer/odom_restorer.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_restorer");
  ros::NodeHandle odom_nh("~");
  OdomRestorer* restorer = new OdomRestorer(odom_nh);
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    restorer->run();
    ros::spinOnce();
    loop_rate.sleep();
  }
}
