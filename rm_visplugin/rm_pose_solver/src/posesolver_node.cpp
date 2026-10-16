//
// Created by kaxns on 4/1/22.
//

#include "rm_pose_solver/posesolver_plugins.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_solver_node");
  ros::NodeHandle n("~");
  rm_pose_solver::PnpPoseSolverMsgNode pose_solver_msg_node(n);
  ros::spin();
  return 0;
}
