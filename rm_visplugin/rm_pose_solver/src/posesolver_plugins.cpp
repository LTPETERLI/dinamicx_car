//
// Created by kaxns on 10/23/21.
//

#include "rm_pose_solver/posesolver_plugins.h"
#include <pluginlib/class_list_macros.h>  // Includes the pluginlib macro that allows us to register classes as plugins.

// checking whether the plugin is available: rospack plugins --attrib=plugin <base_class_pkg>
PLUGINLIB_EXPORT_CLASS(rm_pose_solver::PnpPoseSolverPluginImpl, rm_pose_solver::PnpPoseSolverPluginInterface)
