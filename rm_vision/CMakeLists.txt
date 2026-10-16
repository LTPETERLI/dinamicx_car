cmake_minimum_required(VERSION 3.0.2)
project(rm_vision)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf
        message_generation
        dynamic_reconfigure
        std_msgs
        geometry_msgs
        visualization_msgs
        image_transport
        cv_bridge
        image_geometry
        tf2
        tf2_geometry_msgs
        rm_msgs
        pluginlib
        )


find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        ../rm_visplugin
)

include_directories(
  include/
  include/rm_vision
  ${catkin_INCLUDE_DIRS}
)

#FILE(GLOB_RECURSE SOURCE_FILES "src/target/*.cpp")
