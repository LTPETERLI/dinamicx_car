cmake_minimum_required(VERSION 3.10)
project(lidar_repost)

find_package(PCL REQUIRED)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        pcl_ros
        livox_ros_driver2
        nodelet
        )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        pcl_ros
        livox_ros_driver2
        nodelet
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)
## Declare a C++ library
add_library(lidar_repost
        src/lidar_repost.cpp
        )

target_link_libraries(lidar_repost ${catkin_LIBRARIES})
