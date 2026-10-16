cmake_minimum_required(VERSION 3.10)
project(rm_localization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslint
        rm_msgs
        rm_common
        tf2_geometry_msgs
        apriltag_ros
        )

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        roslint
        rm_msgs
        rm_common
        tf2_geometry_msgs
        apriltag_ros
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(tags_distribute src/tags_distribution/tags_distribute_node.cpp)
target_link_libraries(tags_distribute ${catkin_LIBRARIES})

#rm_localization
add_executable(rm_localization src/localization/rm_localization.cpp)
target_link_libraries(rm_localization ${catkin_LIBRARIES})

add_executable(odom_restorer src/odom_restorer/odom_restorer.cpp src/odom_restorer/restorer_node.cpp)
target_link_libraries(odom_restorer ${catkin_LIBRARIES})
