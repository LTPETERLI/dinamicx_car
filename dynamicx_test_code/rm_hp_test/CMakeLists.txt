cmake_minimum_required(VERSION 3.0.2)
project(rm_hp_test)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        rm_referee
        rm_msgs
)


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ros_tf_tree
        CATKIN_DEPENDS
        roscpp
        std_msgs
        rm_msgs
        rm_referee
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


add_executable(rm_hp_test           src/main.cpp)
add_executable(rm_hp_pub_test       src/rm_hp_pub.cpp)

target_link_libraries(rm_hp_test        ${catkin_LIBRARIES})
target_link_libraries(rm_hp_pub_test    ${catkin_LIBRARIES})