cmake_minimum_required(VERSION 3.0.2)
project(robot_test_fxj)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED)

##命名ROS_DEPENDENCIES，并将roscpp等依赖存到这个变量里面
set(ROS_DEPENDENCIES
        roscpp
        geometry_msgs
        std_msgs
        rm_msgs
)
##直接调用ROS_DEPENDENCIES，即所有的依赖（如需修改依赖，直接在上方修改即可）
find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES})

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        CATKIN_DEPENDS
        ${ROS_DEPENDENCIES}
)

include_directories(
        include     #include指当前包内的include目录，包含其中的所有,h文件
        ${catkin_INCLUDE_DIRS}
)
##添加可执行文件,头文件一定要放在第一个
add_executable(${PROJECT_NAME}  src/robot_test_fxj.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(robot_test_fxj_remote  src/robot_test_fxj_remote.cpp)
target_link_libraries(robot_test_fxj_remote ${catkin_LIBRARIES})




