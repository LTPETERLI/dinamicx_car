cmake_minimum_required(VERSION 3.0.2)
project(rm_digtialimg_proc)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS"-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED
        roscpp
        tf
        rm_vision
        message_generation
        dynamic_reconfigure
        std_msgs
        geometry_msgs
        visualization_msgs
        image_transport
        cv_bridge
        image_geometry
        nodelet
        tf2
        tf2_geometry_msgs
        rm_msgs
        )

#find_package(InferenceEngine REQUIRED)
#set(ngraph_DIR "/opt/intel/openvino_2022/deployment_tools/ngraph")
#find_package(ngraph REQUIRED)
#set(ngraph_LIBRARIES "/opt/intel/openvino_2022/deployment_tools/ngraph/lib/libngraph.so")
#set(ngraph_INCLUDE_DIRS "/opt/intel/openvino_2022/deployment_tools/ngraph/include/")

generate_dynamic_reconfigure_options(
        cfg/Armor.cfg
        cfg/Draw.cfg
        cfg/Preprocess.cfg
        cfg/Makedataset.cfg
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        rm_vision
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
        nodelet
        tf2
        tf2_geometry_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        /opt/intel/openvino_2021/deployment_tools/inference_engine/include/
        /opt/intel/openvino_2021/deployment_tools/ngraph/include/
)

FILE(GLOB_RECURSE SOURCE_FILES "src/*.cpp")

add_library(
        ${PROJECT_NAME}
        src/process_plugins.cpp
        )

add_dependencies(${PROJECT_NAME}
        ${PROJECT_NAME}_gencfg
        )

target_link_libraries(
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${InferenceEngine_LIBRARIES}
        ${ngraph_LIBRARIES}
)