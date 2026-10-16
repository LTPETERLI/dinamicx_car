cmake_minimum_required(VERSION 3.0.2)
project(rm_pose_solver)

add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS "-Wl,--no-undefined")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        pluginlib
        dynamic_reconfigure
        std_msgs
        geometry_msgs
        visualization_msgs
        # image_transport
        cv_bridge
        image_geometry
        nodelet
        tf
        tf2
        tf2_geometry_msgs
        rm_msgs
        rm_vision
		message_generation
        )

## find_package(OpenCV REQUIRED)
## cv_bridge will import OpenCV 4.2.0 automatically
message("OpenCV version:" ${OpenCV_VERSION})

add_service_files(FILES axis_mapping.srv rotation_translation.srv)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        dynamic_reconfigure
        geometry_msgs
        # message_generation
        rm_msgs
        rm_vision
        std_msgs
        pluginlib
        LIBRARIES ${PROJECT_NAME}
)

include_directories(
		include/
		include/rm_vision
		${catkin_INCLUDE_DIRS}
)

add_library(pose_solver_plugin src/posesolver_plugins.cpp)
target_link_libraries(pose_solver_plugin
		${catkin_LIBRARIES}
		)

##add_executable(virtual_pnp_test src/virtual_pnp_test.cpp src/posesolver_plugins.cpp)
##use the above line instead of the following for debugging
#add_executable(virtual_pnp_test src/virtual_pnp_test.cpp)
#add_dependencies(virtual_pnp_test ${PROJECT_NAME}_gencpp)
#target_link_libraries(virtual_pnp_test
#		${catkin_LIBRARIES}
#		)

add_executable(pose_solver_node src/posesolver_node.cpp)
target_link_libraries(pose_solver_node ${catkin_LIBRARIES})
