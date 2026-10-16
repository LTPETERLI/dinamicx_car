cmake_minimum_required(VERSION 3.0.2)
project(buff_vision)
set(CMAKE_BUILD_TYPE Release)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
add_compile_options(-frounding-math)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        cv_bridge
        geometry_msgs
        image_transport
        dynamic_reconfigure
        tf
        rm_msgs
        nodelet
        pluginlib
        camera_info_manager
)

find_package(CGAL REQUIRED)
include(${CGAL_USE_FILE})

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
        cfg/ImageConverter.cfg
        cfg/BuffRecognizer.cfg
        cfg/PnpSolver.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        std_msgs
        cv_bridge
        geometry_msgs
        image_transport
        dynamic_reconfigure
        tf
        pluginlib
        rm_msgs
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/buff_vision.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/buff_vision_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

add_library(bit_flag src/bit_flags.cpp)

add_executable(${PROJECT_NAME}_node src/buff_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${CGAL_LIBS} bit_flag)

add_library(${PROJECT_NAME}_image_converter src/image_converter.cpp src/image_converter_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_image_converter ${catkin_LIBRARIES} bit_flag)
add_dependencies(${PROJECT_NAME}_image_converter ${PROJECT_NAME}_gencfg)

add_library(${PROJECT_NAME}_recognizer src/buff_recognizer_nodelet.cpp src/buff_recognizer_base.cpp)
target_link_libraries(${PROJECT_NAME}_recognizer ${catkin_LIBRARIES} bit_flag)

add_library(${PROJECT_NAME}_pnp_solver src/pnp_solver_nodelet.cpp src/pnp_solver_base.cpp)
target_link_libraries(${PROJECT_NAME}_pnp_solver ${catkin_LIBRARIES} bit_flag)