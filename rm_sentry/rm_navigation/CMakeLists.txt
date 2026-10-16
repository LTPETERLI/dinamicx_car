cmake_minimum_required(VERSION 3.10)
project(rm_navigation)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

find_package(catkin REQUIRED
        COMPONENTS
        )
###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS
)
