cmake_minimum_required(VERSION 3.0.2)
project(swarm_docker)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
include_directories(${catkin_INCLUDE_DIRS})

