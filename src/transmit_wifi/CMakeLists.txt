cmake_minimum_required(VERSION 3.0.2)
project(transmit_wifi)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
include_directories(${catkin_INCLUDE_DIRS})
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES test_pkg
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_lib
)

add_executable(wifi_receiver
        WifiReceiver.cpp
        )

add_executable(wifi_transmitter
        WifiTransmitter.cpp
        )

target_link_libraries(wifi_receiver ${catkin_LIBRARIES})
target_link_libraries(wifi_transmitter ${catkin_LIBRARIES})
