cmake_minimum_required(VERSION 3.0.2)
project(apply_swarm_patch)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs transmit_wifi)
find_package(Boost REQUIRED COMPONENTS filesystem iostreams)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES apply_swarm_patch
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS message_runtime
        CATKIN_DEPENDS message_runtime transmit_wifi
)

include_directories(
        ../../devel/include # Hack for Clion.
        ${catkin_INCLUDE_DIRS}
)

add_executable(apply_swarm_patch
        PatchReceiver.cpp
        PatchReceiver.h
        sha256.cpp
        sha256.h
        )

add_executable(create_swarm_patch
        PatchTransmitter.cpp
        PatchTransmitter.h
        )

add_dependencies(apply_swarm_patch ${catkin_EXPORTED_TARGETS})
add_dependencies(create_swarm_patch ${catkin_EXPORTED_TARGETS})

include_directories(${catkin_INCLUDE_DIRS})
target_link_libraries(apply_swarm_patch ${catkin_LIBRARIES} ${Boost_INCLUDE_DIRS} Boost::filesystem Boost::iostreams)
target_link_libraries(create_swarm_patch ${catkin_LIBRARIES} ${Boost_INCLUDE_DIRS} Boost::filesystem Boost::iostreams)