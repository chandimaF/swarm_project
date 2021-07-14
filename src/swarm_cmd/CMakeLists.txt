cmake_minimum_required(VERSION 3.0.2)
project(swarm_cmd)
add_compile_options(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation transmit_wifi)

add_message_files(
        FILES
        SwarmCommand.msg
)
generate_messages(
        DEPENDENCIES
        std_msgs
        swarm_cmd
        transmit_wifi
)

catkin_package(
        CATKIN_DEPENDS message_runtime
)

include_directories(
        ../../devel/include # Hack for Clion.
        ../../lib
        ${catkin_INCLUDE_DIRS}
)


add_executable(command_swarm
        SwarmCommander.cpp
        )

add_dependencies(command_swarm ${catkin_EXPORTED_TARGETS})

include_directories(${catkin_INCLUDE_DIRS})
target_link_libraries(command_swarm ${catkin_LIBRARIES} ${Boost_INCLUDE_DIRS})