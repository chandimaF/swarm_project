cmake_minimum_required(VERSION 3.0.2)
project(apply_swarm_patch)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs transmit_wifi swarm_utilities)
find_package(Boost REQUIRED COMPONENTS filesystem iostreams)

add_library(nlohmann_json ../../lib/json.cpp)
add_library(sha256 ../../lib/sha256.cpp)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES swarm_utilities
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS message_runtime
        CATKIN_DEPENDS message_runtime transmit_wifi swarm_utilities
)

include_directories(
#        ../../devel/include # Hack for Clion.
        ../../lib
        ${catkin_INCLUDE_DIRS}
)


add_executable(apply_swarm_patch
        PatchReceiver.cpp
        PatchReceiver.h
        )

add_executable(create_swarm_patch
        PatchTransmitter.cpp
        PatchTransmitter.h
        )

add_dependencies(apply_swarm_patch ${catkin_EXPORTED_TARGETS})
add_dependencies(create_swarm_patch ${catkin_EXPORTED_TARGETS})

include_directories(${catkin_INCLUDE_DIRS})
target_link_libraries(apply_swarm_patch ${catkin_LIBRARIES} ${Boost_INCLUDE_DIRS} Boost::filesystem Boost::iostreams sha256 nlohmann_json)
target_link_libraries(create_swarm_patch ${catkin_LIBRARIES} ${Boost_INCLUDE_DIRS} Boost::filesystem Boost::iostreams sha256 nlohmann_json)

install(TARGETS nlohmann_json sha256
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )