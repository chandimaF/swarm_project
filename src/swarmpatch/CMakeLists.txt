cmake_minimum_required(VERSION 3.0.2)
project(swarmpatch)
add_compile_options(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs transmit_wifi swarm_cmd message_generation)
find_package(Boost REQUIRED COMPONENTS filesystem iostreams)

add_library(nlohmann_json ../../lib/json.cpp)
add_library(sha256 ../../lib/sha256.cpp)
add_library(pstream ../../lib/pstream.cpp)

add_message_files(
        FILES
        PatchRequest.msg
)
generate_messages(
        DEPENDENCIES
        std_msgs
        swarmpatch
)

catkin_package(
        CATKIN_DEPENDS message_runtime transmit_wifi swarm_cmd
)

include_directories(
        ../../devel/include # Hack for Clion.
        ../../lib
        ${catkin_INCLUDE_DIRS}
)


add_executable(apply_swarm_patch
        PatchReceiver.cpp
        PatchReceiver.h
        PatchUtil.cpp
        PatchUtil.h)

add_executable(create_swarm_patch
        PatchTransmitter.cpp
        PatchTransmitter.h
        PatchUtil.cpp
        PatchUtil.h)

add_executable(create_full_swarm_patch
        FullPatchTransmitter.cpp
        FullPatchTransmitter.h
        PatchUtil.cpp
        PatchUtil.h)


add_executable(apply_full_swarm_patch
        FullPatchReceiver.cpp
        FullPatchReceiver.h
        PatchUtil.cpp
        PatchUtil.h)

add_dependencies(apply_swarm_patch ${catkin_EXPORTED_TARGETS})
add_dependencies(create_swarm_patch ${catkin_EXPORTED_TARGETS})
add_dependencies(create_full_swarm_patch ${catkin_EXPORTED_TARGETS})
add_dependencies(apply_full_swarm_patch ${catkin_EXPORTED_TARGETS})

target_link_libraries(apply_swarm_patch ${catkin_LIBRARIES} ${Boost_INCLUDE_DIRS} Boost::filesystem Boost::iostreams sha256 nlohmann_json)
target_link_libraries(create_swarm_patch ${catkin_LIBRARIES} ${Boost_INCLUDE_DIRS} Boost::filesystem Boost::iostreams sha256 nlohmann_json)
target_link_libraries(create_full_swarm_patch ${catkin_LIBRARIES} ${Boost_INCLUDE_DIRS} Boost::filesystem Boost::iostreams sha256 nlohmann_json)
target_link_libraries(apply_full_swarm_patch ${catkin_LIBRARIES} ${Boost_INCLUDE_DIRS} Boost::filesystem Boost::iostreams sha256 nlohmann_json pstream)

install(TARGETS nlohmann_json sha256 pstream
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )