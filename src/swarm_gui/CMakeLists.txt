cmake_minimum_required(VERSION 3.0.2)
project(swarm_gui)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_CXX_STANDARD 14)
set(CURL_LIBRARY "-lcurl")

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(CURL REQUIRED)
find_package(wxWidgets COMPONENTS core base REQUIRED)

include( "${wxWidgets_USE_FILE}" )
include_directories(
        ../../devel/include # Hack for Clion.
        ../../lib
        ${catkin_INCLUDE_DIRS}
)

add_executable(swarm_project
        wxsmith/SwarmUIframe.wxs
        SwarmUIApp.cpp
        SwarmUIApp.h
        SoftwareDistributor.cpp
        SoftwareDistributor.h
        SwarmUIDeployment.cpp
        SwarmUIDeployment.h
        SwarmUIMain.cpp
        SwarmUIMain.h)

add_dependencies(swarm_project ${catkin_EXPORTED_TARGETS})
include_directories(${catkin_INCLUDE_DIRS})
target_link_libraries(swarm_project ${catkin_LIBRARIES} ${wxWidgets_LIBRARIES} ${CURL_LIBRARIES} nlohmann_json)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES test_pkg
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_lib
)


