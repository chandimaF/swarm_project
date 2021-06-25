cmake_minimum_required(VERSION 3.19)
project(swarm_project)

set(CMAKE_CXX_STANDARD 14)
set(CURL_LIBRARY "-lcurl")
find_package(CURL REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

find_package(wxWidgets COMPONENTS core base REQUIRED)
include( "${wxWidgets_USE_FILE}" )

include_directories(.)


add_executable(swarm_project
        wxsmith/SwarmUIframe.wxs
        README.md
        SwarmUIApp.cpp
        SwarmUIApp.h
        SoftwareDistributor.cpp
        SoftwareDistributor.h
        SwarmUIDeployment.cpp
        SwarmUIDeployment.h
        SwarmUIMain.cpp
        SwarmUIMain.h)

add_executable(distributor
        SoftwareDistributor.cpp
        SoftwareDistributor.h
        )


target_link_libraries(swarm_project ${wxWidgets_LIBRARIES} ${CURL_LIBRARIES} nlohmann_json::nlohmann_json)
target_link_libraries(distributor ${CURL_LIBRARIES} nlohmann_json::nlohmann_json)

