cmake_minimum_required(VERSION 3.0.2)
project(transmit_wifi)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation
        )


include_directories(
        ../../devel/include # Hack for Clion.
        ${catkin_INCLUDE_DIRS}
)

add_message_files(
        FILES
        Transmission.msg
)
generate_messages(
        DEPENDENCIES
        std_msgs
        transmit_wifi
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES test_pkg
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_libcatkin_package(

        CATKIN_DEPENDS message_runtime
)

add_executable(wifi_receiver
        WifiReceiver.cpp
        )

add_executable(wifi_transmitter
        WifiTransmitter.cpp
        )

add_executable(wifi_loopback
        WifiLoopback.cpp
        )

target_link_libraries(wifi_receiver ${catkin_LIBRARIES})
target_link_libraries(wifi_transmitter ${catkin_LIBRARIES})
target_link_libraries(wifi_loopback ${catkin_LIBRARIES})
