// Client side C/C++ program to demonstrate Socket programming
#include <cstdio>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <ros/ros.h>
#include <std_msgs/ByteMultiArray.h>

using namespace std;

#define HOST "127.0.0.1"
#define SERVER_PORT 5000

void onTransmitRequested(const std_msgs::ByteMultiArray::ConstPtr & msg) {
    const signed char * bytes = msg.get()->data.data();

    int sock;
    struct sockaddr_in serverAddr;
    string hello = (char *) bytes; // this will definitely not cause issues later

    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        ROS_WARN("Socket creation error.");return;
    }

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(SERVER_PORT);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, HOST, &serverAddr.sin_addr) <= 0) {
        ROS_WARN("Invalid address."); return;
    }

    if (connect(sock, (struct sockaddr *) &serverAddr, sizeof(serverAddr)) < 0) {
        ROS_WARN("\nConnection Failed"); return;
    }

    send(sock, hello.c_str(), hello.size(), 0);
    ROS_INFO("Sent message via existing network");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wifi_transmitter");
    ros::NodeHandle nodeHandle;
    ros::Subscriber sub = nodeHandle.subscribe("/wifi_out", 100, onTransmitRequested);

    ros::spin();
}
