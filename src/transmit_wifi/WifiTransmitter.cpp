// Client side C/C++ program to demonstrate Socket programming
#include <cstdio>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <ros/ros.h>
#include <transmit_wifi/Transmission.h>

using namespace std;

#define HOST "127.0.0.1"
#define SERVER_PORT 5000
#define CLIENT_PORT 5001

int sock; // I solemnly swear I will wrap this all in a class at the next possible refactoring opportunity

void onTransmitRequested(const transmit_wifi::Transmission::ConstPtr & msg) {
    size_t nbytes = msg->data.size();
    const signed char * bytes = msg.get()->data.data();
    send(sock, bytes, nbytes, 0);
    ROS_INFO("Sent message via existing network");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wifi_transmitter");
    ros::NodeHandle nodeHandle;

    // Transmitter is the client. (only two devices supported though.)
    struct sockaddr_in serverAddr, clientAddr;

    // Get a socket file pointer
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        ROS_WARN("Socket creation error."); return 1;
    }
    ROS_INFO("Got socket file descriptor");


    // Note how we don't bind() here - we don't really care what the transmitter (outgoing) port is.
    // We do care what the server port is, because it'll be listening on 5000.
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(SERVER_PORT);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, HOST, &serverAddr.sin_addr) <= 0) {
        ROS_WARN("Invalid address."); return 1;
    }

    ROS_INFO("Attempting connection to server...");
    // Connect socket to server!
    if (connect(sock, (struct sockaddr *) &serverAddr, sizeof(serverAddr)) < 0) {
        perror("Connection failed");
        ROS_WARN("\nConnection Failed"); return 1;
    }
    ROS_INFO("Socket connected to server");

    // We want to transmit out of that socket into /wifi_out as needed.
    ros::Subscriber sub = nodeHandle.subscribe("/wifi_out", 100, onTransmitRequested);

    // I don't *think* this socket getting closed spontaneously will be a problem. But it might be better to set
    //   SO_KEEPALIVE? We'll see.

    ros::spin();
}
