// Server side C/C++ program to demonstrate Socket programming
#include <unistd.h>
#include <cstdio>
#include <sys/socket.h>
#include <cstdlib>
#include <netinet/in.h>
#include <string>
#include <ros/ros.h>
#include <transmit_wifi/Transmission.h>

using namespace std;

#define SERVER_PORT 5000

int main(int argc, char ** argv) {
    ros::init(argc, argv, "wifi_receiver");
    ros::NodeHandle nodeHandle;
    ros::Publisher pub = nodeHandle.advertise<transmit_wifi::Transmission>("/wifi_in", 1000);

    int server_fd, new_socket;
    struct sockaddr_in serverAddr;
    char buffer[1024] = {0}; // Messages are 1024 bytes long here - which is not always going to be true!

    // We don't care what our IP address is but we are definitely listening on port 5000
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(SERVER_PORT);

    // We'll just transmit some chars to test
    string hello = "bytes to transmit";

    // Create the socket file descriptor
    if((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }
    ROS_INFO("Got socket file descriptor");

    if(bind(server_fd, (struct sockaddr *) &serverAddr, sizeof(serverAddr)) < 0) {
        perror("bind failed");
        ROS_WARN("Bind failed.");
        exit(EXIT_FAILURE);
    }
    ROS_INFO("Bound socket to port 5000");

    if(listen(server_fd, 3) < 0) {
        perror("listen");
        ROS_WARN("Listen failed.");
        exit(EXIT_FAILURE);
    }
    ROS_INFO("Listening on port 5000");

    size_t addrSize = sizeof(serverAddr);

    int current_client = 0;
    bool has_client = false; // there are probably acceptable sentinals within socket fds (can they be negative?)
                                // but not taking my chances

    size_t nRead;
    while(ros::ok()) {

        // Look for a pending client
        if((new_socket = accept(server_fd, (struct sockaddr *) &serverAddr,
                                (socklen_t *) &addrSize)) < 0) {
            ROS_WARN("Could not accept a client");
        } else {
            has_client = true;
            current_client = new_socket; // we kick old clients out as soon as there are new ones
            ROS_INFO("Connected to client on a new port");
        }

        // Read off of the current client
        if(has_client && (nRead = read(current_client, buffer, 1024)) > 0) {
            ROS_INFO("Received message via existing network");
            transmit_wifi::Transmission msg;
            auto bytes = vector<signed char>(buffer, buffer + nRead);
            msg.data = bytes;
            msg.length = nRead;
            pub.publish(msg);
            ROS_INFO("Published received message");
        }
    }
}
