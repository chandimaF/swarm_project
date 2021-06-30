// Server side C/C++ program to demonstrate Socket programming
#include <unistd.h>
#include <cstdio>
#include <sys/socket.h>
#include <cstdlib>
#include <netinet/in.h>
#include <string>
#include <ros/ros.h>
#include <std_msgs/ByteMultiArray.h>

using namespace std;

#define SERVER_PORT 5000
#define CLIENT_PORT 5001

int main(int argc, char ** argv) {
    ros::init(argc, argv, "wifi_receiver");
    ros::NodeHandle nodeHandle;
    ros::Publisher pub = nodeHandle.advertise<std_msgs::ByteMultiArray>("/wifi_in", 1000);

    ros::spin();


    int server_fd, new_socket;
    struct sockaddr_in serverAddr;
    int opt = 1;
    char buffer[1024] = {0};

    string hello = "Hello from server";

    // Creating socket file descriptor
    if((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Forcefully attaching socket to the port 8080
    if(setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                  &opt, sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(SERVER_PORT);

    // Forcefully attaching socket to the port 8080
    if(bind(server_fd, (struct sockaddr *) &serverAddr, sizeof(serverAddr)) < 0) {
        perror("bind failed");
        ROS_WARN("Bind failed.");
        exit(EXIT_FAILURE);
    }
    if(listen(server_fd, 3) < 0) {
        perror("listen");
        ROS_WARN("Listen failed.");
        exit(EXIT_FAILURE);
    }
    size_t addrSize = sizeof(serverAddr);
    if((new_socket = accept(server_fd, (struct sockaddr *) &serverAddr,
                            (socklen_t *) &addrSize)) < 0) {
        perror("accept");
        ROS_WARN("Accept failed.");
        exit(EXIT_FAILURE);
    }

    size_t nRead;
    while(ros::ok()) {

        if((nRead = read(new_socket, buffer, 1024)) > 0) {
            std_msgs::ByteMultiArray msg = std_msgs::ByteMultiArray();
            for(int i = 0; i < nRead; i++) {
                msg.data[i] = buffer[i];
            }
            pub.publish(msg);
            ROS_INFO("Received message via existing network");
        }
    }
}
