// Server side C/C++ program to demonstrate Socket programming
#include <unistd.h>
#include <sys/socket.h>
#include <cstdlib>
#include <netinet/in.h>
#include <ros/ros.h>
#include <transmit_wifi/Transmission.h>
#include <errno.h>
#include <signal.h>
#include <unordered_map>
#include "WifiUtil.h"
using namespace std;
using namespace transmit_wifi;

#define SERVER_PORT 5001

unordered_map<string, int> connections;

int server_fd, client;
void handleInterrupt(int sig){
    ROS_WARN("[wifi_receiver] Closing open sockets");
    close(server_fd);
    close(client);
    exit(0);
}

void onResponseRequested(const Transmission::ConstPtr & msg) {
    ROS_INFO("[wifi_receiver] Sent reply of size %d to %s", msg->length, msg->connection.c_str());
    if(msg->connection != "~") return; // we only handle responses here
    transmit(client, msg);
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "wifi_receiver");
    ros::NodeHandle nodeHandle;
    ros::Subscriber sub = nodeHandle.subscribe("wifi_out", 1000, onResponseRequested);
    ros::Publisher pub = nodeHandle.advertise<transmit_wifi::Transmission>("wifi_in", 0);
    while(pub.getNumSubscribers() == 0) ros::spinOnce(); // wait for at least one subscriber

    struct sockaddr_in serverAddr;

    // We don't care what our IP address is but we are definitely listening on port 5001
    //    (this is because Docker registries happen to listen on 5000, and it's easy to remember)

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(SERVER_PORT);

    if((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        ROS_ERROR("[wifi_receiver] Socket failed: %s", strerror(errno));
        exit(EXIT_FAILURE);
    }
    ROS_INFO("Got socket file descriptor");

    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(int)) < 0) {
        ROS_WARN("Could not set server socket to reusable; you may have to wait after killing this node to restart it");
    }

    if(bind(server_fd, (struct sockaddr *) &serverAddr, sizeof(serverAddr)) < 0) {
        ROS_ERROR("[wifi_receiver] Bind failed: %s", strerror(errno));
        exit(EXIT_FAILURE);
    }
    ROS_INFO("[wifi_receiver] Bound socket to port %d", SERVER_PORT);

    if(listen(server_fd, 3) < 0) {
        ROS_ERROR("[wifi_receiver] Listen failed: %s", strerror(errno));
        exit(EXIT_FAILURE);
    }
    ROS_INFO("[wifi_receiver] Listening on port %d", SERVER_PORT);

    size_t addrSize = sizeof(serverAddr);

    // If wifi_receiver is killed the socket persists until its timeout, which is really irritating because it takes a few
    //   minutes before you can run the node again
    //   So we have to catch SIGINT for that.

    struct sigaction sa;

    sa.sa_handler = handleInterrupt;
    sa.sa_flags = 0; // or SA_RESTART
    sigemptyset(&sa.sa_mask);

    if (sigaction(SIGINT, &sa, nullptr) == -1) {
        ROS_ERROR("[wifi_receiver] sigaction: %s", strerror(errno));
        exit(1);
    }

    client = 0;

    while(ros::ok()) {
        if(client == 0) {
            // Look for a pending client
            if((client = accept(server_fd, (struct sockaddr *) &serverAddr,
                                   (socklen_t *) &addrSize)) < 0) {
                ROS_ERROR("[wifi_receiver] Could not accept a client.");
            } else ROS_INFO("[wifi_receiver] Received a client; processing traffic.");
        } else {
            processTraffic(client, &pub, "~");
            pollSocket(&client);
            ros::spinOnce();
        }
    }
}


