// Server side C/C++ program to demonstrate Socket programming
#include <unistd.h>
#include <cstdio>
#include <sys/socket.h>
#include <cstdlib>
#include <netinet/in.h>
#include <string>
#include <ros/ros.h>
#include <transmit_wifi/Transmission.h>
#include <sys/poll.h>
#include <sys/wait.h>

using namespace std;

#define SERVER_PORT 5000

// Thunder and <global variables>, very very frightening
//    (but there's a good reason for this one; it's nice for debugging)
long totalMessages = 0;

void processTraffic(int socket, ros::Publisher * pub);

int main(int argc, char ** argv) {
    ros::init(argc, argv, "wifi_receiver");
    ros::NodeHandle nodeHandle;
    ros::Publisher pub = nodeHandle.advertise<transmit_wifi::Transmission>("/wifi_in", 0);
    if(! pub) {
        ROS_WARN("Could not get publisher for WiFi receiver!");
    }

    int server_fd, newSocket;
    struct sockaddr_in serverAddr;

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

    int currentClient = 0;

    size_t nRead;
    newSocket = 0;

    while(ros::ok()) {
        if(newSocket == 0) {
            // Look for a pending client
            if((newSocket = accept(server_fd, (struct sockaddr *) &serverAddr,
                                   (socklen_t *) &addrSize)) < 0) {
                ROS_WARN("Could not accept a client");
            } else {
                ROS_INFO("Received a client; processing traffic");
            }
        } else {
            processTraffic(newSocket, &pub);

            // use the poll system call to be notified about socket status changes
            struct pollfd pfd;
            char buffer[1024] = {0};
            pfd.fd = newSocket;
            pfd.events = POLLIN | POLLHUP | POLLRDNORM;
            pfd.revents = 0;
            while (pfd.revents == 0 && ros::ok()) {
                // call poll with a timeout of 100 ms
                if (poll(&pfd, 1, 100) > 0) {
                    // if result > 0, this means that there is either data, or socket is closed
                    if (recv(newSocket, buffer, sizeof(buffer), MSG_PEEK | MSG_DONTWAIT) == 0) {
                        close(newSocket);
                        newSocket = 0;
                    }
                }
            }
        }
    }
}

void processTraffic(int sock, ros::Publisher * pub) {

    size_t nRead;
    char buffer[2052] = {0};

    if((nRead = read(sock, buffer, 2052)) > 0) {
        cout << "Incoming transmission...\n";
        transmit_wifi::Transmission msg;
        cout << "> Gathering bytes...\n";
        vector<signed char> bytes = vector<signed char>(buffer, buffer + nRead);
        cout << "> Making message...\n";
        msg.data = bytes;
        msg.length = nRead;
        cout << "> Publishing message...\n";
        pub->publish(msg);
        cout << "> Transmission fully received\n";
    }
}
