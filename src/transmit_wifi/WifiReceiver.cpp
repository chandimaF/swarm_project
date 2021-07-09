// Server side C/C++ program to demonstrate Socket programming
#include <unistd.h>
#include <cstdio>
#include <csignal>
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

long totalMessages = 0;

void processTraffic(int socket, ros::Publisher pub);

int main(int argc, char ** argv) {
    ros::init(argc, argv, "wifi_receiver");
    ros::NodeHandle nodeHandle;
    ros::Publisher pub = nodeHandle.advertise<transmit_wifi::Transmission>("/wifi_in", 1000000);

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

    while(ros::ok()) {
        // Look for a pending client
        if((newSocket = accept(server_fd, (struct sockaddr *) &serverAddr,
                               (socklen_t *) &addrSize)) < 0) {
            ROS_WARN("Could not accept a client");
        }

        pid_t childProcess = fork();
        if (childProcess == -1) {
            perror("Unable to create new process for client connection");
            exit(1);
        }
        else if (childProcess == 0) {
            while(ros::ok()) {
                processTraffic(newSocket, pub);
            }
        }
        else {
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
                        // if recv returns zero, connection closed; kill the child process
                        int status = 0;
                        kill(childProcess, SIGKILL);
                        waitpid(childProcess, &status, WNOHANG);
                        close(newSocket);
                    }
                }
            }
        }
    }
}

void processTraffic(int sock, ros::Publisher pub) {

    size_t nRead;
    char buffer[1024] = {0}; // Messages are 1024 bytes long here - which is not always going to be true!

    if((nRead = read(sock, buffer, 1024)) > 0) {
        transmit_wifi::Transmission msg;
        vector<signed char> bytes = vector<signed char>(buffer, buffer + nRead);
        msg.data = bytes;
        msg.length = nRead;
        pub.publish(msg);
    }
}
