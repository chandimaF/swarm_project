//
// Created by miles on 7/12/21.
//

#include <ros/ros.h>
#include <transmit_wifi/Transmission.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/poll.h>

using namespace std;

void processTraffic(int sock, ros::Publisher * pub, string name) {
    if(pub->getNumSubscribers() == 0) {
        ROS_WARN("[wifi_util] No subscribers to given wifi publisher; waiting for one...");
        while(pub->getNumSubscribers() == 0) ros::spinOnce();
        ROS_WARN("[wifi_util]   Subscriber found; OK");
    }

    size_t nRead;
    unsigned char buffer[2049] = {0};

    if((nRead = read(sock, buffer, 2049)) > 0) {
        ROS_INFO("[wifi_receiver] Incoming transmission...");
        transmit_wifi::Transmission msg;
        ROS_DEBUG("[wifi_receiver]   Gathering bytes");
        auto bytes = vector<unsigned char>(buffer, buffer + nRead);
        ROS_DEBUG("[wifi_receiver]   Making message");
        msg.data = bytes;
        msg.length = nRead;
        msg.connection = name;
        ROS_DEBUG("[wifi_receiver]   Publishing message");
        pub->publish(msg);
        ROS_DEBUG("[wifi_receiver]   Transmission fully received");
    }
}

void pollSocket(int sock) {
    struct pollfd pfd;
    unsigned char buffer[1024] = {0};
    pfd.fd = sock;
    pfd.events = POLLIN | POLLHUP | POLLRDNORM;
    pfd.revents = 0;
    while (pfd.revents == 0 && ros::ok()) {
        // call poll with a timeout of 100 ms
        if (poll(&pfd, 1, 100) > 0) {
            // if result > 0, this means that there is either data, or socket is closed
            if (recv(sock, buffer, sizeof(buffer), MSG_PEEK | MSG_DONTWAIT) == 0) {
                close(sock);
                sock = 0;
            }
        }
    }
}

void transmit(int sock, const transmit_wifi::Transmission::ConstPtr & msg) {
    size_t nbytes = msg->data.size();
    const unsigned char * bytes = msg.get()->data.data();
    send(sock, bytes, nbytes, 0);
}