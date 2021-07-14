#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <ros/ros.h>
#include <transmit_wifi/Transmission.h>
#include <transmit_wifi/Connection.h>
#include <unordered_map>
#include "WifiUtil.h"

using namespace std;

//#define HOST "10.42.0.190"
// //#define HOST "127.0.0.1"
//#define SERVER_PORT 5000
//#define CLIENT_PORT 5001

unordered_map<string, int> connections;

void onConnectionRequested(const transmit_wifi::Connection::ConstPtr & msg) {
    ROS_INFO("[wifi_transmitter] Connecting to '%s' (%s:%d)", msg->name.c_str(), msg->ip.c_str(), msg->port);

    if(connections.count(msg->name) != 0) {
        ROS_WARN("[wifi_transmitter] Connection '%s' already exists", msg->name.c_str());
        close(connections[msg->name]);
    }

    int sock;

    // Transmitter is the client in this case.
    // This seems a little odd at first, but remember that listening is generally the job of the agents, not the
    //     ground station.
    struct sockaddr_in serverAddr;

    // Get a socket file pointer
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        ROS_ERROR("[wifi_transmitter] Socket creation error.");
        return;
    }
    ROS_INFO("[wifi_transmitter] Got socket file descriptor");

    // Note how we don't bind() here - we don't really care what the transmitter (outgoing) port is.
    // We do care what the server port is, because it'll be listening on 5000.
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(msg->port);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, msg->ip.c_str(), &serverAddr.sin_addr) <= 0) {
        ROS_ERROR("[wifi_transmitter]   Invalid address.");
        return;
    }

    ROS_INFO("[wifi_transmitter] Attempting connection to %s:%d", msg->ip.c_str(), msg->port);

    // Connect socket to server!
    if (connect(sock, (struct sockaddr *) &serverAddr, sizeof(serverAddr)) < 0) {
        ROS_ERROR("[wifi_transmitter] Connection failed: %s", strerror(errno));
        return;
    }

    ROS_INFO("[wifi_transmitter] Socket connected to server");
    connections[msg.get()->name] = sock;
}

void onTransmitRequested(const transmit_wifi::Transmission::ConstPtr & msg) {
    ROS_INFO("[wifi_transmitter] Sending message of size %d to %s", msg->length, msg->connection.c_str());
    if(msg->connection == "~") return; // special case for replies to the ground station - receiver should handle those
    if(connections.count(msg->connection) == 0) ROS_ERROR("[wifi_transmitter] No connection named '%s'", msg->connection.c_str());

    transmit(connections[msg->connection], msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wifi_transmitter");
    ros::NodeHandle nodeHandle;

    // We want to transmit out of that socket into /wifi_out as needed.
    ros::Subscriber sub1 = nodeHandle.subscribe("wifi_out", 10000, onTransmitRequested);
    ros::Subscriber sub2 = nodeHandle.subscribe("connect", 10, onConnectionRequested);
    ros::Publisher responsePub = nodeHandle.advertise<transmit_wifi::Transmission>("wifi_in", 0);

    ROS_INFO("[wifi_transmitter] Awaiting connections and traffic");

    while(ros::ok()) {
        for(auto const & s: connections) {
            string name = s.first;
            int sock = s.second;
            processTraffic(sock, &responsePub, name);
            pollSocket(&sock);
        }
        ros::spinOnce();
    }
}
