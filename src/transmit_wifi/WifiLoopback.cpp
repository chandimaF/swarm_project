// Client side C/C++ program to demonstrate Socket programming
#include <ros/ros.h>
#include <transmit_wifi/Transmission.h>

using namespace std;

ros::Publisher pub;

void onWifiIn(const transmit_wifi::Transmission::ConstPtr & msg) {
    pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wifi_transmitter");
    ros::NodeHandle nodeHandle;
    pub = nodeHandle.advertise<transmit_wifi::Transmission>("/wifi_in", 0);
    ros::Subscriber sub = nodeHandle.subscribe("/wifi_out", 0, onWifiIn);

    while(ros::ok()) {
        ros::spin();
    }
}
