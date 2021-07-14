//
// Created by miles on 7/12/21.
//


#ifndef SWARM_PROJECT_WIFIUTIL_H
#define SWARM_PROJECT_WIFIUTIL_H

#include <ros/ros.h>
#include <transmit_wifi/Transmission.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/poll.h>

using namespace std;

void processTraffic(int sock, ros::Publisher * pub, string name);
void pollSocket(int * sock);
void transmit(int sock, const transmit_wifi::Transmission::ConstPtr & msg);

#endif