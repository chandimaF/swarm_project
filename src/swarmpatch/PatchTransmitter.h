//
// Created by miles on 7/8/21.
//

#ifndef SWARM_PROJECT_PATCHTRANSMITTER_H
#define SWARM_PROJECT_PATCHTRANSMITTER_H

#include <ros/ros.h>

using namespace std;

class PatchTransmitter {

    ros::Publisher * pub;
    const string project;
    int version;

public:
    PatchTransmitter(string project, int version, ros::Publisher * output);
    void pack();
    void transmit();
};


#endif //SWARM_PROJECT_PATCHTRANSMITTER_H
