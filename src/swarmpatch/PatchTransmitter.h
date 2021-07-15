//
// Created by miles on 7/8/21.
//

#ifndef SWARM_PROJECT_PATCHTRANSMITTER_H
#define SWARM_PROJECT_PATCHTRANSMITTER_H

#include <ros/ros.h>

using namespace std;

class PatchTransmitter {

    ros::Publisher pub;
    string target;
    string project;
    int version;

public:
    PatchTransmitter(string target, string project, int version);

    void aim(string target, string project, int version);
    void pack();
    void transmit();
};


#endif //SWARM_PROJECT_PATCHTRANSMITTER_H
