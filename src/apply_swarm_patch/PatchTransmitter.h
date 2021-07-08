//
// Created by miles on 7/8/21.
//

#ifndef SWARM_PROJECT_PATCHTRANSMITTER_H
#define SWARM_PROJECT_PATCHTRANSMITTER_H

#include <ros/ros.h>
#define DEFAULT_SWARM_DIR "/home/miles/swarmpatch"

using namespace std;

class PatchTransmitter {

    ros::Publisher pub;
    string swarmDir;
    const string & project;
    int version;

public:
    PatchTransmitter(const string & project, int version, ros::Publisher & output);
    void checkPaths();
    void pack();
    void transmit();
};


#endif //SWARM_PROJECT_PATCHTRANSMITTER_H
