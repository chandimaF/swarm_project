//
// Created by miles on 7/8/21.
//

#ifndef SWARM_PROJECT_PATCHTRANSMITTER_H
#define SWARM_PROJECT_PATCHTRANSMITTER_H

#include <ros/ros.h>
#include <swarm_cmd/SwarmCommand.h>
#include <swarmpatch/PatchRequest.h>

#define TARGET_PULLING -1
#define TARGET_OK 0
#define TIMEOUT 5000

using namespace std;

class PatchTransmitter {

    ros::Publisher commandOut;
    ros::Subscriber commandIn;
    ros::Subscriber requestIn;
    string target;
    string project;
    int version;
    int targetStatus = 0;

public:
    PatchTransmitter(string target, string project, int version);

    void aim(string target, string project, int version);
    void pack();
    void transmit();
    bool awaitDone() const;
    void onStatusUpdate(const swarm_cmd::SwarmCommand::ConstPtr &msg);
    void onPatchRequest(const swarmpatch::PatchRequest &msg);

};


#endif //SWARM_PROJECT_PATCHTRANSMITTER_H
