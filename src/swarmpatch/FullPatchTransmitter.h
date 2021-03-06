//
// Created by miles on 7/12/21.
//

#ifndef SWARM_PROJECT_FULLPATCHTRANSMITTER_H
#define SWARM_PROJECT_FULLPATCHTRANSMITTER_H

#include <string>
#include <ros/ros.h>
#include <swarm_cmd/SwarmCommand.h>
#include <swarmpatch/PatchRequest.h>


#define TARGET_PENDING -1
#define TARGET_OK 0

using namespace std;

class FullPatchTransmitter {

    ros::Publisher commandOut;
    ros::Subscriber commandIn;
    ros::Subscriber requestIn;
    string target;
    string project;
    int version;
    int targetStatus = 0;

public:
    FullPatchTransmitter(string target, string project, int version);

    void aim(string target, string project, int version);
    void inform();
    bool awaitDone() const;
    void onStatusUpdate(const swarm_cmd::SwarmCommand::ConstPtr &msg);
    void onPatchRequest(const swarmpatch::PatchRequest_<allocator<void>> &msg);
};


#endif //SWARM_PROJECT_FULLPATCHTRANSMITTER_H
