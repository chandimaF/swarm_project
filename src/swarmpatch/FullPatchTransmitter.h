//
// Created by miles on 7/12/21.
//

#ifndef SWARM_PROJECT_FULLPATCHTRANSMITTER_H
#define SWARM_PROJECT_FULLPATCHTRANSMITTER_H

#include <string>
#include <ros/ros.h>
#include <swarm_cmd/SwarmCommand.h>


#define TARGET_PULLING -1
#define TARGET_OK 0

using namespace std;

class FullPatchTransmitter {

    string project;
    int version;
    string target;
    ros::Publisher commandOut;
    ros::Subscriber commandIn;
    int targetStatus = 0;

public:
    FullPatchTransmitter(string project, int version);
    void setImage(string project, int version);
    void setTarget(string target);

    void inform();
    bool awaitDone() const;
    void onStatusUpdate(const swarm_cmd::SwarmCommand::ConstPtr &msg);
};


#endif //SWARM_PROJECT_FULLPATCHTRANSMITTER_H
