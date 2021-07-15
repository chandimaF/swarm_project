//
// Created by miles on 7/6/21.
//

#ifndef SWARM_PROJECT_PATCHRECEIVER_H
#define SWARM_PROJECT_PATCHRECEIVER_H

#include <string>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <swarm_cmd/SwarmCommand.h>

#define DEFAULT_SWARM_DIR "/home/miles/swarmpatch"

using namespace std;

class PatchReceiver {
public:

    ros::Subscriber cmdIn;
    string project;
    int version;

    PatchReceiver(string project, int version);
    void unpack() const;
    void build() const;
    void apply() const;
    void onIncomingCommand(const swarm_cmd::SwarmCommand::ConstPtr & cmd) const;
};


#endif //SWARM_PROJECT_PATCHRECEIVER_H
