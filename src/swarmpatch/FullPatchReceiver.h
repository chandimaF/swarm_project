//
// Created by miles on 7/12/21.
//

#ifndef SWARM_PROJECT_FULLPATCHRECEIVER_H
#define SWARM_PROJECT_FULLPATCHRECEIVER_H

#include <transmit_wifi/Transmission.h>
#include <swarm_cmd/SwarmCommand.h>
#include <ros/ros.h>

class FullPatchReceiver {
public:
    ros::Subscriber cmdSub;
    ros::Publisher cmdPub;

    FullPatchReceiver();
    void onIncomingCommand(const swarm_cmd::SwarmCommand::ConstPtr & msg);
};


#endif //SWARM_PROJECT_FULLPATCHRECEIVER_H
