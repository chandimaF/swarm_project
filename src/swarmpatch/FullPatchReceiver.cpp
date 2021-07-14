//
// Created by miles on 7/12/21.
//

#include "FullPatchReceiver.h"
#include <pstream.cpp>
#include <string>
#include <iostream>

std::string exec(const char* cmd, unsigned char * status) {
    redi::ipstream proc(cmd, redi::pstreams::pstdout | redi::pstreams::pstderr);
    std::string line;
    std::string result;

    while (std::getline(proc.out(), line)) result += "OUT: " + line + "\n";
    if (proc.eof() && proc.fail()) proc.clear();
    while (std::getline(proc.err(), line)) result += "ERR: " + line + "\n";
    *status = proc.rdbuf()->status();
    return result;
}

void FullPatchReceiver::onIncomingCommand(const swarm_cmd::SwarmCommand::ConstPtr & msg) {
    if(msg->type != 1) return; // we only care about patch requests
    ROS_INFO("[full_patch_receivers] Received a patch command from '%s' ('~' is ground)", msg->agent.c_str());


    std::string image(msg->data.begin(), msg->data.end());
    unsigned char status = 0;
    std::string error = exec(("docker pull " + image).c_str(), &status);

    swarm_cmd::SwarmCommand cmd;
    auto * data = new unsigned char[error.length() + 1];
    const char * cError = error.c_str();
    memcpy(data+1, cError, error.length());
    data[0] = status;

    cmd.data = std::vector<unsigned char>(data, data+error.length());
    cmd.type = 2;
    cmd.data_length = error.length();
    cmd.order = 0;
    cmd.agent = msg->agent; // special case for a response to ground
    cmdPub.publish(cmd);
}

FullPatchReceiver::FullPatchReceiver() {
    ros::NodeHandle nh;
    this->cmdSub = nh.subscribe("command_in", 100, &FullPatchReceiver::onIncomingCommand, this);
    this->cmdPub = nh.advertise<swarm_cmd::SwarmCommand>("command_out", 100);
    while(cmdPub.getNumSubscribers() == 0) ros::spinOnce();
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "full_patch_receiver");
    FullPatchReceiver p = FullPatchReceiver();
    ros::spin();
}


