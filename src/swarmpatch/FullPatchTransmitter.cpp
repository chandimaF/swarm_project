//
// Created by mel on 7/12/21.
//

#include "FullPatchTransmitter.h"

#include <utility>
#include <transmit_wifi/Transmission.h>
#include <transmit_wifi/Connection.h>
#include "PatchUtil.h"
#include <swarmpatch/PatchRequest.h>

#define TIMEOUT 30000 // docker pull can take a while

using namespace std;

int main(int argc, char ** argv) {
    ros::init(argc, argv, "full_patch_transmitter");
//    ros::NodeHandle nh;

    auto * t = new FullPatchTransmitter("odroid", "10.42.0.1:5000/sizeteste", 1);

//    for(long i = 1; i < 10000000l; i *= 2) {
//        t->aim("odroid", "10.42.0.1:5000/sizeteste_" + to_string(i), 1);
//        t->inform();
//        t->awaitDone();
//    }

    ros::spin();
}


FullPatchTransmitter::FullPatchTransmitter(string t, string p, int v): target(std::move(t)), project(std::move(p)), version(v) {
    ROS_INFO("[full_patch_transmitter] Aiming new transmitter: %s v%d -> %s", p.c_str(), v, t.c_str());
    ros::NodeHandle nh;
    this->commandOut = nh.advertise<swarm_cmd::SwarmCommand>("command_out", 1000);
    // This is never used, but as far as I can tell it gets garbage'd or something otherwise?
    this->commandIn = nh.subscribe("command_in", 100, &FullPatchTransmitter::onStatusUpdate, this);
    this->requestIn = nh.subscribe("patch_requests", 100, &FullPatchTransmitter::onPatchRequest, this);
    while(commandOut.getNumSubscribers() == 0) ros::spinOnce(); // wait for subscribers to be ready
}

void FullPatchTransmitter::aim(string t, string p, int v) {
    ROS_INFO("[full_patch_transmitter] Re-aiming transmitter: %s v%d -> %s", p.c_str(), v, t.c_str());
    this->project = std::move(p);
    this->version = v;
    this->target = std::move(t);
}

void FullPatchTransmitter::inform() {
    ROS_INFO("[full_patch_transmitter] Informing target %s of new update (%s v%d)", target.c_str(), project.c_str(), version);
    swarm_cmd::SwarmCommand cmd;
    cmd.agent = target;
    cmd.data_length = project.length();
    cmd.data = vector<unsigned char>(project.c_str(), project.c_str() + project.length());
    cmd.type = 0b1u;
    cmd.order = version;
    commandOut.publish(cmd);
    targetStatus = TARGET_PULLING;
}

long startTime;
bool FullPatchTransmitter::awaitDone() const {
    startTime = millitime();
    while(targetStatus == TARGET_PULLING && millitime() < startTime + TIMEOUT) ros::spinOnce();
    ROS_INFO("[full_patch_transmitter] Full patch of %s complete; took %lu milliseconds", this->project.c_str(), millitime() - startTime);
    return targetStatus == TARGET_OK;
}

void FullPatchTransmitter::onStatusUpdate(const swarm_cmd::SwarmCommand::ConstPtr & msg) {
    ROS_INFO("[full_patch_transmitter] Status update from target %s", msg->agent.c_str());
    if(msg->type != 2) return;
    string response = string((char *) msg->data.data()+1, msg->data_length);

    // Very annoyingly, docker pull gives zero exit status no matter what.
    // So I went to all the work to transmit it and it's not even useful. :/
    // targetStatus = msg->data[0];
    if(response.find("Status: Downloaded") != string::npos || response.find("up to date") != string::npos) {
        targetStatus = TARGET_OK;
    } else targetStatus = 1;

    if(targetStatus != 0) {
        ROS_INFO("[full_patch_transmitter] Response as follows:\n");
        ROS_ERROR("%s", response.c_str());
        ROS_ERROR("\n[full_patch_transmitter] (Target failed to install latest version)");
    } else {
        ROS_INFO("[full_patch_transmitter] Response as follows:\n");
        ROS_INFO("%s", response.c_str());
        ROS_INFO("\n[full_patch_transmitter] (Target successfully installed latest version)");
    }
}

void FullPatchTransmitter::onPatchRequest(const swarmpatch::PatchRequest &msg) {
    if(! msg.full) return;
    aim(msg.target, msg.project, msg.version);
    inform();
    awaitDone();
}
