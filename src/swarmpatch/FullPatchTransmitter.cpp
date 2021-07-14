//
// Created by mel on 7/12/21.
//

#include "FullPatchTransmitter.h"

#include <utility>
#include <transmit_wifi/Transmission.h>
#include <transmit_wifi/Connection.h>
#include "PatchUtil.h"

#define TIMEOUT 10000 // docker pull can take a while

using namespace std;

int main(int argc, char ** argv) {
    ros::init(argc, argv, "full_patch_transmitter");
    ros::NodeHandle nh;

    // Step 1: connect to localhost (should really be there by default)
    transmit_wifi::Connection msg;
    msg.name = "local";
    msg.port = 5001;
    msg.ip = "127.0.0.1";
    ros::Publisher pub = nh.advertise<transmit_wifi::Connection>("connect", 1000);
    while(pub.getNumSubscribers() == 0) ros::spinOnce(); // wait for subscribers to be ready
    pub.publish(msg);

    for(int i = 512; i < 17000; i *= 2) {
        auto * t = new FullPatchTransmitter("127.0.0.1:5000/sizeteste_" + to_string(i), 1);
        t->setTarget("local");
        t->inform();
        t->awaitDone();
    }

}


FullPatchTransmitter::FullPatchTransmitter(string p, int v): project(std::move(p)), version(v) {
    ros::NodeHandle nh;
    this->commandOut = nh.advertise<swarm_cmd::SwarmCommand>("command_out", 1000);
    // This is never used, but as far as I can tell it gets garbage'd or something otherwise?
    this->commandIn = nh.subscribe("command_in", 100, &FullPatchTransmitter::onStatusUpdate, this);
    while(commandOut.getNumSubscribers() == 0) ros::spinOnce(); // wait for subscribers to be ready
}

void FullPatchTransmitter::setImage(string p, int v) {
    this->project = std::move(p);
    this->version = v;
}

void FullPatchTransmitter::setTarget(string t) {
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
    while(targetStatus == TARGET_PULLING || millitime() > startTime + TIMEOUT) ros::spinOnce();
    ROS_ERROR("[full_patch_transmitter] Full patch complete; took %lu milliseconds", millitime() - startTime);
    return targetStatus == TARGET_OK;
}

void FullPatchTransmitter::onStatusUpdate(const swarm_cmd::SwarmCommand::ConstPtr & msg) {
    ROS_INFO("[full_patch_transmitter] Status update from target!");
    if(msg->type != 2) return;
    string response = string((char *) msg->data.data()+1, msg->data_length);


    // Very annoyingly, docker pull gives zero exit status no matter what.
    // So I went to all the work to transmit it and it's not even useful. :/
    // targetStatus = msg->data[0];
    if(response.find("Status: Downloaded") != string::npos || response.find("up to date") != string::npos) {
        targetStatus = TARGET_OK;
    } else targetStatus = 1;

    if(targetStatus != 0) {
        ROS_WARN("[full_patch_transmitter] Response as follows:");
        ROS_WARN("%s", response.c_str());
        ROS_WARN("(exited with code %d)", targetStatus);
    } else {
        ROS_INFO("[full_patch_transmitter] Response as follows:");
        ROS_INFO("%s", response.c_str());
        ROS_INFO("(exited with code %d)", targetStatus);
    }
}
