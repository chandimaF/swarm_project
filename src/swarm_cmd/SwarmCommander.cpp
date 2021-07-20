//
// Created by miles on 7/12/21.
//

#include <ros/ros.h>
#include <swarm_cmd/SwarmCommand.h>
#include <transmit_wifi/Transmission.h>

using namespace swarm_cmd;
using namespace std;

ros::Publisher outPub;
ros::Publisher commandInPub;

void onCommandRequested(const SwarmCommand::ConstPtr & cmd) {

    unsigned int nBytes = cmd->data_length + 4 + 4 + 1;
    auto * transmission = new unsigned char[nBytes];

    // Fun fact: Our messages are going to be big endian.
    //    I don't know enough about architecture to justify this decision. It's arbitrary.

    transmission[0] = cmd->type; // single byte is (I hope) endian-safe
    transmission[1] = cmd->order >> 24 & 0xFF;
    transmission[2] = cmd->order >> 16 & 0xFF;
    transmission[3] = cmd->order >> 8  & 0xFF;
    transmission[4] = cmd->order       & 0xFF;
    transmission[5] = cmd->data_length >> 24 & 0xFF;
    transmission[6] = cmd->data_length >> 16 & 0xFF;
    transmission[7] = cmd->data_length >> 8  & 0xFF;
    transmission[8] = cmd->data_length       & 0xFF;
    memcpy(transmission+9, cmd->data.data(), cmd->data_length);

    transmit_wifi::Transmission out;
    out.connection = cmd->agent;
    out.data = vector<unsigned char>(transmission, transmission+nBytes);
    out.length = nBytes;

    ROS_INFO("[command_swarm] Commanding agent '%s' (code %d): #%d, %d bytes of data",
             cmd->agent.c_str(), cmd->type, cmd->order, cmd->data_length);

    outPub.publish(out);
}

void onCommandReceived(const transmit_wifi::Transmission & msg) {

    if(msg.length < 9) {
        ROS_WARN("[command_swarm] Received %d bytes, which is not enough for a command", msg.length);
        return;
    }
    SwarmCommand cmd;
    const unsigned char * transmission = msg.data.data();

    cmd.type = transmission[0];
    cmd.order = (transmission[1] << 24) + (transmission[2] << 16) + (transmission[3] << 8) + (transmission[4]);
    cmd.data_length = (transmission[5] << 24) + (transmission[6] << 16) + (transmission[7] << 8) + (transmission[8]);
    cmd.data = vector<unsigned char>(transmission + 9, transmission+9+cmd.data_length);
    cmd.agent = msg.connection;

    ROS_INFO("[command_swarm] Parsed a command (code %d): #%d, %d bytes of data",
             cmd.type, cmd.order, cmd.data_length);

    commandInPub.publish(cmd);
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "command_swarm");
    ROS_INFO("[command_swarm] Node initialized");
    ros::NodeHandle nh;

    outPub = nh.advertise<transmit_wifi::Transmission>("radio_out", 1000);
    commandInPub = nh.advertise<SwarmCommand>("command_in", 1000);

    if(outPub.getNumSubscribers() == 0) {
        ROS_WARN("[command_swarm] Nobody is listening to outgoing transmissions; waiting...");
    }
    while(outPub.getNumSubscribers() == 0) ros::spinOnce();

    ROS_INFO("[command_swarm] Awaiting commands");
    ros::Subscriber wifiIn = nh.subscribe("wifi_in", 1000, onCommandReceived);
    ros::Subscriber radioIn = nh.subscribe("radio_in", 1000, onCommandReceived);
    ros::Subscriber commandsOut = nh.subscribe("command_out", 1000, onCommandRequested);

    ros::spin();
    return 0;
}