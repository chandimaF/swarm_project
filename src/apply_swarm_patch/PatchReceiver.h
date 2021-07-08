//
// Created by miles on 7/6/21.
//

#ifndef SWARM_PROJECT_PATCHRECEIVER_H
#define SWARM_PROJECT_PATCHRECEIVER_H

#include <string>
#include <boost/shared_ptr.hpp>
#include <transmit_wifi/Transmission.h>
#include <ros/ros.h>

#define DEFAULT_SWARM_DIR "/home/miles/swarmpatch"

using namespace std;

class PatchReceiver {
public:
    ros::Subscriber sub;
    string swarmDir;
    const string & project;
    int version;


    PatchReceiver(const string &project, int version, ros::Subscriber & sub);
    void dumpBytes(unsigned char *bytes);
    void unpack();
    void build();
    void apply();
    void checkPaths();
    void onIncomingChunk(const boost::shared_ptr<const transmit_wifi::Transmission_<allocator<void>>> &msg);
};


#endif //SWARM_PROJECT_PATCHRECEIVER_H
