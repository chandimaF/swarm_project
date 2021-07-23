//
// Created by miles on 7/8/21.
//


#include <fstream>
#include <boost/filesystem.hpp>
#include <iostream>
#include "PatchTransmitter.h"
#include <json.cpp>
#include <utility>
#include "PatchUtil.h"
#include <swarm_cmd/SwarmCommand.h>
#include <sys/stat.h>

using namespace std;
using json = nlohmann::json;

swarm_cmd::SwarmCommand * chunks;

int main(int argc, char ** argv) {
    ros::init(argc, argv, "patch_transmitter");
    auto * pt = new PatchTransmitter("odroid_agent", "sizeteste_256", 2);
    ros::NodeHandle nh;
    ros::Publisher patches = nh.advertise<swarmpatch::PatchRequest>("patch_requests", 100);
    while(patches.getNumSubscribers() == 0) ros::spinOnce();

    for(int i = 2048*2*2*2; i < 2048*2*2*2+1; i*=2) { //536870911
        ROS_INFO("========================================");
        ROS_INFO("STARTING TRIAL %d", i);
        ROS_INFO("========================================");
        pt->aim("odroid_agent", "sizeteste_" + to_string(i), 2);

        pt->pack();
        pt->transmit();
        ROS_INFO("========================================");
        ROS_INFO("FINISHED TRIAL %d", i);
        ROS_INFO("========================================");

        ros::Duration(5).sleep();

    }

//    ros::spin();
}

PatchTransmitter::PatchTransmitter(string t, string p, int v): target(t), project(p), version(v) {
    ros::NodeHandle nh;
    this->commandOut = nh.advertise<swarm_cmd::SwarmCommand>("command_out", 1000, true);
    this->commandIn = nh.subscribe("command_in", 100, &PatchTransmitter::onStatusUpdate, this);
    this->requestIn = nh.subscribe("patch_requests", 100, &PatchTransmitter::onPatchRequest, this);
    ROS_INFO("[patch_transmitter] Initialized transmitter for %s v%d -> %s ", p.c_str(), v, t.c_str());
}

void PatchTransmitter::aim(string t, string p, int v){
    ROS_INFO("[patch_transmitter] Aiming transmitter: %s v%d -> %s", p.c_str(), v, t.c_str());
    target = std::move(t);
    project = std::move(p);
    version = v;

    // aim the receiver too
    swarm_cmd::SwarmCommand cmd;
    auto * data = (unsigned char *) project.c_str();
    cmd.type = 5;
    cmd.data = vector<unsigned char>(data, data+project.length());
    cmd.order = version;
    cmd.data_length = project.length();
    cmd.agent = target;
    targetStatus = TARGET_PENDING;
    commandOut.publish(cmd);
    awaitDone();
}

void PatchTransmitter::pack() {
    // Save the layer from docker to SWARM_DIR/packs/version.

    checkPaths(project);
    string swarmDir = getSwarmDir();
    ROS_INFO("[patch_transmitter] Packing project %s v%d", project.c_str(), version);

    string path = swarmDir+"/packs/"+project;
    chdir(path.c_str());

    // Take the example "myproject" v<#>. This is what these commands will do:
    // "docker save myproject:latest > myproject.tar": Dump a tar archive of ALL layers to packs/myproject/myproject.tar
    system(("docker save " + project + ":latest > " + project + ".tar").c_str());
//    system(("docker rmi " + project + "_compiled").c_str());
//    system("rm Dockerfile");


    // "tar -xf myproject.tar -C 3": Extract contents to packs/myproject/<#>.
    boost::filesystem::create_directory(swarmDir + "/packs/" + project + "/" + to_string(version));
    system(("tar -xf "+project+".tar -C "+to_string(version)).c_str());

    // "rm myproject.tar": remove the extraneous tar file after extracting it.
    system(("rm "+project+".tar").c_str());

    // The extracted archive has a "manifest.json" that tells us what the layers are and what order they go in.
    json j; // i hope that whoever nlohmann is, they're having a wonderful day
    ifstream file(swarmDir+"/packs/"+project+"/"+to_string(version)+"/manifest.json", fstream::in);
    file >> j;
    file.close();

    if(j[0].is_null())
        ROS_ERROR("[patch_transmitter] Could not get a Docker manifest for %s", project.c_str());
    if(j[0]["Layers"].is_null())
        ROS_ERROR("[patch_transmitter] Malformed Docker manifest for project %s", project.c_str());
    if(j[0]["Layers"][version-1].is_null())
        ROS_ERROR("[patch_transmitter] Project %s does not have version v%d)", project.c_str(), version);

    string layer = j[0]["Layers"][version - 1];

    // After picking the layer name from the layers list, zip it and move it to packs/myproject/<#>.tar.gz.
    string layerPath = swarmDir + "/packs/" + project + "/" + to_string(version) + "/" + layer;
    string archivePath = swarmDir + "/packs/" + project + "/" + to_string(version) + ".tar.gz";

    ROS_DEBUG("[patch_transmitter] Zipping layer at %s to archive %s", layerPath.c_str(), archivePath.c_str());
    system(("gzip -c " + layerPath + " > " + archivePath).c_str());

    // I was doing this with Boost, but frankly, I hate working with tar files; they yield eof in weird places and
    //    the format seems inconsistent. Thus the use of gzip.
}

long filesize(std::string & filename)
{
    struct stat stat_buf{};
    int rc = stat(filename.c_str(), &stat_buf);
    return rc == 0 ? stat_buf.st_size : -1;
}

void PatchTransmitter::transmit() {
    // Send a command containing the patch.

    checkPaths(project);
    string swarmDir = getSwarmDir();
    ROS_INFO("[patch_transmitter] Transmitting project %s v%d -> %s", project.c_str(), version, target.c_str());

    string archivePath = swarmDir + "/packs/" + project + "/" + to_string(version) + ".tar.gz";
    std::streamsize nBytes = filesize(archivePath);
    ifstream file(archivePath, ifstream::in | ofstream::binary);
    auto * buffer = (char *) calloc(nBytes+1, 1);
    ROS_INFO("[patch_transmitter] Patch appears to have size %lu", nBytes);
    file.read(buffer, nBytes);

    long nChunks = nBytes / 119 + 1;

    swarm_cmd::SwarmCommand header;
    header.type = 7;
    header.data_length = 0;
    header.agent = target;
    header.order = nChunks;
    targetStatus = TARGET_PENDING;
    commandOut.publish(header);

    awaitDone();

    chunks = new swarm_cmd::SwarmCommand[nChunks];

    targetStatus = TARGET_PENDING;
    for(int i = 0; i < nChunks; i++) {
        ros::spinOnce(); // this should make the code below reachable, right?
        chunks[i].type = 3;
        chunks[i].data_length = (i == nChunks-1)? nBytes%119:119;
        chunks[i].agent = target;
        chunks[i].order = i;
        chunks[i].data = vector<unsigned char>(buffer + i*119, buffer + i*119 + chunks[i].data_length);
        commandOut.publish(chunks[i]);
        if(! awaitDone()) {
            ROS_WARN("Didn't get ACK'd on chunk %d; trying again", i);
            i--;
        }
    }

    ros::Duration(1).sleep();
    swarm_cmd::SwarmCommand done;
    done.type = 9;
    done.data_length = 1;
    done.agent = target;
    done.order = 0;
    done.data = {1};
    commandOut.publish(done);

    awaitDone();
}

long startTime;
int nAcks = 0;
bool PatchTransmitter::awaitDone() {
    ROS_INFO("[patch_transmitter] Awaiting ACK from receiver");
    startTime = millitime();
    while(targetStatus < TARGET_OK && ros::ok() && millitime() < startTime + 1000) ros::spinOnce();
    if(! ros::ok()) exit(0);
    nAcks++;
    ROS_INFO("[patch_transmitter] Got %dth ACK from receiver", nAcks);

    int result = targetStatus == TARGET_OK;
    targetStatus = TARGET_PENDING;
    if(millitime() > startTime + 1000) return false;
    return result;
}

void PatchTransmitter::onStatusUpdate(const swarm_cmd::SwarmCommand::ConstPtr & msg) {
    if(msg->type == 2) { // ACK
        ROS_INFO("[patch_transmitter] Status update from target %s", msg->agent.c_str());
        string response = string((char *) msg->data.data() + 1, msg->data_length - 1);

        targetStatus = msg->data[0];
        ROS_INFO("[patch_transmitter] Got ACK (code %d); response as follows:\n", msg->data[0]);
        ROS_INFO("%s\n", response.c_str());

//        targetStatus = TARGET_OK;
    } else if(msg->type == 6) { // Retransmit request
        ROS_INFO("[patch_transmitter] RETRANSMIT request for %d", msg->order);
        commandOut.publish(chunks[msg->order]);

        ros::Duration(1).sleep();
        swarm_cmd::SwarmCommand done;
        done.type = 9;
        done.data_length = 1;
        done.agent = target;
        done.order = 0;
        done.data = {1};
        commandOut.publish(done);
    }
}

void PatchTransmitter::onPatchRequest(const swarmpatch::PatchRequest &msg) {
    if(msg.full) return;
    aim(msg.target, msg.project, msg.version);
    pack();
    transmit();
}
