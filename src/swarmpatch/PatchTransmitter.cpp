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

int main(int argc, char ** argv) {
    ros::init(argc, argv, "patch_transmitter");
    auto * pt = new PatchTransmitter("odroid_agent", "sizeteste_256", 2);
    ros::spin();
}

PatchTransmitter::PatchTransmitter(string t, string p, int v): target(t), project(p), version(v) {
    ros::NodeHandle nh;
    this->commandOut = nh.advertise<swarm_cmd::SwarmCommand>("command_out", 1000, true);
    this->commandIn = nh.subscribe("command_in", 100, &PatchTransmitter::onStatusUpdate, this);
    this->requestIn = nh.subscribe("patch_requests", 100, &PatchTransmitter::onPatchRequest, this);
    ROS_INFO("[patch_transmitter] Initialized transmitter for %s v%d -> %s ", p.c_str(), v, t.c_str());
    aim(t, p, v);
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
    commandOut.publish(cmd);
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
    cout << "\n\n------\n" << j << "------\n\n";
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

    swarm_cmd::SwarmCommand cmd;
    cmd.type = 3;
    cmd.data_length = nBytes;
    cmd.agent = target;
    cmd.order = version;
    cmd.data = vector<unsigned char>(buffer, buffer+nBytes);
    commandOut.publish(cmd);
    targetStatus = TARGET_PULLING;
}

long startTime;
bool PatchTransmitter::awaitDone() const {
    startTime = millitime();
    while(targetStatus == TARGET_PULLING || millitime() > startTime + TIMEOUT) ros::spinOnce();
    ROS_ERROR("[patch_transmitter] Patch of %s complete; took %lu milliseconds", this->project.c_str(), millitime() - startTime);
    return targetStatus == TARGET_OK;
}

void PatchTransmitter::onStatusUpdate(const swarm_cmd::SwarmCommand::ConstPtr & msg) {
    ROS_INFO("[full_patch_transmitter] Status update from target %s", msg->agent.c_str());
    if(msg->type != 2) return;
    string response = string((char *) msg->data.data()+1, msg->data_length);

    targetStatus = msg->data[0];

    if(targetStatus != 0) {
        ROS_INFO("[patch_transmitter] Response as follows:\n");
        ROS_ERROR("%s", response.c_str());
        ROS_ERROR("\n[patch_transmitter] (Target failed to install latest version)");
    } else {
        ROS_INFO("[full_patch_transmitter] Response as follows:\n");
        ROS_INFO("%s", response.c_str());
        ROS_INFO("\n[patch_transmitter] (Target successfully installed latest version)");
    }
}

void PatchTransmitter::onPatchRequest(const swarmpatch::PatchRequest &msg) {
    if(msg.full) return;
    aim(msg.target, msg.project, msg.version);
    pack();
    transmit();
    awaitDone();
}
