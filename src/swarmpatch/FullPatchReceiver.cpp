//
// Created by miles on 7/12/21.
//

#include "FullPatchReceiver.h"
#include <pstream.cpp>
#include <string>
#include <boost/filesystem.hpp>
#include "PatchUtil.h"
#include <json.cpp>
#include <fstream>

using namespace std;

void updateLayerCache(string project) {
    string swarmDir = getSwarmDir();

    // if the hostname was given as part of the project, get rid of that
    if(project.find('/') != string::npos) {
        project = project.substr(project.find('/')+1, project.length());
    }

    checkPaths(project);
    string path = swarmDir+"/packs/"+project;
    chdir(swarmDir.c_str());

    boost::filesystem::create_directory(swarmDir + "/packs/" + project);
    system(("docker save " + project + ":latest > " + path + "/" + project + ".tar").c_str());
    boost::filesystem::create_directory(path + "/all");
    system(("tar -xf "+ path + "/" +project+".tar -C " + path + "/" +"all").c_str());

    nlohmann::json j; // i hope that whoever nlohmann is, they're having a wonderful day
    ifstream file(swarmDir+"/packs/"+project+"/all/manifest.json");
    file >> j;
    vector<string> layers = j[0]["Layers"];
    file.close();

    int v = 1;
    for(string & layer: layers) {
        string layerPath = swarmDir + "/packs/" + project + "/all/" + layer;
        string archivePath = swarmDir + "/images/" + project + "/" + to_string(v) + ".tar";
        cout << "Copying layer at " << layerPath << " to archive " << archivePath << "\n";
        system(("cp " + layerPath + " " + archivePath).c_str());
        v++;
    }
}

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
//    updateLayerCache(image);

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


