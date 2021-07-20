//
// Created by mel on 7/6/21.
//

#include <fstream>
#include "PatchReceiver.h"
#include "PatchUtil.h"
#include <swarm_cmd/SwarmCommand.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <utility>
#include <ros/ros.h>
#include <sha256.h>


// Timeout will be one second for now. In the future this should be wrapped by another layer that says when exactly to be done,
// but that's not a priority here.
#define TIMEOUT 1000

long lastMessageReceived = 0;

int main(int argc, char ** argv) {
    ros::init(argc, argv, "patch_receiver");

    auto * p = new PatchReceiver("definitely_not_alpine", 1);

    // await messages until the first one is received, then wait until a timeout for more
    while ((lastMessageReceived == 0 || millitime() < lastMessageReceived + TIMEOUT) && ros::ok()) {
        ros::spinOnce();
    }
    if(lastMessageReceived == 0) ROS_INFO("[patch_receiver] Final message received at %ld", lastMessageReceived);

    p->unpack();
    p->build();
    p->apply();
}

PatchReceiver::PatchReceiver(string p, int v): project(std::move(p)), version(v) {
    checkPaths(project);
    string swarmDir = getSwarmDir();
    ROS_INFO("[patch_receiver] Preparing to receive project %s v%d in %s", project.c_str(), version, swarmDir.c_str());
    ros::NodeHandle nh;
//

    this->cmdOut = nh.advertise<swarm_cmd::SwarmCommand>("command_out", 100);
    while(cmdOut.getNumSubscribers() == 0) ros::spinOnce();
    cmdIn = nh.subscribe("command_in", 1000, &PatchReceiver::onIncomingCommand, this);
}

void PatchReceiver::onIncomingCommand(const swarm_cmd::SwarmCommand::ConstPtr & cmd) {
    if(cmd->type == 3) {
        string swarmDir = getSwarmDir();

        // a hack for 6/16 test
        version = (int) cmd->order;
        if(version == 1) {
            boost::filesystem::remove_all(swarmDir + "/outbound/" + project + "/");
            boost::filesystem::remove_all(swarmDir + "/incoming/" + project + "/");
            boost::filesystem::remove_all(swarmDir + "/images/" + project + "/");
        }
        checkPaths(project);

        string archivePath = swarmDir + "/incoming/" + project + "/" + to_string(version) + ".tar.gz";
        ROS_INFO("[patch_receiver] Incoming patch command of size %d", cmd->data_length);

        ofstream file;
        file.open(archivePath, fstream::out | fstream::app | fstream::binary);
        file.write((const char *) cmd->data.data(), cmd->data_length);
        file.close();

        lastMessageReceived = millitime();
    }
}
void PatchReceiver::unpack() const {
    // This takes an incoming .tar.gz file (made via dumpBytes() above), unzips it, and puts it alongside other
    //    layers from the project.

    checkPaths(project);
    string swarmDir = getSwarmDir();
    ROS_INFO("[patch_receiver] Unpacking project %s v%d", project.c_str(), version);

    string archivePath = swarmDir + "/incoming/" + project + "/" + to_string(version) + ".tar.gz";
    string layerPath = swarmDir + "/images/" + project + "/" + to_string(version) + ".tar";
    system(("gunzip -c " + archivePath + " > " + layerPath).c_str());

    // If you're interested in the `docker save` format:
    // Usually Docker saved images have a bunch of files named according to their SHA-256 hashes. I don't really
    //    care about checksumming right now (probably something that should happen but not here), so the ones that can
    //    be renamed more simply, are. (Here they're named by version number, which makes ordering much, much easier.)

    // Technically the tree of how Docker layers rely on each other is more complicated than this, but here I assume
    //    it's basically linear.

}

void PatchReceiver::build() const {
    // This remakes a saved Docker image from transmitted layers.

    checkPaths(project);
    string swarmDir = getSwarmDir();
    boost::filesystem::remove_all(swarmDir + "/outbound/" + project + "/"); // clean off from the last time we built this image
    boost::filesystem::create_directory(swarmDir + "/outbound/" + project);
    ROS_INFO("[patch_receiver] Building project %s v%d", project.c_str(), version);

    // The saved image structure is as follows:
    // * [image name].
    // * * manifest.json
    // * * repositories   (allowed to be missing)
    // * * config.json
    // * * [a layer]
    // * * * VERSION
    // * * * json
    // * * * layer.tar

    // The only file we need to transmit is layer.tar! (One per "project version," which is a concept made up here to
    //    make the frontend more familiar. It's a Docker commit, basically.) The others, especially config.json, take up
    //    a few kilobytes, which in the grand scheme of things isn't much. But it turns out that we have to remake them
    //    anyway (to get linked list structure right), so we may as well just transmit layer.tar.


    // 1: load information about all the layers
    string layersList = "[";
    string shaList = "[";

    // For every version... (they are 1-indexed)
    for(int i = 1; i <= version; i++) {
        // We really just need to know layer names (which are programmatic) and SHA256 hashes, into a JSON list.
        // Get layer name:
        layersList += "\"" + to_string(i) + "/layer.tar" + ((i == version)? "\"]" : "\",");
        string layerPath = swarmDir + "/images/"+project+"/"+to_string(i)+".tar";

        // Get layer SHA256:
        shaList += "\"sha256:"+ getVersionSHA256(layerPath) + ((i == version)? "\"]" : "\",");

        // Write the three layer-specific files.
        // Set up the layer directory in the saved image:
        string layerDir = swarmDir + "/outbound/" + project + "/" + to_string(i);
        boost::filesystem::create_directory(layerDir);

        // Write the layer json file (named simply "json") - determines layer linked list structure, among other things
        string layerJSON = R"({"id":")"+to_string(i)+R"(","docker_version":"20.10.1","architecture":"amd64","os":"linux")";
        if(i > 1) layerJSON += R"(,"parent":\")" + to_string((i-1)) + "\"";
        layerJSON += R"(,"config": {"Env":["PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"],"Cmd":["sh"]})";
        layerJSON += R"(,"container_config": {"Env":["PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"],"Cmd":["sh"]}})";
        ofstream jsonFile(layerDir + "/json", fstream::out | fstream::app | fstream::binary);
        jsonFile << layerJSON;
        jsonFile.close();

        // Write the VERSION file
        ofstream versionFile(layerDir + "/VERSION", fstream::out | fstream::app | fstream::binary);
        versionFile << "1.0"; // yes, as far as I can tell it really is always 1.0.
        versionFile.close();

        // Copy actual layer tar from image to staging area
        std::ifstream src(layerPath, std::ios::binary);
        std::ofstream dest(layerDir+"/layer.tar", std::ios::binary);
        dest << src.rdbuf();

        src.close();
        dest.close();
    }

    // 2: dump into manifest.json
    string manifestJSON = R"([{"Config":"config.json","RepoTags":[")"+project+R"(:latest"],"Layers":)" + layersList + "}]";
    ofstream manifestFile(swarmDir + "/outbound/"+project+"/manifest.json", fstream::out | fstream::app | fstream::binary);
    manifestFile << manifestJSON;
    manifestFile.close();

    // 3: dump into config.json
    string configJSON = R"({"architecture":"amd64","rootfs":{"type":"layers","diff_ids":)" + shaList + "}";
    configJSON += R"(,"config": {"Env":["PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"],"Cmd":["sh"]})";
    configJSON += R"(,"container_config": {"Env":["PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"],"Cmd":["sh"]}})";
    ofstream configFile(swarmDir + "/outbound/"+project+"/config.json", fstream::out | fstream::app | fstream::binary);
    configFile << configJSON;
    configFile.close();
}

void PatchReceiver::apply() const {
    checkPaths(project);
    string swarmDir = getSwarmDir();
    ROS_INFO("[patch_receiver] Applying project %s v%d", project.c_str(), version);

    // we do assume that we have Docker, plus the tar utility. (Also gzip, elsewhere.)
    //    Docker obviously we can't manage without (and clearly should be on any system receiving a patch).
    //    The presence of tar kind of assumes a UNIX (?) machine I guess, but I don't know if there's anything
    //    I can do about that. Tar is a pain to work with programmatically
    //    and I don't want to waste time prematurely optimizing.

    string path = swarmDir + "/outbound/"+project;
    chdir(path.c_str());

    system(("tar -cf "+project+".tar *").c_str());
    system(("docker load < "+project+".tar").c_str());

    swarm_cmd::SwarmCommand cmd;
    cmd.data = {0};
    cmd.type = 2;
    cmd.data_length = 1;
    cmd.order = 0;
    cmd.agent = "~"; // special case for a response to ground
    cmdOut.publish(cmd);
}
