//
// Created by mel on 7/6/21.
//

#include <fstream>
#include "PatchReceiver.h"
#include <transmit_wifi/Transmission.h>
#include <boost/filesystem.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <iostream>
#include <ros/ros.h>
#include <sha256.h>

int main(int argc, char ** argv) {
    ros::init(argc, argv, "patch_receiver");
    ros::NodeHandle nh;
    ros::Subscriber s;

    auto * p = new PatchReceiver("alpine", 1, s);
    p->sub = nh.subscribe("/wifi_in", 1000, &PatchReceiver::onIncomingChunk, p); // smells bidependent

    ros::spin();
}

string getVersionSHA256(string & layerPath) {
    SHA256 sha;
    char buffer[256];

    fstream layerFile(layerPath, fstream::in | fstream::binary);
    while(! layerFile.eof()) {
        layerFile.read(buffer, 256);
        sha.add(buffer, layerFile.gcount());
    }
    layerFile.close();
    return sha.getHash();
}

PatchReceiver::PatchReceiver(const string & p, int v, ros::Subscriber & s): project(p), version(v), sub(s) {
    char * envSwarmDir = getenv("SWARM_DIR");
    if(envSwarmDir == nullptr) this->swarmDir = DEFAULT_SWARM_DIR;
    else this->swarmDir = string(envSwarmDir);
    cout << "Preparing to receive project " + project + " v" + to_string(version) + " in " + swarmDir << "\n";
}

void PatchReceiver::checkPaths() {
    boost::filesystem::create_directory(swarmDir);
    boost::filesystem::create_directory(swarmDir + "/incoming");
    boost::filesystem::create_directory(swarmDir + "/images");
    boost::filesystem::create_directory(swarmDir + "/outbound");
    boost::filesystem::create_directory(swarmDir + "/incoming/" + project);
    boost::filesystem::create_directory(swarmDir + "/images/" + project);
    boost::filesystem::create_directory(swarmDir + "/outbound/" + project);
}

void PatchReceiver::onIncomingChunk(const transmit_wifi::Transmission::ConstPtr & msg) {
    size_t nbytes = msg->data.size();
    const signed char * bytes = msg.get()->data.data();
    dumpBytes((unsigned char *) bytes);
}

void PatchReceiver::dumpBytes(unsigned char *bytes) {

    // This spits bytes (incoming via Chandima's data receiving system) into a corresponding .tar.gz file.
    //    It assumes that the data receiver knows the project name and version, and provides the bytes
    //    in order.

    checkPaths();
    string archivePath = swarmDir + "/incoming/" + project + "/" + to_string(version) + ".tar.gz";
    ofstream file;
    file.open(archivePath, fstream::out | fstream::app | fstream::binary);
    file << bytes;
    file.close();
}

void PatchReceiver::unpack() {
    cout << "Unpacking project " + project + " v" + to_string(version) << "\n";

    // This takes an incoming .tar.gz file (made via dumpBytes() above), unzips it, and puts it alongside other
    //    layers from the project.

    // Usually Docker saved images have a bunch of files named according to their SHA-256 hashes. I don't really
    //    care about checksumming right now (probably something that should happen but not here), so the ones that can
    //    be renamed more simply, are. (Here they're named by version number, which makes ordering much, much easier.)

    // Technically the tree of how Docker layers rely on each other is more complicated than this, but here I assume
    //    it's basically linear.

    checkPaths();
    string archivePath = swarmDir + "/incoming/" + project + "/" + to_string(version) + ".tar.gz";
    string layerPath = swarmDir + "/images/" + project + "/" + to_string(version) + ".tar";

    ifstream archiveFile(archivePath, ifstream::in | ifstream::binary);
    ofstream layerFile(layerPath, ofstream::out | ofstream::binary);

    boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
    in.push(boost::iostreams::gzip_decompressor());
    in.push(archiveFile);

    boost::iostreams::copy(in, layerFile);
    archiveFile.close();
    layerFile.close();
}

void PatchReceiver::build() {
    cout << "Building project " + project + " v" + to_string(version) << "\n";

    // This remakes a saved Docker image from transmitted layers.

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

    // 0: clean up the directory we'll work in
    checkPaths();
    boost::filesystem::remove_all(swarmDir + "/outbound/" + project + "/"); // clean off from the last time we built this image
    boost::filesystem::create_directory(swarmDir + "/outbound/" + project);

    // 1: load information about all the layers
    string layersList = "[";
    string shaList = "[";

    // For every version... (they are 1-indexed)
    for(int i = 1; i == version; i++) {
        // We really just need to know layer names (which is programmatic) and SHA256 hashes, into a JSON list.
        // Get layer name:
        layersList += "\"" + to_string(i) + "/layer.tar" + ((i == version)? "\"]" : "\",");
        string layerPath = swarmDir + "/images/"+project+"/"+to_string(version)+".tar";

        // Get layer SHA256:
        shaList += "\"sha256:"+ getVersionSHA256(layerPath) + ((i == version)? "\"]" : ",");

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

void PatchReceiver::apply() {
    cout << "Applying project " + project + " v" + to_string(version) << "\n";

    // we do assume that we have Docker, plus the tar utility.
    //    Docker obviously we can't manage without (and clearly should be on any system receiving a patch).
    //    The presence of tar kind of assumes a UNIX (?) machine I guess, but I don't know if there's anything
    //    I can do about that. Tar is a pain to work with programmatically
    //    and I don't want to waste time prematurely optimizing.

    string path = swarmDir + "/outbound/"+project;
    chdir(path.c_str());

    string commands[] = {
            "tar -cf "+project+".tar *",
            "docker load < "+project+".tar"
    };
    for(string & s: commands) system(s.c_str());
}

//int main(int argc, char ** argv) {
//    PatchReceiver p = PatchReceiver( "demo1", 1);
//    p.unpack();
//    p.build();
//    p.apply();
//}
