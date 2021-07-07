//
// Created by miles on 7/6/21.
//

#include <fstream>
#include "sha256.h"
#include "PatchReceiver.h"
#include <boost/filesystem.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <nlohmann/json.hpp>
#include <boost/iostreams/detail/access_control.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <iostream>


#define SWARM_DIR "/home/miles/swarmpatch"

using json = nlohmann::json;


int main(int argc, char ** argv) {
    PatchReceiver p;
    auto * data = (unsigned char *) "abc";
    string project = "demo1";
//    p.dumpBytes(data, project, 1);
    p.unpack(project, 1);
    p.apply(project, 1);
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

void checkPaths(string & project) {
    boost::filesystem::create_directory(SWARM_DIR);
    boost::filesystem::create_directory(SWARM_DIR "/incoming");
    boost::filesystem::create_directory(SWARM_DIR "/images");
    boost::filesystem::create_directory(SWARM_DIR "/outbound");
    boost::filesystem::create_directory(SWARM_DIR "/incoming/" + project);
    boost::filesystem::create_directory(SWARM_DIR "/images/" + project);
    boost::filesystem::create_directory(SWARM_DIR "/outbound/" + project);
}

void PatchReceiver::dumpBytes(unsigned char *bytes, string & project, int version) {

    // This spits bytes (incoming via Chandima's data receiving system) into a corresponding .tar.gz file.
    //    It assumes that the data receiver knows the project name and version, and provides the bytes
    //    in order.

    checkPaths(project);
    string archivePath = SWARM_DIR "/incoming/" + project + "/" + to_string(version) + ".tar.gz";
    ofstream file;
    file.open(archivePath, fstream::out | fstream::app | fstream::binary);
    file << bytes;
    file.close();
}

void PatchReceiver::unpack(string & project, int version) {

    // This takes an incoming .tar.gz file (made via dumpBytes() above), unzips it, and puts it alongside other
    //    layers from the project.

    // Usually Docker saved images have a bunch of files named according to their SHA-256 hashes. I don't really
    //    care about checksumming right now (probably something that should happen but not here), so the ones that can
    //    be renamed more simply, are. (Here they're named by version number, which makes ordering much, much easier.)

    // Technically the tree of how Docker layers rely on each other is more complicated than this, but here I assume
    //    it's basically linear.

    checkPaths(project);
    string archivePath = SWARM_DIR "/incoming/" + project + "/" + to_string(version) + ".tar.gz";
    string layerPath = SWARM_DIR "/images/" + project + "/" + to_string(version) + ".tar";
    ifstream archiveFile;
    ofstream layerFile;
    archiveFile.open(archivePath, ifstream::in | ifstream::binary);
    layerFile.open(layerPath, ofstream::out | ofstream::binary);

    boost::iostreams::filtering_streambuf<boost::iostreams::input> in;
    in.push(boost::iostreams::gzip_decompressor());
    in.push(archiveFile);

    try {
        boost::iostreams::copy(in, layerFile);
        archiveFile.close();
        layerFile.close();
    } catch(boost::wrapexcept<boost::iostreams::gzip_error> & e) {
        cout << "Error: Could not unpack transmitted image layer (" << e.what() << ")\n";
    }
}

void PatchReceiver::apply(string & project, int version) {

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
    checkPaths(project);
    boost::filesystem::remove_all(SWARM_DIR "/outbound/" + project + "/"); // clean off from the last time we built this image
    boost::filesystem::create_directory(SWARM_DIR "/outbound/" + project);

    // 1: load information about all the layers
    string layersList = "[";
    string shaList = "[";

    // For every version... (they are 1-indexed)
    for(int i = 1; i == version; i++) {
        // We really just need to know layer names (which is programmatic) and SHA256 hashes, into a JSON list.
        // Get layer name:
        layersList += "\"" + to_string(i) + "/layer.tar" + ((i == version)? "\"]" : "\",");
        string layerPath = SWARM_DIR "/images/"+project+"/"+to_string(version)+".tar";

        // Get layer SHA256:
        shaList += "\"sha256:"+ getVersionSHA256(layerPath) + ((i == version)? "\"]" : ",");

        // Write the three layer-specific files.
        // Set up the layer directory in the saved image:
        string layerDir = SWARM_DIR "/outbound/" + project + "/" + to_string(i);
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
    ofstream manifestFile(SWARM_DIR "/outbound/"+project+"/manifest.json", fstream::out | fstream::app | fstream::binary);
    manifestFile << manifestJSON;
    manifestFile.close();

    // 3: dump into config.json
    string configJSON = R"({"architecture":"amd64","rootfs":{"type":"layers","diff_ids":)" + shaList + "}";
    configJSON += R"(,"config": {"Env":["PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"],"Cmd":["sh"]})";
    configJSON += R"(,"container_config": {"Env":["PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"],"Cmd":["sh"]}})";
    ofstream configFile(SWARM_DIR "/outbound/"+project+"/config.json", fstream::out | fstream::app | fstream::binary);
    configFile << configJSON;
    configFile.close();
}
