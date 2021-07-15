//
// Created by mel on 7/12/21.
//

#include <ros/time.h>
#include <boost/filesystem.hpp>
#include <sha256.h>

#define DEFAULT_SWARM_DIR "/home/miles/swarmpatch"

using namespace std;

long millitime() {
    // i love oop but this snippet belongs in the tenth circle of hell (verbosity)
    struct timeval tp;
    gettimeofday(&tp, nullptr);
    return tp.tv_sec * 1000 + tp.tv_usec / 1000;
}

string getSwarmDir() {
    char * envSwarmDir = getenv("SWARM_DIR");
    if(envSwarmDir == nullptr) return DEFAULT_SWARM_DIR;
    return string(envSwarmDir);
}

void checkPaths(string project) {
    string swarmDir = getSwarmDir();
    boost::filesystem::create_directory(swarmDir);
    boost::filesystem::create_directory(swarmDir + "/incoming");
    boost::filesystem::create_directory(swarmDir + "/images");
    boost::filesystem::create_directory(swarmDir + "/outbound");
    boost::filesystem::create_directory(swarmDir + "/packs");
    boost::filesystem::create_directory(swarmDir + "/packs/" + project);
    boost::filesystem::create_directory(swarmDir + "/incoming/" + project);
    boost::filesystem::create_directory(swarmDir + "/images/" + project);
    boost::filesystem::create_directory(swarmDir + "/outbound/" + project);
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
