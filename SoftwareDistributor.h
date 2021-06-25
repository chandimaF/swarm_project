#ifndef SWARM_PROJECT_SOFTWAREDISTRIBUTOR_H
#define SWARM_PROJECT_SOFTWAREDISTRIBUTOR_H

#include <string>
#include <vector>

using namespace std;

vector<string> getImages();
long getImageSize(const string & imageName);

class Patch {
public:
    char* patchFile;
    long size;
};


class Target {
public:
    int UID;
    int update(Patch p);
    

};


class SoftwareDistributor {
public:
    int updateAll(Patch p);
};


#endif
