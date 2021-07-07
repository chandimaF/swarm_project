#ifndef SWARM_PROJECT_SOFTWAREDISTRIBUTOR_H
#define SWARM_PROJECT_SOFTWAREDISTRIBUTOR_H

#include <string>
#include <vector>

using namespace std;

vector<string> getImages();
long getImageSize(const string & imageName);

class Image {

    string name;
    int layer_fd;

public:

    Image(string name) {

    }

    signed char * toBytes();
};

#endif
