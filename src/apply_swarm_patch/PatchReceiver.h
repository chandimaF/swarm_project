//
// Created by miles on 7/6/21.
//

#ifndef SWARM_PROJECT_PATCHRECEIVER_H
#define SWARM_PROJECT_PATCHRECEIVER_H

#include <string>

using namespace std;

class PatchReceiver {
public:
    void dumpBytes(unsigned char *bytes, string & project, int version);
    void unpack(string & project, int version);
    void apply(string & name, int version);
};


#endif //SWARM_PROJECT_PATCHRECEIVER_H
