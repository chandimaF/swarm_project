//
// Created by miles on 7/12/21.
//

#ifndef SWARM_PROJECT_PATCHUTIL_H
#define SWARM_PROJECT_PATCHUTIL_H

long millitime();
void checkPaths(std::string project);
std::string getSwarmDir();
std::string getVersionSHA256(std::string & layerPath);

#endif //SWARM_PROJECT_PATCHUTIL_H
