//
// Created by miles on 7/8/21.
//


#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <iostream>
#include "PatchTransmitter.h"
#include <transmit_wifi/Transmission.h>
#include <json.cpp>

using namespace std;
using json = nlohmann::json;

int main(int argc, char ** argv) {

    ros::init(argc, argv, "patch_transmitter");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<transmit_wifi::Transmission>("/wifi_out", 0);

    auto * pt = new PatchTransmitter("alpine", 1, &pub);

//    pt->pack();
    pt->transmit();
//    ros::spin();
}

PatchTransmitter::PatchTransmitter(string p, int v, ros::Publisher * o): project(p), version(v), pub(o) {
    char * envSwarmDir = getenv("SWARM_DIR");
    if(envSwarmDir == nullptr) this->swarmDir = DEFAULT_SWARM_DIR;
    else this->swarmDir = string(envSwarmDir);

    checkPaths();
//    boost::filesystem::remove_all(swarmDir + "/packs/" + project + "/");

    cout << "Preparing to transmit project " + project + " v" + to_string(version) + " in " + swarmDir << "\n";
}

void PatchTransmitter::checkPaths() {
    boost::filesystem::create_directory(swarmDir);
    boost::filesystem::create_directory(swarmDir + "/packs");
    boost::filesystem::create_directory(swarmDir + "/packs/" + project);
    boost::filesystem::create_directory(swarmDir + "/incoming/");
    boost::filesystem::create_directory(swarmDir + "/incoming/" + project);
}

void PatchTransmitter::pack() {
    cout << "Packing project " + project + " v" + to_string(version) << "\n";

    checkPaths();
    string path = swarmDir+"/packs/"+project;
    chdir(path.c_str());

    system(("docker save " + project + ":latest > " + project + ".tar").c_str());
    boost::filesystem::create_directory(swarmDir + "/packs/" + project + "/" + to_string(version));
    system(("tar -xf "+project+".tar -C "+to_string(version)).c_str());
    system(("rm "+project+".tar").c_str());

    json j; // i hope that whoever nlohmann is, they're having a wonderful day
    ifstream file(swarmDir+"/packs/"+project+"/"+to_string(version)+"/manifest.json", fstream::in);
    file >> j;
    file.close();
    string layer = j[0]["Layers"][version - 1];

    string layerPath = swarmDir + "/packs/" + project + "/" + to_string(version) + "/" + layer;
    string archivePath = swarmDir + "/packs/" + project + "/" + to_string(version) + ".tar.gz";
    cout << "Zipping layer at " << layerPath << " to archive " << archivePath << "\n";

//    ifstream layerFile(layerPath, ifstream::in | ofstream::binary);
//    ofstream archiveFile(archivePath, ofstream::out | ifstream::binary);

    // I was doing this with Boost, but frankly, I hate working with tar files; they yield eof in weird places and
    //    the format seems inconsistent
    // So let's just use the gzip utility :/

    system(("gzip -c " + layerPath + " > " + archivePath).c_str());

//    boost::iostreams::filtering_streambuf<boost::iostreams::input> out;
//    out.push(boost::iostreams::gzip_compressor(boost::iostreams::gzip_params(boost::iostreams::gzip::best_compression)));
//    out.push(layerFile);
//
//    boost::iostreams::copy(out, archiveFile);
//    archiveFile.close();
//    layerFile.close();
}

long totalBytes = 0;
void PatchTransmitter::transmit() {
    checkPaths();
    cout << "Transmitting project " + project + " v" + to_string(version) << "\n";

    auto * buffer = (char *) calloc(2052, 1);
    string archivePath = swarmDir + "/packs/" + project + "/" + to_string(version) + ".tar.gz";
    ifstream file(archivePath, ifstream::in | ofstream::binary);

    ros::Rate rate(50);
    int i = 0;
    while(! file.eof()) {
        transmit_wifi::Transmission msg;
        file.read(buffer, 2048);
        buffer[2048] = (char) (i / (256*256*256));
        buffer[2049] = (char) (i / (256*256) % 256);
        buffer[2050] = (char) (i / (256) % (256*256));
        buffer[2051] = (char) (i % (256*256*256));

        totalBytes += 2048;
        cout << "Pushing chunks of layer from " << archivePath << " (" + to_string(totalBytes) + " bytes total)\n";
        vector<signed char> bytes = vector<signed char>(buffer, buffer + file.gcount() + 4);
        msg.data = bytes;
        msg.length = file.gcount();
        pub->publish(msg);
        i++;
        ros::spinOnce();
        rate.sleep();
    }
}


// Here's some old code that simply copies the file into the corresponding local swarmpatch/incoming -
//    useful for testing

//void PatchTransmitter::transmit() {
//    checkPaths();
//    cout << "Transmitting project " + project + " v" + to_string(version) << "\n";
//
//    // Gonna try the whole system for a hot second
//    string specifier = project + "/" + to_string(version);
//    system(("cp " + swarmDir + "/packs/" + specifier + ".tar.gz " + swarmDir + "/incoming/" + specifier + ".tar.gz").c_str());
//}
