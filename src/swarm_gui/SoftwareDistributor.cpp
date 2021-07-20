#include "SoftwareDistributor.h"
#include <curl/curl.h>
#include <json.cpp>
#include <string>
#include <iostream>
#include "ros/ros.h"

using json = nlohmann::json;
using namespace std;

size_t onChunkReceived(void *contents, size_t size, size_t nmemb, std::string *s) {
    size_t newLength = size * nmemb;
    try {
        s->append((char *) contents, newLength);
    } catch(std::bad_alloc &e) {
        return 0;
    }
    return newLength;
}

vector<string> getImages() {
    CURL * curl = curl_easy_init();
    string catalog;
    if(curl) {
        curl_easy_setopt(curl, CURLOPT_URL, "localhost:5000/v2/_catalog");
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, onChunkReceived);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &catalog);

        CURLcode res = curl_easy_perform(curl);
        if(res != 0) throw runtime_error("Could not access Docker registry: error " + to_string(res));
        curl_easy_cleanup(curl);
    } else throw runtime_error("Could not initialize libcurl.");

    json j = json::parse(catalog);
    return j["repositories"].get<vector<string>>();
}

long getImageSize(const string & imageName) {
    CURL * curl = curl_easy_init();
    string manifest;
    if(curl) {
        struct curl_slist *chunk = nullptr;
        chunk = curl_slist_append(chunk, "Accept: application/vnd.docker.distribution.manifest.v2+json");
        curl_easy_setopt(curl, CURLOPT_URL, ("localhost:5000/v2/" + imageName + "/manifests/latest").c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, onChunkReceived);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &manifest);
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, chunk);
        CURLcode res = curl_easy_perform(curl);
        if(res != 0) throw runtime_error("Could not access Docker registry: error " + to_string(res));
        curl_easy_cleanup(curl);
        delete chunk;
    } else throw runtime_error("Could not initialize libcurl.");

    json j = json::parse(manifest);
    return j["layers"][j["layers"].size()-1]["size"].get<long>();
}

