#include "ros/ros.h"
#include "std_msgs/String.h"
#include "DockerImagePusher.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void DockerImagePusher::onUpdateRequested(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("[docker_image_pusher] We should push the image called '%s'", msg->data.c_str());
}

void DockerImagePusher::listen(int argc, char **argv) {
    ros::init(argc, argv, "docker_image_pusher");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("update_requests", 1000, &DockerImagePusher::onUpdateRequested, this);
    ros::spin();
}

int main(int argc, char ** argv) {
    DockerImagePusher d = DockerImagePusher();
    d.listen(argc, argv);
}