

#include <ros/ros.h>
#include <std_msgs/String.h>

class DockerImagePusher {
public:
    ros::Subscriber update_sub;

    void onUpdateRequested(const std_msgs::String::ConstPtr & msg);
    void listen(int argc, char **argv);
};

int main(int argc, char **argv);
