#include "ros/ros.h"
#include "libviso_mono_odometr.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_listener");
    ros::NodeHandle nodeHandle;
    ROS_INFO_STREAM("It's alive!");
    ros::NodeHandle local_nh("~");
//        read params
    std::string imageTopic;
    std::string infoTopic;
    std::string odomOutTopic;

    local_nh.param("image", imageTopic, std::string("/image"));
    local_nh.param("info", infoTopic, std::string("/info"));
    local_nh.param("odometry", odomOutTopic, std::string("/odometry"));
//    ROS_INFO_STREAM("Args:" << imageTopic << "," << infoTopic << "," << odomOutTopic);

    LibvisoMonoOdometer odometer(nodeHandle, imageTopic, infoTopic, odomOutTopic);
    ros::spin();
    return 0;
}
