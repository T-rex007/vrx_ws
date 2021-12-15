#include <ros/ros.h>
#include "navigation/wamv.h"

int main(int argc, char **argv)
{
    // Initialize a ROS node called 'navigation'
    ros::init(argc, argv, "navigation");

    // Main access point to communications with the ROS system
    ros::NodeHandle n;

    // Wait for this node to be shutdown, whether through Ctrl-C, ros::shutdown() or similar
    ros::waitForShutdown();
    return 0;
}