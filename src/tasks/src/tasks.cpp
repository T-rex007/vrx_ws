#include <ros/ros.h>
#include "tasks/processing.h"

int main(int argc, char **argv)
{
    // Initialize a ROS node called 'tasks'
    ros::init(argc, argv, "tasks");

    // Main access point to communications with the ROS system
    ros::NodeHandle n;

    // Wait for this node to be shutdown, whether through Ctrl-C, ros::shutdown() or similar
    ros::waitForShutdown();
    return 0;
}