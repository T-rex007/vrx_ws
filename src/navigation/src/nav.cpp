#include <ros/ros.h>
#include "navigation/wamv.h"
#include "controllers/pid.h"

#define SAMPLE 100

WAMV boat;
PID controller;

void GPSCallback(const std::msgs::Float::ConstPtr& msg)
{
    boat.UpdateLocal(msg.longitude, msg.latitude)
}

void GoalCallback(const std::msgs::Float::ConstPtr& msg)
{
    boat.UpdateGoal(msg.longitude, msg.latitude)
}

int main(int argc, char **argv)
{
    // Initialize a ROS node called 'navigation'
    ros::init(argc, argv, "navigation");

    // Main access point to communications with the ROS system
    ros::NodeHandle n;
    ros::Subscriber gps = n.subscribe("chatter", 100, GPSCallback);
    ros::Subscriber goal = n.subscribe("chatter", 100, GoalCallback);

    

    while ros.ok()
    {

    }

    // Wait for this node to be shutdown, whether through Ctrl-C, ros::shutdown() or similar
    ros::waitForShutdown();
    return 0;
}