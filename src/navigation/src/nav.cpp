#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "navigation/wamv.h"
#include "controllers/pid.h"


#define SAMPLE 50
#define QUEUE 50

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
    ros::Subscriber gps = n.subscribe("/wamv/sensors/gps/gps/fix", QUEUE, GPSCallback);
    ros::Subscriber goal = n.subscribe("/wamv/goal", QUEUE, GoalCallback);
    ros::Publisher left_front_cmd = n.advertise<std_msgs::Float32>("/wamv/thrusters/left_front_thrust_cmd", QUEUE);
    ros::Publisher left_front_angle = n.advertise<std_msgs::Float32>("/wamv/thrusters/left_front_thrust_angle", QUEUE);
    ros::Publisher right_front_cmd = n.advertise<std_msgs::Float32>("/wamv/thrusters/right_front_thrust_cmd", QUEUE);
    ros::Publisher right_front_angle = n.advertise<std_msgs::Float32>("/wamv/thrusters/right_front_thrust_angle", QUEUE);
    ros::Publisher left_rear_cmd = n.advertise<std_msgs::Float32>("/wamv/thrusters/left_rear_thrust_cmd", QUEUE);
    ros::Publisher left_rear_angle = n.advertise<std_msgs::Float32>("/wamv/thrusters/left_rear_thrust_angle", QUEUE);
    ros::Publisher right_rear_cmd = n.advertise<std_msgs::Float32>("/wamv/thrusters/right_rear_thrust_cmd", QUEUE);
    ros::Publisher right_rear_angle = n.advertise<std_msgs::Float32>("/wamv/thrusters/right_rear_thrust_angle", QUEUE);

    

    while ros.ok()
    {

    }

    // Wait for this node to be shutdown, whether through Ctrl-C, ros::shutdown() or similar
    ros::waitForShutdown();
    return 0;
}