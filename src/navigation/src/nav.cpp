#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/NavSatFix.h>
#include "navigation/wamv.h"
#include "controllers/pid.h"


#define SAMPLE 100
#define QUEUE 50

WAMV boat;
PID controller;

void GPSCallback(const sensor_msgs::NavSatFix msg)
{
    boat.UpdateLocal(msg.longitude, msg.latitude);
}

void GoalCallback(const sensor_msgs::NavSatFix msg)
{
    boat.UpdateGoal(msg.longitude, msg.latitude);
}

int main(int argc, char **argv)
{
    float calculated;
    float difference;
    float thrusters[4][2];
    // Initialize a ROS node called 'navigation'
    ros::init(argc, argv, "navigation");
    ros::Rate loop_rate(SAMPLE);

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

    
    controller.SetGains(1,1,1);
    while (ros::ok())
    {
        
        calculated = controller.Compute(boat.ReturnAngle());
        difference = boat.CalcAngle(calculated);
        thrusters = boat.TurnBoat(difference);
        left_front_cmd.publish(thrusters[1][1]);
        left_front_angle.publish(thrusters[1][2]);
        right_front_cmd.publish(thrusters[2][1]);
        right_front_angle.publish(thrusters[2][2]);
        left_rear_cmd.publish(thrusters[3][1]);
        left_rear_angle.publish(thrusters[3][2]);
        right_rear_cmd.publish(thrusters[4][1]);
        right_rear_angle.publish(thrusters[4][2]);

        loop_rate.sleep();
    }

    // Wait for this node to be shutdown, whether through Ctrl-C, ros::shutdown() or similar
    ros::waitForShutdown();
    return 0;
}