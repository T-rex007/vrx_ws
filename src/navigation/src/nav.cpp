#include <ros/ros.h>
#include "navigation/wamv.h"
#include "controllers/pid.h"


#define SAMPLE 10

int main(int argc, char **argv)
{
    float calculated;
    float difference;
    std::array<std::tuple<float, float>, 4> thrusters;
    // Initialize a ROS node called 'navigation'
    ros::init(argc, argv, "navigation");
    ros::Rate loop_rate(SAMPLE);

    // Main access point to communications with the ROS system
    ros::NodeHandle n;

    WAMV boat(&n);
    PID controller(&n);


    controller.SetGains(1,1,1);
    while (ros::ok())
    {
        
        calculated = controller.Compute(boat.ReturnAngle());
        difference = boat.CalcAngle(calculated);
        thrusters = boat.TurnBoat(difference);
        loop_rate.sleep();
    }

    // Wait for this node to be shutdown, whether through Ctrl-C, ros::shutdown() or similar
    ros::waitForShutdown();
    return 0;
}