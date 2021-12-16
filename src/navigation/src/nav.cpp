#include <ros/ros.h>
#include "navigation/wamv.h"
#include "controllers/pid.h"
#include "vrx_gazebo/Task.h"


#define SAMPLE 100

bool ready_flag = false;

void TaskCallback(vrx_gazebo::Task msg)
{
    ROS_INFO("%s",msg.state.c_str());
    if("ready" == msg.state)
    {
        ready_flag = true;

    }
}

int main(int argc, char **argv)
{
    
    // Initialize a ROS node called 'navigation'
    ros::init(argc, argv, "navigation");

    ros::AsyncSpinner spinner(2);
    spinner.start();
    
    // Main access point to communications with the ROS system
    ros::NodeHandle n;
    ros::Rate loop_rate(SAMPLE);

    ros::Subscriber state = n.subscribe("/vrx/task/info", QUEUE, TaskCallback);

    while(!ready_flag)
    {
    }
    state.shutdown();

    WAMV boat(&n);
    PID controller(&n);

    float calculated;
    float difference;
    std::array<std::tuple<float, float>, 4> thrusters;

    controller.SetGains(1,1,1);
    

    while (ros::ok())
    {
        calculated = controller.Compute(boat.ReturnAngle());
        difference = boat.CalcAngle(calculated);
        thrusters = boat.TurnBoat(difference);
        boat.UpdateThruster(thrusters);
        loop_rate.sleep();
        
    }

    // Wait for this node to be shutdown, whether through Ctrl-C, ros::shutdown() or similar
    ros::waitForShutdown();
    return 0;
}