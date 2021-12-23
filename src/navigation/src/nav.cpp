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
    PID horizontal(&n);
    PID vertical(&n);
    PID angle (&n);
    double *goal = boat.ReturnGoal();
    float *target_vector = boat.ReturnTargetVector();
    float distance = sqrt(pow(target_vector[0],2)+pow(target_vector[1],2));

    float O_x;
    float O_y;
    float O_a;

    float calculated;
    float difference;
    std::array<std::tuple<float, float>, 4> thrusters;

    horizontal.SetGains(1,1,1);
    vertical.SetGains(1,1,1);
    angle.SetGains(1,1,1);
    
    
    angle.SetRef(goal[3]);

    while (ros::ok())
    {
        target_vector = boat.ReturnTargetVector();
        // horizontal.SetRef(target_vector[0]);
        // vertical.SetRef(target_vector[1]);
        // O_x = horizontal.Compute(target_vector[0]);
        // O_y = vertical.Compute(target_vector[1]);
        O_a = angle.Compute(boat.ReturnAngle());
        O_x = target_vector[0];
        O_y = target_vector[1];
        thrusters = boat.Thrust_Converter(O_x, O_y, O_a, distance);
        boat.UpdateThruster(thrusters);
        loop_rate.sleep();
        
    }

    // Wait for this node to be shutdown, whether through Ctrl-C, ros::shutdown() or similar
    ros::waitForShutdown();
    return 0;
}