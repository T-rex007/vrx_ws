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

    WAMV boat(&n);
    PID horizontal(&n);
    PID vertical(&n);
    PID angle(&n);
    PID major(&n)

    double *goal = boat.ReturnGoal();
    double *target_vector = boat.ReturnTargetVector();
    double *location = boat.ReturnLocation();
    double distance;

    double O_x;
    double O_y;
    float O_a;

    float calculated;
    float difference;
    std::array<std::tuple<float, float>, 4> thrusters;

    horizontal.SetGains(1,0,0);
    vertical.SetGains(0.5,0,0);
    angle.SetGains(1,1,1);
    major.SetGains(1,1,1);

    horizontal.SetRef(0);
    vertical.SetRef(0);
    angle.SetRef(goal[2]);
    major.SetRef(0);
    

    while(!ready_flag)
    {
    }
    state.shutdown();

    while (ros::ok())
    {
        distance = sqrt(pow(target_vector[0],2)+pow(target_vector[1],2));
        calculated = boat.CalcAngle(boat.ReturnAngle());    //consider using raw target angle instead of difference
        O_a = major.Calculate(calculated);

        if(distance > 20)
        {
            //consider reseting minor control values in pid here
            thrusters = boat.MajorControl(O_a);
        }else
        {
            //consider reseting major control pid values here
            O_x = horizontal.Compute(target_vector[0]);
            O_y = vertical.Compute(target_vector[1]);
            // O_x = target_vector[0];
            // O_y = target_vector[1];
            thrusters = boat.MiniControl(O_x, O_y, O_a, 3.5); 
        }

        boat.UpdateThruster(thrusters);
        loop_rate.sleep();
        
    }

    // Wait for this node to be shutdown, whether through Ctrl-C, ros::shutdown() or similar
    ros::waitForShutdown();
    return 0;
}