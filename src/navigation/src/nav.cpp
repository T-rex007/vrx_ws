#include <ros/ros.h>
#include "navigation/wamv.h"
#include "controllers/pid.h"


#define SAMPLE 15
#define SENSITIVITY 1
#define SENSITIVITY2 3


int main(int argc, char **argv)
{
    
    // Initialize a ROS node called 'navigation'
    ros::init(argc, argv, "nav");

    // Spinners allows ROS messages to be processed during blocking services
    ros::AsyncSpinner spinner(2);
    spinner.start();
    
    // Main access point to communications with the ROS system
    ros::NodeHandle n;
    ros::Rate loop_rate(SAMPLE);

    ROS_INFO("Navigation Package: started successfully");

    WAMV boat(&n);
    PID horizontal(&n);
    PID vertical(&n);
    PID angle(&n);
    PID major(&n);

    double *goal = boat.ReturnGoal();
    double *target_vector = boat.ReturnTargetVector();
    double *location = boat.ReturnLocation();
    double distance;

    double O_x;
    double O_y;
    float O_a;

    float calculated;
    float difference;
    float head;
    std::array<std::tuple<float, float>, 4> thrusters;

    horizontal.SetGains(2,0,2);
    vertical.SetGains(2,0,2);
    angle.SetGains(1,0,1);
    major.SetGains(1,1,1);

    horizontal.SetRef(0);
    vertical.SetRef(0);
    angle.SetRef(0);
    major.SetRef(0);

    while(boat.ReturnAngle() == 0){ loop_rate.sleep(); }

    boat.UpdateGoal();
    double *offset_distances = boat.ReturnDistances();
    location = boat.ReturnLocation();

    while (ros::ok())
    {
        boat.CalcVelocities();
        offset_distances = boat.ReturnDistances();
        target_vector = boat.ReturnTargetVector();
        location = boat.ReturnLocation();
        goal = boat.ReturnGoal();
        //ROS_ERROR("Goal[0]: %f, Goal[1]: %f, Goal[2]: %f", goal[0], goal[1], goal[2]);
        distance = sqrt(std::pow(target_vector[0],2)+std::pow(target_vector[1],2));
        head = boat.ReturnAngle();
        
        //ROS_INFO("Tx: %f, Tx: %f", target_vector[0], target_vector[1]);
        //ROS_INFO("X: %f, Y: %f", offset_distances[0], offset_distances[1]);
        //ROS_INFO("lx: %s", std::to_string(location[0]).c_str());
        //ROS_INFO("ly: %s", std::to_string(location[1]).c_str());
        //ROS_INFO("gx: %s", std::to_string(goal[2]).c_str());
        
        if(distance > 10)
        {
            //consider reseting minor control values in pid here
            calculated = boat.CalcRef();    //consider using raw target angle instead of difference
            O_a = major.Compute(calculated);
            thrusters = boat.MajorControl(calculated, 45);
        }else if(distance > SENSITIVITY)
        {
            //consider reseting major control pid values here
            calculated = boat.CalcAngle();    //consider using raw target angle instead of difference
            // O_x = horizontal.Compute(target_vector[0]);
            // O_y = vertical.Compute(target_vector[1]);
            // O_a = angle.Compute(calculated);
            O_a = calculated;
            O_x = target_vector[0];
            O_y = target_vector[1];
            thrusters = boat.MiniControl(O_x, O_y, O_a, 3.5); 
        }else if(abs(calculated) < SENSITIVITY2){
            boat.GoalReached(true);
        }

        // ROS_INFO("head: %s", std::to_string(head).c_str());
        // ROS_INFO("target angle: %s", std::to_string(boat.ReturnTargetAngle()).c_str());
        // ROS_INFO("difference: %s", std::to_string(boat.CalcRef()).c_str());
        // ROS_INFO("tx: %s", std::to_string(target_vector[0]).c_str());
        // ROS_INFO("ty: %s", std::to_string(target_vector[1]).c_str());
        // ROS_INFO("distance: %s", std::to_string(distance).c_str());


        boat.UpdateThruster(thrusters);

        
        loop_rate.sleep();
        
    }

    // Wait for this node to be shutdown, whether through Ctrl-C, ros::shutdown() or similar
    ros::waitForShutdown();
    return 0;
}
