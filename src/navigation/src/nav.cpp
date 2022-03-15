#include <ros/ros.h>
#include "navigation/wamv.h"
#include "controllers/pid.h"

#define SAMPLE 30                   // sample rate of PID loops

#define SENSITIVITY_DISTANCE 3      // 0.5 meters distance sensitivity
#define SENSITIVITY_ANGLE 360        // 2 degrees angle sensitivity

int main(int argc, char **argv)
{
    /**************************NODE INITIALIZATION**************************/

    // Initialize a ROS node called 'nav'
    ros::init(argc, argv, "nav");

    // Spinners allows ROS messages to be processed during blocking services
    ros::AsyncSpinner spinner(2);
    spinner.start();
    
    // Main access point to communications with the ROS system
    ros::NodeHandle n;
    ros::Rate loop_rate(SAMPLE);

    // Indicate the package initialized successfully
    ROS_INFO("Navigation Package: started successfully");


    /**************************WAMV INITIALIZATION**************************/

    // Initialize WAMV class
    WAMV boat(&n);
    
    // Initialize starting variables (goal, target vector, current location, distance)
    double *goal = boat.ReturnGoal();
    double *target_vector = boat.ReturnTargetVector();
    double *location = boat.ReturnLocation();
    double distance;

    // Initialize output vectors (x-direction, y-direction, angle)
    double O_x;
    double O_y;
    float O_a;

    //
    float calculated;
    float difference;
    float head;
    std::array<std::tuple<float, float>, 4> thrusters;
    // int count = 0;


    /**************************PID INITIALIZATION**************************/

    // MINOR CONTROL 

    // Initialize minor control and set its respective parameters
    PID horizontal(&n);
    PID vertical(&n);
    PID angle(&n);

    // Set PID references
    horizontal.SetRef(0);
    vertical.SetRef(0);
    angle.SetRef(0);

    // Set PID modes
    horizontal.SetMode(1);
    vertical.SetMode(1);
    angle.SetMode(1);

    // Set PID directions
    horizontal.SetPIDDirection(1);
    vertical.SetPIDDirection(1);
    angle.SetPIDDirection(1);

    // Set PID sample time
    horizontal.SetSampleTime(1.0/SAMPLE);
    vertical.SetSampleTime(1.0/SAMPLE);
    angle.SetSampleTime(1.0/SAMPLE);

    // Set PID output limits
    horizontal.SetOutputLimits(-40, 40);
    vertical.SetOutputLimits(-40, 40);
    angle.SetOutputLimits(-20, 20);

    // Set PID gains
    horizontal.SetGains(27, 2, 45);
    vertical.SetGains(27, 2, 45);
    angle.SetGains(3, 4, 10.5);

    // MAJOR CONTROL

    // Initialize major control and set its respective parameters
    PID major(&n);

    // Set PID reference, mode, direction, sample time, output limits and gains
    major.SetRef(0);
    major.SetMode(1);
    major.SetPIDDirection(0);
    major.SetSampleTime(1.0/SAMPLE);
    major.SetOutputLimits(-90, 90);
    major.SetGains(0.5,0,0.1);


    /**************************MAIN LOOP**************************/

    // Wait for first goal to be published
    while (boat.ReturnAngle() == 0) { loop_rate.sleep(); }

    // Update goal and current location
    boat.UpdateGoal();
    location = boat.ReturnLocation();

    // FILE *fp_ox;
    // FILE *fp_oy;
    // FILE *fp_ang;
    // std::string pkgpath = ros::package::getPath("navigation");
    // fp_ox = std::fopen((pkgpath+"/ox.txt").c_str(), "w");
    // fp_oy = std::fopen((pkgpath+"/oy.txt").c_str(), "w");
    // fp_ang = std::fopen((pkgpath+"/ang.txt").c_str(), "w");

    // Main navigation controller loop 
    while (ros::ok())
    {
        target_vector = boat.ReturnTargetVector();
        location = boat.ReturnLocation();
        goal = boat.ReturnGoal();

        distance = sqrt(std::pow(target_vector[0],2)+std::pow(target_vector[1],2));
        head = boat.ReturnAngle();
        
        //ROS_INFO("Tx: %f, Tx: %f", target_vector[0], target_vector[1]);
        //ROS_INFO("X: %f, Y: %f", offset_distances[0], offset_distances[1]);
        //ROS_INFO("lx: %s", std::to_string(location[0]).c_str());
        //ROS_INFO("ly: %s", std::to_string(location[1]).c_str());
        
        /// TODO: consider using raw target angle instead of difference
        if (distance > 15)
        {
            calculated = boat.CalcRef();
                
            O_a = major.Compute(calculated);
            thrusters = boat.MajorControl(O_a, 20);
            // boat.GoalReached(false);
        }
        else if ((distance > SENSITIVITY_DISTANCE) || (abs(boat.CalcAngle()) > SENSITIVITY_ANGLE))
        {
            calculated = boat.CalcAngle(); 

            O_x = horizontal.Compute(target_vector[0]);
            O_y = vertical.Compute(target_vector[1]);
            O_a = angle.Compute(calculated);

            // float time = ros::Time::now().toSec();
            // fprintf(fp_ox, "%f,%f,%f\n", time, target_vector[0], O_x);
            // fprintf(fp_oy, "%f,%f,%f\n", time, target_vector[1], O_y);
            // fprintf(fp_ang, "%f,%f,%f\n", time, calculated, O_a);

            // O_x = target_vector[0];
            // O_y = target_vector[1];
            // O_a = calculated;

            thrusters = boat.MinorControl(O_x, O_y, O_a, 3.5);
            // boat.GoalReached(false);
        }
        else
        {
            boat.GoalReached(true);
        }
        // if(count > 200){count = 0;
        // ROS_INFO("head: %s", std::to_string(head).c_str());
        // ROS_INFO("gx: %s", std::to_string(goal[2]).c_str());
        // ROS_INFO("angle diff: %s", std::to_string(boat.CalcAngle()).c_str());
        // ROS_INFO("tx: %f", target_vector[0]);
        // ROS_INFO("ty: %f", target_vector[1]);
        // ROS_INFO("distance: %s", std::to_string(distance).c_str());
        // }else{count++;}

        boat.UpdateThruster(thrusters);
  
        loop_rate.sleep();
    }

    // std::fclose(fp_ox);
    // std::fclose(fp_oy);
    // std::fclose(fp_ang);

    // Wait for this node to be shutdown, whether through Ctrl-C, ros::shutdown() or similar
    ros::waitForShutdown();
    return 0;
}
