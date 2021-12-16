#ifndef _WAMV_H_
#define _WAMV_H_

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <math.h>

class WAMV
{
public:
    /// @brief Constructor for the WAMV
    explicit WAMV(ros::NodeHandle *node_handle);

    /// @brief Destructor for the WAMV
    ~WAMV();

    ///@brief Update wamv location (x, y) and heading (in degrees)
    void UpdateLocal(double longitude, double latitude, float theta);

    ///@brief Update the goal position (x, y)
    void UpdateGoal(double longitude, double latitude);

    ///@brief Returns the trajectory of the target
    float ReturnAngle();

    ///@brief Calculates the Angle the boat will turn
    float CalcAngle(float ref_angle);

    ///@brief Turns the boat
    float** TurnBoat(float ref_angle);

    ///@brief Update the target vector and reference angle to goal
    void UpdateAngle();

private:
    ros::NodeHandle node; // ROS node handler
    double location[2];   //x and y pos of robot center
    float heading;    //heading of front of the robot 
    float goal[2];  //location of goal position
    float target_vector[2];     //vector from current location to the goal
    float angle;        //angle between heading and goal distance
};

#endif
