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
    void UpdateLocal(float local[2], float theta);

    ///@brief Update the goal position (x, y)
    void UpdateGoal(float aim[2]);

    ///@brief Update the target vector and reference angle to goal
    void UpdateAngle();

private:
    ros::NodeHandle node; // ROS node handler
    float location[2];   //x and y pos of robot center
    float heading;    //heading of front of the robot 
    float goal[2];  //location of goal position
    float target_vector[2];     //vector from current location to the goal
    float angle;        //angle between heading and goal distance
};

#endif
