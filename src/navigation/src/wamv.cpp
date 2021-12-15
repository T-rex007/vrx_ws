#include "navigation/wamv.h"

/// @brief Constructor for the WAMV
WAMV::WAMV(ros::NodeHandle *node_handle)
    : node(*node_handle)
{
    return;
}

/// @brief Destructor for the WAMV
WAMV::~WAMV()
{
    return;
}

///@brief Update wamv location (x, y) and heading (in degrees)
void WAMV::UpdateLocal(float local[2], float theta)
{
    std::copy(local, local + 2 , location);   //consider using array of len 3
    heading = theta;

    UpdateAngle();
}

///@brief Update the goal position (x, y)
void WAMV::UpdateGoal(float aim[2])
{
    std::copy(aim, aim + 2 , goal); //find how to automate finding the end of array

    UpdateAngle();
}

///@brief Update the target vector and reference angle to goal
void WAMV::UpdateAngle()
{
    float target[2];
    float ref_angle;
    float distance;

    target[0] = goal[0] - location[0];
    target[1] = goal[1] - location[1];
    distance = sqrt(pow(target[0], 2) + (target[1], 2));

    ref_angle = asin(target_vector[0]/distance);  //angle between euclidean distance vector and north reference in radians
    ref_angle = (ref_angle/M_PI) * 180;     //put in degrees
    
    if(target[1] > 0)
    {
        ref_angle = remainder((ref_angle + 360), 360);
    }else
        ref_angle = -1 * ref_angle + 180;

    std::copy(target, target + 2, target_vector);
    angle = ref_angle;
}