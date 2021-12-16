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

///@brief Returns the trajectory of the target
float WAMV::ReturnAngle()
{
   return angle;
}

///@brief Calculates the Angle the boat will turn
float WAMV::CalcAngle(float ref_angle)
{
    float difference;
    difference = ref_angle - heading;
    if (abs(difference) > 180)  //consider accounting for momentum in turning
    { 
        difference = remainder((360 - difference), 360);

    } 
   return difference;
}

///@brief Turns the boat
float WAMV::TurnBoat(float ref_angle)
{
    float thrusters[4][2];
    thrusters[1][1] = 1;
    thrusters[1][2] = remainder(ref_angle,90);
    thrusters[2][1] = 1;
    thrusters[2][2] = remainder(ref_angle,90);
    thrusters[3][1] = 1;
    thrusters[3][2] = 0;
    thrusters[4][1] = 1;
    thrusters[4][2] = 0;
    return thrusters;
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