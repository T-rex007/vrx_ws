#ifndef _WAMV_H_
#define _WAMV_H_

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/NavSatFix.h>
#include <tuple>
#include <array>
#include <math.h>

#define QUEUE 50

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
    std::array<std::tuple<float, float>, 4> TurnBoat(float ref_angle);

    ///@brief Update the target vector and reference angle to goal
    void UpdateAngle();

    void UpdateThruster(std::array<std::tuple<float, float>, 4> thrusters);

    void GPSCallback(const sensor_msgs::NavSatFix msg);

    void GoalCallback(const sensor_msgs::NavSatFix msg);

private:
    ros::NodeHandle node; // ROS node handler
    double location[2];   //x and y pos of robot center
    float heading;    //heading of front of the robot 
    float goal[2];  //location of goal position
    float target_vector[2];     //vector from current location to the goal
    float angle;        //angle between heading and goal distance
    ros::Subscriber gps;
    ros::Subscriber goal_node;
    // ros::Publisher left_front_cmd;
    // ros::Publisher left_front_angle;
    // ros::Publisher right_front_cmd;
    // ros::Publisher right_front_angle;
    // ros::Publisher left_rear_cmd;
    // ros::Publisher left_rear_angle;
    // ros::Publisher right_rear_cmd;
    // ros::Publisher right_rear_angle;
    ros::Publisher thrusters_pub[8];

};

#endif
