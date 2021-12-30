#ifndef _WAMV_H_
#define _WAMV_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geographic_msgs/GeoPath.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
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

    ///@brief Returns the trajectory of the target
    float ReturnAngle();

    ///@brief Calculates the Angle the boat will turn
    float CalcAngle(float ref_angle);

    ///@brief Turns the boat
    std::array<std::tuple<float, float>, 4> MajorControl(float ref, float range = 45);

    ///@brief Update the target vector and reference angle to goal
    void UpdateAngle();

    void GetMatrix();

    void GoalReached(const bool flag);

    void UpdateThruster(std::array<std::tuple<float, float>, 4> thrusters);

    double ConvertOrientation(geometry_msgs::Quaternion quat);

    std::array<std::tuple<float, float>, 4> MiniControl(float x, float y, float O_a, float a, float ratio = 0.5);

    double* ReturnTargetVector();

    double* ReturnGoal();

    void UpdateGoal();

    double* ReturnLocation();

    void GPSCallback(const sensor_msgs::NavSatFix msg);

    void IMUCallback(const sensor_msgs::Imu msg);

    void GoalCallback(const geographic_msgs::GeoPoseStamped msg);


private:
    ros::NodeHandle node; // ROS node handler
    double location[2];   //x and y pos of robot center
    float heading;    //heading of front of the robot 
    //std::vector<std::array<double, 3>> goals;  //location of goal positions
    double goal[3];
    double target_vector[2];  //vector from current location to the goal
    float target_angle;       //angle between north and goal distance
    ros::Subscriber gps;
    ros::Subscriber imu;
    ros::Subscriber goal_node;
    ros::Publisher thrusters_pub[8];
    ros::Publisher goal_reached_pub;

};

#endif
