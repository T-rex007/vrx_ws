#ifndef _WAMV_H_
#define _WAMV_H_

#include <ros/ros.h>
#include <ros/package.h>
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

    /// @brief Returns the trajectory of the heading
    float ReturnAngle();

    /// @brief Returns the trajectory of the target
    float ReturnTargetAngle();

    /// @brief Calculates the Angle the boat will turn to meet goal angle
    float CalcAngle();

    /// @brief Calculates the Angle the boat will turn to meet target angle
    float CalcRef();

    /// @brief Turns the boat
    std::array<std::tuple<float, float>, 4> MajorControl(float ref, float range = 45);

    /// @brief Update the target vector and reference angle to goal
    void UpdateAngle();

    void GoalReached(const bool flag);

    void UpdateThruster(std::array<std::tuple<float, float>, 4> thrusters);

    double ConvertOrientation(geometry_msgs::Quaternion quat);

    std::array<std::tuple<float, float>, 4> MinorControl(float x, float y, float angle, float a, float ratio = 0.5);

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
    double goal[3];
    double linear_acc[2] = {0,0};
    double rec_linear_acc[2] = {0,0};
    double vel[2] = {0,0};
    double offset_distances[2] = {0,0};
    double prev_locations[2] = {0, 0};
    double distances[2];
    ros::Time last_time;
    double target_vector[2];  //vector from current location to the goal
    float target_angle;       //angle between north and goal distance
    ros::Subscriber gps;
    ros::Subscriber imu;
    ros::Subscriber goal_node;
    ros::Publisher thrusters_pub[8];
    ros::Publisher goal_reached_pub;
};

#endif
