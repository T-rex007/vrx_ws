#ifndef _PROCESSING_H_
#define _PROCESSING_H_

#include <ros/ros.h>
#include "vrx_gazebo/Task.h"
#include <std_msgs/Bool.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <geographic_msgs/GeoPath.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/NavSatFix.h>

#include <limits>
#include <math.h>
#include <vector>

#define QUEUE 100                       // Number of messages to queue in ROS subscribers before discarding

class Processing
{
public:
    std::string taskName = "NULL";      // Stores current task name
    std::string taskState = "NULL";     // Stores current task state

    /// @brief Constructor for the Processing
    explicit Processing(ros::NodeHandle *node_handle);

    /// @brief Destructor for the Processing
    ~Processing();

    /// @brief Initialize the correct subscribers and publishers based on the task
    void InitSubsAndPubs();

    /// @brief Publish the correct messages based on the task
    void PublishMessages();

    /// @brief Finds optimum path of the WAMV given a cost matrix using recursion
    std::vector<float> GoalSort(std::vector<std::vector<float>> matrix, int current, std::vector<int> nodes);

    /// @brief Function to get the distance matrix and optimize the goal pathing
    void GetMatrix(geographic_msgs::GeoPath road);

    /// @brief circle object
    std::vector<std::array<double, 3>> Circle(double center[2], int num_pts, float r, double ref0 = 0, double ref1 = 0);

    /// @brief circle object
    std::vector<std::array<double, 3>> avoidC(double center[2], double gal[2], float r);

    void avoid(double center[2], double temp[2], float r);

    ///@brief converts x and y with wamv ref to vector of geostamped
    std::vector<geographic_msgs::GeoPoseStamped> CarttoGeo(std::vector<std::array<double, 3>> points, double x[2]);

    ///@brief gets minigoal points based on obstacle encountered
    void Special(std::string obstacle, double center[2]);

    /// @brief Callback function for tasks subscriber
    void TasksCallback(const vrx_gazebo::Task msg);

    /// @brief Callback function for goal subscriber in task 1
    void GoalT1Callback(const geographic_msgs::GeoPoseStamped msg);

    /// @brief Callback function for goal subscriber in task 2
    void GoalT2Callback(const geographic_msgs::GeoPath msg);

       /// @brief Callback function for goal subscriber in task 4
    void GoalT4Callback(const geographic_msgs::GeoPath msg);

    /// @brief Callback function to indicate the wamv reached the goal
    void GoalReachedCallback(const std_msgs::Bool msg);

    /// @brief Callback function to get GPS location
    void GPSCallback(const sensor_msgs::NavSatFix msg);

private:

    ros::NodeHandle node; // ROS node handler
    ros::Subscriber gps;
    ros::Subscriber tasksSub; // ROS subscriber for tasks info topic
    ros::Subscriber goalT1Sub; // ROS subcriber for goal topic in task 1
    ros::Subscriber goalT2Sub; // ROS subcriber for goal topic in task 2
    ros::Subscriber goalT4Sub; // ROS subcriber for goal topic in task 4
    ros::Subscriber goalReachedSub; // ROS subcriber to determine if the wamv reached its goal
    ros::Publisher goalPub; // ROS Publisher for goal pose
    geographic_msgs::GeoPoseStamped goal; // Stores goal message for task 1
    geographic_msgs::GeoPath waypoints; // Stores waypoints message for task 2 and 4
    geographic_msgs::GeoPath obstway;
    int waypointsNo; // Keeps track of the number of waypoints
    int goalNo; // Keeps track of the current goal
    int obstacleNo; // Keeps track of the current goal
    double location[2];   //x and y pos of robot center
    geographic_msgs::GeoPath minigoals;  //subgoals that may need to be achieved to specify details of a path (investigate appending to waypoints)
    int minigoalNo;  // Keeps track of the current mini goal
    std::vector<float> path;
    std::array<int, 2> tempgoal;
    std_msgs::Bool goalReachedFlag; // Boolean to indicate if the wamv reached the goal
    bool flag;
    bool flag2;
    float offset;
};

#endif
