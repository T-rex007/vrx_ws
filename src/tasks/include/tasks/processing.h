#ifndef _PROCESSING_H_
#define _PROCESSING_H_

#include <ros/ros.h>
#include "vrx_gazebo/Task.h"
#include <std_msgs/Bool.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <geographic_msgs/GeoPath.h>
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
    void GetMatrix();

    /// @brief Callback function for tasks subscriber
    void TasksCallback(const vrx_gazebo::Task msg);

    /// @brief Callback function for goal subscriber in task 1
    void GoalT1Callback(const geographic_msgs::GeoPoseStamped msg);

    /// @brief Callback function for goal subscriber in task 2
    void GoalT2Callback(const geographic_msgs::GeoPath msg);

    /// @brief Callback function to indicate the wamv reached the goal
    void GoalReachedCallback(const std_msgs::Bool msg);

    /// @brief Callback function to get GPS location
    void GPSCallback(const sensor_msgs::NavSatFix msg);

private:
    ros::NodeHandle node;                       // ROS node handler
    ros::Subscriber gps;                        // ROS subscriber for GPS topic
    ros::Subscriber tasksSub;                   // ROS subscriber for tasks info topic
    ros::Subscriber goalT1Sub;                  // ROS subcriber for goal topic in task 1
    ros::Subscriber goalT2Sub;                  // ROS subcriber for goal topic in task 2
    ros::Subscriber goalReachedSub;             // ROS subcriber to determine if the wamv reached its goal
    ros::Publisher goalPub;                     // ROS Publisher for goal pose
    geographic_msgs::GeoPoseStamped goal;       // Stores goal message for task 1
    geographic_msgs::GeoPath waypoints;         // Stores waypoints message for task 2
    int waypointsNo;                            // Keeps track of the number of waypoints
    int goalNo;                                 // Keeps track of the current goal
    double location[2];                         // X and Y position of WAMV center
    std::vector<float> path;                    // 
    std_msgs::Bool goalReachedFlag;             // Boolean to indicate if the wamv reached the goal
};

#endif
