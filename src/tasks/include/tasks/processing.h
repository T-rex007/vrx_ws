#ifndef _PROCESSING_H_
#define _PROCESSING_H_

#include <ros/ros.h>
#include "vrx_gazebo/Task.h"
#include <geographic_msgs/GeoPoseStamped.h>
#include <geographic_msgs/GeoPath.h>

#include <math.h>

#define QUEUE 100

class Processing
{
public:
    std::string taskName = "NULL"; // Stores current task name
    std::string taskState = "NULL"; // Stores current task state

    /// @brief Constructor for the Processing
    explicit Processing(ros::NodeHandle *node_handle);

    /// @brief Destructor for the Processing
    ~Processing();

    /// @brief Initialise the correct subscribers and publishers based on the task
    void InitSubsAndPubs();

    /// @brief Initialise the correct nodes based on the task
    void InitNodes();

    /// @brief Callback function for tasks subscriber
    void TasksCallback(const vrx_gazebo::Task msg);

    /// @brief Callback function for goal subscriber in task 1
    void GoalT1Callback(const geographic_msgs::GeoPoseStamped msg);

    /// @brief Callback function for goal subscriber in task 2
    void GoalT2Callback(const geographic_msgs::GeoPath msg);

private:
    ros::NodeHandle node; // ROS node handler
    ros::Subscriber tasksSub; // ROS subscriber for tasks info topic
    ros::Subscriber goalT1Sub; // ROS subcriber for goal topic in task 1
    ros::Subscriber goalT2Sub; // ROS subcriber for goal topic in task 2
    ros::Publisher goalPub; // ROS Publisher for goal pose
    geographic_msgs::GeoPoseStamped goal; // Stores goal message for task 1
    geographic_msgs::GeoPath waypoints; // Stores waypoints message for task 2
};

#endif
