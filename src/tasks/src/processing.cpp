#include "tasks/processing.h"


/// @brief Constructor for the Processing
Processing::Processing(ros::NodeHandle *node_handle)
    : node(*node_handle)
{
    tasksSub = node.subscribe("/vrx/task/info", QUEUE, &Processing::TasksCallback, this);
}

/// @brief Destructor for the Processing
Processing::~Processing()
{
    return;
}

/// @brief Initialise the correct subscribers and publishers based on the task
void Processing::InitSubsAndPubs()
{
    goalPub = node.advertise<geographic_msgs::GeoPoseStamped>("goal", QUEUE);

    if (taskName == "station_keeping")
    {
        goalT1Sub = node.subscribe("/vrx/station_keeping/goal", QUEUE, &Processing::GoalT1Callback, this);
    }

    if (taskName == "wayfinding")
    {
        goalT2Sub = node.subscribe("/vrx/wayfinding/waypoints", QUEUE, &Processing::GoalT2Callback, this);
    }

    if (taskName == "perception"){}

    if (taskName == "wildlife"){}

    if (taskName == "gymkhana"){}

    if (taskName == "scan_dock_deliver"){}
}

/// @brief Initialise the correct nodes based on the task
void Processing::InitNodes()
{
    // Start nav node in Navigation Package
    std::system("roslaunch navigation nav.launch");
}

/// @brief Callback function for tasks subscriber
void Processing::TasksCallback(const vrx_gazebo::Task msg)
{
    if (taskState != msg.state)
        ROS_INFO("Tasks Callback: %s task - %s state", msg.name.c_str(), msg.state.c_str());

    taskName = msg.name;
    taskState = msg.state;
}

/// @brief Callback function for goal subscriber in task 1
void Processing::GoalT1Callback(const geographic_msgs::GeoPoseStamped msg)
{
    goal = msg;
}

/// @brief Callback function for goal subscriber in task 2
void Processing::GoalT2Callback(const geographic_msgs::GeoPath msg)
{
    waypoints = msg;
}
