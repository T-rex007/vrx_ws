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
    goalPub = node.advertise<geographic_msgs::GeoPoseStamped>("/tasks/goal", QUEUE);

    if (taskName == "station_keeping")
    {
        goalT1Sub = node.subscribe("/vrx/station_keeping/goal", QUEUE, &Processing::GoalT1Callback, this);
    }

    if (taskName == "wayfinding")
    {
        goalT2Sub = node.subscribe("/vrx/wayfinding/waypoints", QUEUE, &Processing::GoalT2Callback, this);
        goalReachedSub = node.subscribe("/navigation/goal_reached", QUEUE, &Processing::GoalReachedCallback, this);
    }

    if (taskName == "perception"){}

    if (taskName == "wildlife"){}

    if (taskName == "gymkhana"){}

    if (taskName == "scan_dock_deliver"){}
}

/// @brief finds optimum path of the wamv give a cost matrix using recursion
std::vector<float> Processing::GoalSort(std::vector<std::vector<float>> matrix, int node, std::vector<int> nodes)
{
    std::vector<float> min{std::numeric_limits<float>::infinity()};
    std::vector<float> total;
    std::vector<int> temp_nodes;

    if(nodes.empty())
    {
        total.push_back(0);
        total.push_back(node);
        return total;
    }
    
    for(int i = 0; i < nodes.size(); i++)
    {
        temp_nodes = nodes;
        temp_nodes.erase(temp_nodes.begin()+i);
        total = GoalSort(matrix, nodes.at(i), temp_nodes);
        total.at(0) = total.at(0) + (matrix.at(node)).at(i);
        if(total.at(0) < min.at(0))
        {
            min = total;
        }
    }

    min.push_back(node);

    return min;
}

/// @brief gets distance matrix between location and all the goals then starts optimization
void Processing::GetMatrix()
{
    geographic_msgs::GeoPath temp;
    std::vector<std::vector<float>> distance_matrix;
    std::vector<std::array<double, 2>> goals;
    std::array<double, 2> goal = {location[0], location[1]};
    std::vector<float> row;
    std::vector<float> path;
    std::vector<int> nodes;
    double goal_vector[2];
    float distance;

    goals.push_back(goal);

    for(int i = 0; i < waypointsNo; i++)
    {
        goal[0] = waypoints.poses[i].pose.position.longitude;
        goal[1] = waypoints.poses[i].pose.position.latitude;
        goals.push_back(goal);
    }

    for(int i = 0; i <= waypointsNo; i++)
    {
        row.clear();
        for(int x = 0; x <= waypointsNo; x++)
        {
            goal_vector[0] = ((goals.at(i))[0] - (goals.at(x))[0])*6371000*M_PI/180;
            goal_vector[1] = ((goals.at(i))[1] - (goals.at(x))[1])*6371000*M_PI/180;
            distance = sqrt(pow(goal_vector[0], 2) + (goal_vector[1], 2));
            row.push_back(distance);
        }
        distance_matrix.push_back(row);
    }

    for(int i = 1; i <= waypointsNo; i++)
    {
        nodes.push_back(i);
    }

    path = GoalSort(distance_matrix, 0, nodes);
    path.erase(path.begin());
    path.erase(path.end());
    std::reverse(path.begin(), path.end());

    temp = waypoints;
    for(int i = 0; i < waypointsNo; i++)
    {
        waypoints.poses[i] = temp.poses[path.at(i)];
    }
}

/// @brief Publish the correct messages based on the task
void Processing::PublishMessages()
{
    goalPub.publish(goal);
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
    ROS_INFO("GoalT1 Callback: received goal");
}

/// @brief Callback function for goal subscriber in task 2
void Processing::GoalT2Callback(const geographic_msgs::GeoPath msg)
{
    waypoints = msg;
    waypointsNo = waypoints.poses.size();
    goalNo = 0;
    goal = waypoints.poses[goalNo];
    ROS_INFO("GoalT2 Callback: %s Waypoints", std::to_string(waypointsNo).c_str());
}

/// @brief Callback function to indicate the wamv reached the goal
void Processing::GoalReachedCallback(const std_msgs::Bool msg)
{
    goalReachedFlag = msg;

    if (taskName == "wayfinding" && taskState == "running")
    {
        if(goalReachedFlag.data)
        {
            if (goalNo == (waypointsNo - 1))
            {
                ROS_INFO("GoalReached Callback: last goal achieved");
            }
            else
            {
                ROS_INFO("GoalReached Callback: goal achieved");
                goalNo++;
                goal = waypoints.poses[goalNo];
                PublishMessages();
            }
        }
    }
}

/// @brief Callback function save the current location of the wamv
void Processing::GPSCallback(const sensor_msgs::NavSatFix msg)
{
    location[0] = msg.longitude;
    location[1] = msg.latitude;
} 