#include "tasks/processing.h"

/// @brief Constructor for the Processing
Processing::Processing(ros::NodeHandle *node_handle)
    : node(*node_handle)
{
    // Initialize subscribers
    tasksSub = node.subscribe("/vrx/task/info", QUEUE, &Processing::TasksCallback, this);
    gps = node.subscribe("/wamv/sensors/gps/gps/fix", QUEUE, &Processing::GPSCallback, this);
}

/// @brief Destructor for the Processing
Processing::~Processing()
{
    return;
}

/// @brief Initialize the correct subscribers and publishers based on the task
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

    if (taskName == "perception") {}

    if (taskName == "wildlife") {}

    if (taskName == "gymkhana") {}

    if (taskName == "scan_dock_deliver") {}
}

/// @brief Publish the correct messages based on the task
void Processing::PublishMessages()
{
    goalPub.publish(goal);
}

/// @brief Finds optimum path of the WAMC given a cost matrix using recursion
std::vector<float> Processing::GoalSort(std::vector<std::vector<float>> matrix, int current, std::vector<int> nodes)
{
    // Initialize variables
    std::vector<float> min{std::numeric_limits<float>::infinity()};
    std::vector<float> total;
    std::vector<int> temp_nodes;

    // Check if 
    if(nodes.empty())
    {
        total.push_back(0);
        total.push_back(current);
        return total;
    }

    for(int i = 0; i < nodes.size(); i++)
    {
        temp_nodes = nodes;
        temp_nodes.erase(temp_nodes.begin()+i);

        total = GoalSort(matrix, nodes.at(i), temp_nodes);

        total.at(0) = total.at(0) + (matrix.at(current)).at(nodes.at(i));

        if(total.at(0) < min.at(0))
        {
            min = total;
        }
    }

    min.push_back(current);

    return min;
}

/// @brief Function to get the distance matrix and optimize the goal pathing
void Processing::GetMatrix()
{

    geographic_msgs::GeoPath temp;
    std::vector<std::vector<float>> distance_matrix;
    std::vector<std::array<double, 2>> goals;
    std::array<double, 2> gal = {location[0], location[1]};
    std::vector<float> row;
    std::vector<int> nodes;
    double goal_vector[2];
    float distance;

    goals.push_back(gal);

    for(int i = 0; i < waypointsNo; i++)
    {
        gal[0] = waypoints.poses[i].pose.position.longitude;
        gal[1] = waypoints.poses[i].pose.position.latitude;
        goals.push_back(gal);
    }

    for(int i = 0; i <= waypointsNo; i++)
    {

        row.clear();

        for(int x = 0; x <= waypointsNo; x++)
        {
            goal_vector[0] = ((goals.at(i))[0] - (goals.at(x))[0])*6371000*M_PI/180;
            goal_vector[1] = ((goals.at(i))[1] - (goals.at(x))[1])*6371000*M_PI/180;
            distance = sqrt(std::pow(goal_vector[0], 2) + std::pow(goal_vector[1], 2));

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
    path.pop_back();
    std::reverse(path.begin(), path.end());
}


/// @brief circle object
std::vector<std::array<float, 3>> Processing::Circle(double center[2], int num_pts, float r, double ref0, double ref1)
{
    // Calculate eqn of line
    float m = (center[1] - ref1) / (center[0] - ref0);
    float intercept = center[1] - (m*center[0]);
    // ROS_INFO("intercept: %s", std::to_string(intercept).c_str());

    // Sub y into x and determine a,b,c for quadratic formula 
    float a = (m*m) + 1;
    float b = (-2*center[1]*m) + (2*intercept*m) + (-2*center[0]);
    float c = std::pow(center[0], 2) + std::pow(center[1], 2) - (2*center[1]*intercept) + std::pow(intercept, 2) - std::pow(r, 2);

    // Calculate points of intersection using quadratic formula
    float x1 = (-b + sqrt(std::pow(b, 2) - (4 * a * c))) / (2 * a);
    float x2 = (-b - sqrt(std::pow(b, 2) - (4 * a * c))) / (2 * a);

    // Determine corresponding y-coordinates
    float y1 = (m * x1) + intercept;
    float y2 = (m * x2) + intercept;

    // Determine the set of points which is closest to the WAMV
    float dist1 = sqrt(std::pow((ref0 - x1), 2) + std::pow((ref1 - y1), 2));
    float dist2 = sqrt(std::pow((ref0 - x2), 2) + std::pow((ref1 - y2), 2));

    float x = 0;
    float y = 0;

    if(dist1 < dist2)
    {
        x = x1;
        y = y1;
    }else
    {
        x = x2;
        y = y2;
    }
    // ROS_INFO("circle check 1");
    // Calculate starting/offset angle based on point of intersection
    float offset_angle = ((atan2(x - center[0], y - center[1])) * (180 / M_PI)) - 90;

    if(offset_angle < 0)
    {
        offset_angle = M_PI*(-offset_angle / 180);
    }else
    {
        offset_angle = M_PI*((360 - offset_angle) / 180);
    }

    // Calculate equal angle segments
    float angle = (360 / num_pts) * (M_PI/180);

    // Generate specified number of points(position and orientation)
    std::vector<std::array<float, 3>> points;
    for(int i = 0; i < num_pts; i++)
    {
        float pos_x = (r * cos((i*angle) + offset_angle)) + center[0];
        float pos_y = (r * sin((i*angle) + offset_angle)) + center[1];
        float ang =  ((((i * angle) + offset_angle) * 180 / M_PI) + 180);
        ang = fmod(ang, 360);

        points.push_back({pos_x, pos_y, ang});
    }
    return points;
}

///@brief converts x and y with wamv ref to vector of geostamped
void Processing::CarttoGeo(std::vector<std::array<float, 3>> points, double x, double y)
{
    // ROS_INFO("convert");
    geographic_msgs::GeoPoseStamped temp;
    double yaw;
    tf2::Quaternion q;
    geometry_msgs::Quaternion quat;

    for(int i = 0; i < points.size(); i++)
    {
        temp.pose.position.longitude = ((points.at(i))[0]/(6371000*M_PI/180)) + x;
        temp.pose.position.latitude = ((points.at(i))[1]/(6371000*M_PI/180)) + y;
        temp.pose.position.altitude = i;
        yaw = (points.at(i))[2];
        
        if(yaw > 180)
        {
            yaw -= 360;
        }
        yaw = yaw * M_PI/180;
        q.setRPY(0, 0, yaw);
        q.normalize();
        quat = tf2::toMsg(q);

        temp.pose.orientation = quat;
        minigoals.poses.push_back(temp);
    }
}

///@brief gets minigoal points based on obstacle encountered
void Processing::Special(std::string obstacle, double center[2])
{
    // ROS_INFO("special");
    minigoals.poses.clear();
    minigoalNo = 0;
    double converted[2];
    double ref[2];
    double temp = location[0];
    double temp2 = location[1];

    converted[0] = (center[0] - temp)*6371000*M_PI/180;
    converted[1] = (center[1] - temp2)*6371000*M_PI/180;
    std::vector<std::array<float, 3>> points = Circle(converted, 10, 7);
    // ROS_INFO("after circle");
    points.push_back(points.at(0));
    
    // for(int i = 0; i < 10; i++)
    // {
    //     ROS_INFO("pt %s x: %s", std::to_string(i).c_str(), std::to_string(points.at(i)[0]).c_str());
    //     ROS_INFO("pt %s y: %s", std::to_string(i).c_str(), std::to_string(points.at(i)[1]).c_str());
    // }

    if(obstacle == "platypus")
    {
        
    }else if(obstacle == "crocodile")
    {

    }
    // ROS_INFO("before cart");
    CarttoGeo(points, temp, temp2);
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
    
    GetMatrix();
  
    ROS_INFO("current: %s", std::to_string(path.at(goalNo) - 1).c_str());
    goal = waypoints.poses[path.at(goalNo)-1];
    double pos[2] = {goal.pose.position.longitude, goal.pose.position.latitude};
    Special("turtle", pos);
    goal = minigoals.poses[minigoalNo];

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
            // ROS_INFO("GoalNo: %s", std::to_string(goalNo).c_str());
            if(minigoalNo < minigoals.poses.size()-1)
            {
                ROS_INFO("GoalReached Callback: goal achieved");
                minigoalNo++;
                goal = minigoals.poses[minigoalNo];
                PublishMessages();
            }else if (goalNo != (waypointsNo - 1))
            {
                goalNo++;
                ROS_INFO("GoalReached Callback: goal achieved");
                ROS_INFO("Goal: %s", std::to_string(path.at(goalNo) - 1).c_str());
                goal = waypoints.poses[path.at(goalNo)-1];
                double pos[2] = {goal.pose.position.longitude, goal.pose.position.latitude};
                Special("turtle", pos);
                PublishMessages();
                goal = minigoals.poses[minigoalNo];
            }
            else
            {
                ROS_INFO("GoalReached Callback: last goal achieved");
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
