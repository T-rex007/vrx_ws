#include "navigation/wamv.h"

/// @brief Constructor for the WAMV
WAMV::WAMV(ros::NodeHandle *node_handle)
    : node(*node_handle)
{
    gps = node.subscribe("/wamv/sensors/gps/gps/fix", QUEUE, &WAMV::GPSCallback, this);
    imu = node.subscribe("/wamv/sensors/imu/imu/data", QUEUE, &WAMV::IMUCallback, this);
    goal_node = node.subscribe("/vrx/station_keeping/goal", QUEUE, &WAMV::GoalCallback, this);

    thrusters_pub[0] = node.advertise<std_msgs::Float32>("/wamv/thrusters/left_front_thrust_cmd", QUEUE);
    thrusters_pub[1] = node.advertise<std_msgs::Float32>("/wamv/thrusters/left_front_thrust_angle", QUEUE);
    thrusters_pub[2] = node.advertise<std_msgs::Float32>("/wamv/thrusters/right_front_thrust_cmd", QUEUE);
    thrusters_pub[3] = node.advertise<std_msgs::Float32>("/wamv/thrusters/right_front_thrust_angle", QUEUE);
    thrusters_pub[4] = node.advertise<std_msgs::Float32>("/wamv/thrusters/left_rear_thrust_cmd", QUEUE);
    thrusters_pub[5] = node.advertise<std_msgs::Float32>("/wamv/thrusters/left_rear_thrust_angle", QUEUE);
    thrusters_pub[6] = node.advertise<std_msgs::Float32>("/wamv/thrusters/right_rear_thrust_cmd", QUEUE);
    thrusters_pub[7] = node.advertise<std_msgs::Float32>("/wamv/thrusters/right_rear_thrust_angle", QUEUE);
}

/// @brief Destructor for the WAMV
WAMV::~WAMV()
{
    return;
}


///@brief Update the goal position (x, y)
void WAMV::UpdateGoal(double longitude, double latitude)
{
    goal[0] = longitude;
    goal[1] = latitude;  //find how to automate finding the end of array

    UpdateAngle();
}

///@brief Returns the trajectory of the target
float WAMV::ReturnAngle()
{
   return target_angle;
}

///@brief Calculates the Angle the boat will turn
float WAMV::CalcAngle(float ref_angle)
{
    float difference;
    difference = ref_angle - heading;
    if (abs(difference) > 180)  //consider accounting for momentum in turning
    { 
        difference = (-difference/abs(difference)) * (360 - abs(difference));

    } 
   return difference;
}

///@brief Turns the boat
std::array<std::tuple<float, float>, 4> WAMV::MajorControl(float ref, float range)
{
    std::array<std::tuple<float, float>, 4> thrusters;
    float back;
    float scale = abs(ref)/(abs(ref) + 64);
    float angle = (180/(1+exp(-0.01 + ref))) - 90;

    if(ref <= range)
    {
        back = 1;   //consider another decay here instead of if
    }else{
        back = 0;
    }

    float cmd[4] = {scale, scale, back, back};
    float thruster_angle[4] = {angle, angle, 0, 0};

    for (int i = 0; i < 4; i++)
    {
        thrusters[i] = std::make_tuple(cmd[i], thruster_angle[i]);
    }

    return thrusters;
}


///@brief Update the target vector and reference angle to goal
void WAMV::UpdateAngle()
{
    double target[2];
    float ref_angle;
    float distance;

    target[0] = (location[0] - goal[0])*6371000*M_PI/180;
    target[1] = (location[1] - goal[1])*6371000*M_PI/180;
    distance = sqrt(pow(target[0], 2) + (target[1], 2));

    ref_angle = asin(target[0]/distance);  //angle between euclidean distance vector and north reference in radians
    ref_angle = (ref_angle/M_PI) * 180;     //put in degrees
    
    if(target[1] > 0)
    {
        ref_angle = remainder((ref_angle + 360), 360);
    }else
        ref_angle = -1 * ref_angle + 180;

    std::copy(target, target + 2, target_vector);
    target_angle = ref_angle;
}

void WAMV::UpdateThruster(std::array<std::tuple<float, float>, 4> thrusters)
{
    std_msgs::Float32 msg;
    float temp;

    for(int i = 0; i < 4; i++)
    {
        msg.data = std::get<0>(thrusters[i]);
        thrusters_pub[i * 2].publish(msg);
        temp = std::get<1>(thrusters[i]);
        msg.data = (temp / 180) * M_PI;
        thrusters_pub[(i * 2) + 1].publish(msg);
    }
}

double WAMV::ConvertOrientation(geometry_msgs::Quaternion quat)
{
    double roll;
    double pitch;
    double yaw;
    tf::Quaternion q;
    tf::quaternionMsgToTF(quat, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw = yaw * 180.0 / M_PI; // conversion to degrees
    if(yaw < 0) 
    {
        yaw += 360.0;
    }
    return yaw;
}

std::array<std::tuple<float, float>, 4> WAMV::MiniControl(float x, float y, float O_a, float a, float ratio)
{
    std::array<std::tuple<float, float>, 4> thrusters;
    float scalx;
    float scaly;
    float O_x;
    float O_y;

    // float x = log*6371000*M_PI/180;
    // float y = lat*6371000*M_PI/180;
    // ROS_INFO("compute");
    // ROS_INFO(std::to_string(x).c_str());
    // ROS_INFO(std::to_string(y).c_str());
    scalx = abs(x)/(abs(x) + a);
    scaly = abs(y)/(abs(y) + a);
    // scalx = 1;
    // scaly = 1;
    O_x = scalx * x;
    O_y = scaly * y;
    // ROS_INFO(std::to_string(O_x).c_str());
    // ROS_INFO(std::to_string(O_y).c_str());

    float cmd[4] = {O_x + O_y,-O_x + O_y,-O_x + O_y,O_x + O_y};
    float thruster_angles[4] = {-45,45,45,-45};

    float distance = sqrt(pow(target_vector[0], 2) + (target_vector[1] , 2));
    // ROS_INFO(std::to_string(distance).c_str());
    float scalar;

    scalar = distance / (distance +a);
    // scalar = 1;

    for(int i = 0; i < 4; i++)
    {
        // cmd[i] = (cmd[i]/(abs(O_x) + abs(O_y))) * scalar;
        cmd[i] = (cmd[i]/(abs(O_x) + abs(O_y))) * scalar;
    }
    // float temp = (tan((O_a * M_PI) / 360)) / 4;
    // cmd[0] = (pow(distance, 2)/init) * cmd[0]; // Incorporate Ratio
    // cmd[1] = (pow(distance, 2)/init) * cmd[1];
    // cmd[2] = (pow(distance, 2)/init) * cmd[2];
    // cmd[3] = (pow(distance, 2)/init) * cmd[3];

    // for(int i = 0; i < 4; i++)
    // {
    //     if(temp < abs(cmd[i]))
    //     {
    //         temp = abs(cmd[i]);
    //     }
    // }

    for(int i = 0; i<4; i++)
    {
        // cmd[i] = cmd[i] / abs(temp);
        thrusters[i] = std::make_tuple(cmd[i], thruster_angles[i]);
    }

    return thrusters;
}

double* WAMV::ReturnTargetVector()
{
    return target_vector;
}

double* WAMV::ReturnGoal()
{
    return goal;
}

double* WAMV::ReturnLocation()
{
    return location;
}

void WAMV::GPSCallback(const sensor_msgs::NavSatFix msg)
{
    location[0] = msg.longitude;
    location[1] = msg.latitude;
    UpdateAngle();
} 

void WAMV::IMUCallback(const sensor_msgs::Imu msg)
{
    heading = ConvertOrientation(msg.orientation);
}

void WAMV::GoalCallback(const geographic_msgs::GeoPoseStamped msg)
{
    goal[0] = msg.pose.position.longitude;
    goal[1] = msg.pose.position.latitude;
    goal[2] = ConvertOrientation(msg.pose.orientation);
    UpdateAngle();
}



