#include "navigation/wamv.h"

/// @brief Constructor for the WAMV
WAMV::WAMV(ros::NodeHandle *node_handle)
    : node(*node_handle)
{
    gps = node.subscribe("/wamv/sensors/gps/gps/fix", QUEUE, &WAMV::GPSCallback, this);
    imu = node.subscribe("/wamv/sensors/imu/imu/data", QUEUE, &WAMV::IMUCallback, this);
    goal_node = node.subscribe("/wrx/station_keeping/goal", QUEUE, &WAMV::GoalCallback, this);

    // left_front_cmd = node.advertise<std_msgs::Float32>("/wamv/thrusters/left_front_thrust_cmd", QUEUE);
    // left_front_angle = node.advertise<std_msgs::Float32>("/wamv/thrusters/left_front_thrust_angle", QUEUE);
    // right_front_cmd = node.advertise<std_msgs::Float32>("/wamv/thrusters/right_front_thrust_cmd", QUEUE);
    // right_front_angle = node.advertise<std_msgs::Float32>("/wamv/thrusters/right_front_thrust_angle", QUEUE);
    // left_rear_cmd = node.advertise<std_msgs::Float32>("/wamv/thrusters/left_rear_thrust_cmd", QUEUE);
    // left_rear_angle = node.advertise<std_msgs::Float32>("/wamv/thrusters/left_rear_thrust_angle", QUEUE);
    // right_rear_cmd = node.advertise<std_msgs::Float32>("/wamv/thrusters/right_rear_thrust_cmd", QUEUE);
    // right_rear_angle = node.advertise<std_msgs::Float32>("/wamv/thrusters/right_rear_thrust_angle", QUEUE);

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

///@brief Update wamv location (x, y) and heading (in degrees)
void WAMV::UpdateLocal(double longitude, double latitude, float theta)
{
    location[0] = longitude;
    location[1] = latitude;   //consider using array of len 3
    heading = theta;

    UpdateAngle();
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
   return angle;
}

///@brief Calculates the Angle the boat will turn
float WAMV::CalcAngle(float ref_angle)
{
    float difference;
    difference = ref_angle - heading;
    if (abs(difference) > 180)  //consider accounting for momentum in turning
    { 
        difference = remainder((360 - difference), 360);

    } 
   return difference;
}

///@brief Turns the boat
std::array<std::tuple<float, float>, 4> WAMV::TurnBoat(float ref_angle)
{
    std::array<std::tuple<float, float>, 4> thrusters;
    float cmd[4] = {0.1,0.1,0.1,0.1};
    float angle[4] = {remainderf(ref_angle,90), remainderf(ref_angle,90),0,0};
    for (int i = 0; i < 4; i++)
    {
        thrusters[i] = std::make_tuple(cmd[i], angle[i]);
    }

    return thrusters;
}


///@brief Update the target vector and reference angle to goal
void WAMV::UpdateAngle()
{
    float target[2];
    float ref_angle;
    float distance;

    target[0] = goal[0] - location[0];
    target[1] = goal[1] - location[1];
    distance = sqrt(pow(target[0], 2) + (target[1], 2));

    ref_angle = asin(target_vector[0]/distance);  //angle between euclidean distance vector and north reference in radians
    ref_angle = (ref_angle/M_PI) * 180;     //put in degrees
    
    if(target[1] > 0)
    {
        ref_angle = remainder((ref_angle + 360), 360);
    }else
        ref_angle = -1 * ref_angle + 180;

    std::copy(target, target + 2, target_vector);
    angle = ref_angle;
}

void WAMV::UpdateThruster(std::array<std::tuple<float, float>, 4> thrusters)
{
    std_msgs::Float32 msg;

    for(int i = 0; i < 4; i++)
    {
        msg.data = std::get<0>(thrusters[i]);
        thrusters_pub[i * 2].publish(msg);
        msg.data = std::get<1>(thrusters[i]);
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

void WAMV::GPSCallback(const sensor_msgs::NavSatFix msg)
{
    location[0] = msg.longitude;
    location[1] = msg.longitude;
    UpdateAngle();
} 

void WAMV::IMUCallback(const sensor_msgs::Imu msg)
{
    heading = ConvertOrientation(msg.orientation);
}

void WAMV::GoalCallback(const geographic_msgs::GeoPoseStamped msg)
{
    goal[0] = msg.pose.position.longitude;
    goal[1] = msg.pose.position.longitude;
    goal[2] = ConvertOrientation(msg.pose.orientation);
    UpdateAngle();
}

