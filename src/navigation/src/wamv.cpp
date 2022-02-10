#include "navigation/wamv.h"

/// @brief Constructor for the WAMV
WAMV::WAMV(ros::NodeHandle *node_handle)
    : node(*node_handle)
{
    // Initlialise subscribers
    goal_node = node.subscribe("/tasks/goal", QUEUE, &WAMV::GoalCallback, this);
    gps = node.subscribe("/wamv/sensors/gps/gps/fix", QUEUE, &WAMV::GPSCallback, this);
    imu = node.subscribe("/wamv/sensors/imu/imu/data", QUEUE, &WAMV::IMUCallback, this);

    // Initialise goal and thrusters publishers
    goal_reached_pub = node.advertise<std_msgs::Bool>("/navigation/goal_reached", QUEUE);

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

///@brief Returns the trajectory of the target
float WAMV::ReturnAngle()
{
   return heading;
}

float WAMV::ReturnTargetAngle()
{
   return target_angle;
}

///@brief Calculates the Angle the boat will turn
float WAMV::CalcAngle()
{
    float difference;
    difference = heading - goal[2];  

    if (abs(difference) > 180) 
    { 
        difference = (-difference/abs(difference)) * (360 - abs(difference));
    } 

    return difference;
}   

float WAMV::CalcRef()
{
    float difference;
    difference = target_angle;

    
    if (abs(difference) > 180)  
    { 
        difference = (-difference/abs(difference)) * (360 - abs(difference));
    }

    //ROS_INFO("2: Differecne: %f", difference);
    //ROS_INFO("3: Target Angle: %f", target_angle);

    return difference;
}

///@brief Turns the boat
std::array<std::tuple<float, float>, 4> WAMV::MajorControl(float ref, float range)
{
    std::array<std::tuple<float, float>, 4> thrusters;
    float back;
    float scale;
    //float angle = (180/(1+exp(-0.04 * ref))) - 90;
    float angle = static_cast<float>(ref * M_PI / 180);

    //ROS_INFO("1: Ref: %f", ref);

    if (abs(ref) <= range)
    {
        back = 1;   
        scale = 1 - ((abs(ref)/(abs(ref) + range)));
    }
    else
    {
        back = 0;
        scale = (abs(ref)/(abs(ref) + range))*0.5;
    }

    float cmd[4] = {scale, scale, back, back};
    float thruster_angle[4] = {ref, ref, 0, 0};

    for (int i = 0; i < 4; i++)
    {
        thrusters[i] = std::make_tuple(cmd[i], thruster_angle[i]);
    }

    return thrusters;
}

void WAMV::GoalReached(const bool flag)
{
    std_msgs::Bool msg;
    msg.data = flag;
    goal_reached_pub.publish(msg);
}

///@brief Update the target vector and reference angle to goal
void WAMV::UpdateAngle()
{
    double temp[2];
    double target[2];
    float ref_angle;
    float distance;

    temp[0] = (goal[0] - location[0])*6371000*M_PI/180;
    temp[1] = (goal[1] - location[1])*6371000*M_PI/180;
    target[1] = temp[0] * cos(M_PI*(heading/180)) + temp[1] * sin(M_PI*(heading/180));   
    target[0] = -temp[1] * cos(M_PI*(heading/180)) + temp[0] * sin(M_PI*(heading/180));


    distance = sqrt(std::pow(target[0], 2) + std::pow(target[1], 2));

    // Angle between euclidean distance vector and north reference in degrees
    ref_angle = asin(target[0]/distance);  
    ref_angle = (ref_angle/M_PI) * 180;    
    
    if(target[1] > 0)
    {
        ref_angle = ref_angle  + 360;
        ref_angle = fmod(ref_angle, 360);
    }
    else
    {
        ref_angle = -1 * ref_angle + 180;
    }

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
    yaw = yaw * 180.0 / M_PI; 

    if (yaw < 0) 
    {
        yaw += 360.0;
    }

    return yaw;
}

std::array<std::tuple<float, float>, 4> WAMV::MinorControl(float x, float y, float angle, float a, float ratio)
{
    std::array<std::tuple<float, float>, 4> thrusters;
    float scalx;
    float scaly;
    float scala;
    float O_x;
    float O_y;
    float O_a;

    // scalx = (2/(1+exp(-0.05 * x))) - 1;
    // scaly = (2/(1+exp(-0.05 * y))) - 1;
    // scala = (abs(angle)/(abs(angle) + 60));

    scalx = x;
    scaly = y;
    scala = 1;

    O_x = scalx;
    O_y = scaly;
    //O_a = (angle/abs(angle))* scala;
    O_a = angle;

    float cmd[4] = {O_x + O_y + O_a, -O_x + O_y - O_a, -O_x + O_y + O_a, O_x + O_y - O_a};

    //ROS_INFO("1: cmd[%f, %f, %f, %f]", cmd[0], cmd[1], cmd[2], cmd[3]);

    float thruster_angles[4] = {-45, 45, 45, -45};

    float distance = sqrt(std::pow(target_vector[0], 2) + std::pow(target_vector[1] , 2));

    float scalar;
    float max = abs(cmd[0]);

    for(int i = 1; i < 4; i++)
    {
        if (abs(cmd[i]) > max) 
        {
            max = abs(cmd[i]);
        }
    }

    //scalar = (2/(1+exp(-0.33 * distance))) - 1;
    scalar = 0.01;

    for (int i = 0; i < 4; i++)
    {
        //cmd[i] = (cmd[i]/max) * scalar;
        cmd[i] = cmd[i] * scalar;
    }

    for(int i = 0; i<4; i++)
    {
        thrusters[i] = std::make_tuple(cmd[i], thruster_angles[i]);
    }

    //ROS_INFO("2: cmd[%f, %f, %f, %f]", cmd[0], cmd[1], cmd[2], cmd[3]);

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

void WAMV::UpdateGoal()
{
    goal[0] = location[0];
    goal[1] = location[1];
    goal[2] = heading;
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
    linear_acc[0] = msg.linear_acceleration.x;
    linear_acc[1] = msg.linear_acceleration.y;
    heading  = ConvertOrientation(msg.orientation);
}

void WAMV::GoalCallback(const geographic_msgs::GeoPoseStamped msg)
{
    goal[0] = msg.pose.position.longitude;
    goal[1] = msg.pose.position.latitude;
    goal[2] = ConvertOrientation(msg.pose.orientation);
    GoalReached(false);
    UpdateAngle();
}
