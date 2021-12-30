#include "navigation/wamv.h"

/// @brief Constructor for the WAMV
WAMV::WAMV(ros::NodeHandle *node_handle)
    : node(*node_handle)
{
    goal_node = node.subscribe("/tasks/goal", QUEUE, &WAMV::GoalCallback, this);
    gps = node.subscribe("/wamv/sensors/gps/gps/fix", QUEUE, &WAMV::GPSCallback, this);
    imu = node.subscribe("/wamv/sensors/imu/imu/data", QUEUE, &WAMV::IMUCallback, this);

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
    float scale = (abs(ref)/(abs(ref) + 64))*0.25;
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

// void WAMV::GetMatrix()
// {
//     std::vector<std::vector<float>> distance_matrix;
//     std::vector<float> row;
//     double goal_vector[2];
//     float distance;

//     for(int i = 0; i < goals.size(); i++)
//     {
//         row.clear();
//         for(int x = 0; x < goals.size(); x++)
//         {
//             goal_vector[0] = ((goals.at(i))[0] - (goals.at(x))[0])*6371000*M_PI/180;
//             goal_vector[1] = ((goals.at(i))[0] - (goals.at(x))[0])*6371000*M_PI/180;
//             distance = sqrt(pow(goal_vector[0], 2) + (goal_vector[1], 2));
//             row.push_back(distance);
//         }
//         distance_matrix.push_back(row);
//     }

//     //sort goals from tsp
//     UpdateAngle();
// }

void WAMV::GoalReached(const bool flag)
{
    // if(goals.size() < 1){return;}
    // goals.erase(goals.begin());
    // UpdateAngle();
    //consider resorting goals 
    std_msgs::Bool msg;
    msg.data = flag;
    goal_reached_pub.publish(msg);
}

///@brief Update the target vector and reference angle to goal
void WAMV::UpdateAngle()
{
    //if(goals.size() <= 0){/*target_vector[0] = 0; target_vector[1] = 0; target_angle = 0;*/ return;}

    double temp[2];
    double target[2];
    //std::array<double, 3> goal = goals.front();
    float ref_angle;
    float distance;
    temp[0] = (goal[0] - location[0])*6371000*M_PI/180;
    temp[1] = (goal[1] - location[1])*6371000*M_PI/180;
    target[1] = temp[0] * cos(M_PI*(heading/180)) + temp[1] * sin(M_PI*(heading/180));   //pray this works by assuming the rate of change should be small enough to always have diff < 90 or 180 maybe
    target[0] = -temp[1] * cos(M_PI*(heading/180)) + temp[0] * sin(M_PI*(heading/180));
    // target[0] = temp[0];
    // target[1] = temp[1];
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
    float max = cmd[0];

    for(int i = 1; i < 4; i++)
    {
        if(cmd[i] > max){max = cmd[i];}
    }

    // scalar = distance / (distance +a);
    scalar = 1;

    for(int i = 0; i < 4; i++)
    {
        cmd[i] = (cmd[i]/(abs(O_x)+ abs(O_y))) * scalar;
        // cmd[i] = (cmd[i]/max) * scalar;
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
    // if(goals.size() <= 0)
    // {
    //     return {0, 0};
    // }
    // return goals.front();

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
    heading  = ConvertOrientation(msg.orientation);
    // ROS_INFO("head: %s", std::to_string(heading).c_str());
}

void WAMV::GoalCallback(const geographic_msgs::GeoPoseStamped msg)
{
    // read data into multidimentional vector or array
    // float size = sizeof(msg.poses)/sizeof(msg.poses[0]);
    //  ROS_INFO("%s", std::to_string(sizeof(msg.poses[1])).c_str());
    
    //float size = 3;
    //std::array<double, 3> goal;

    // for(int i = 0; i < size; i++)
    // {
    //     goal[0] = msg.poses[i].pose.position.longitude;
    //     goal[1] = msg.poses[i].pose.position.latitude;
    //     goal[2] = ConvertOrientation(msg.poses[i].pose.orientation);
    //     goals.push_back(goal);
    // }

    //GetMatrix();
    
    goal[0] = msg.pose.position.longitude;
    goal[1] = msg.pose.position.latitude;
    goal[2] = ConvertOrientation(msg.pose.orientation);
    GoalReached(false);
    UpdateAngle();
}