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

float WAMV::ReturnTargetAngle()
{
   return target_angle;
}

///@brief Calculates the Angle the boat will turn
float WAMV::CalcAngle()
{
    float difference;
    difference = heading - goal[2];     //figure out why this works
    if (abs(difference) > 180)  //consider accounting for momentum in turning
    { 
        difference = (-difference/abs(difference)) * (360 - abs(difference));

    } 
    return difference;
}   

float WAMV::CalcRef()
{
    float difference;
    // difference = target_angle - heading;
    difference = target_angle;
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
    float scale;
    float angle = (180/(1+exp(-0.04 * ref))) - 90;

    if(abs(ref) <= range) //not sure if should be abs
    {
        back = 1;   //consider another decay here instead of if
        scale = 1 - ((abs(ref)/(abs(ref) + range)));
    }else{
        back = 0;
        scale = (abs(ref)/(abs(ref) + range))*0.5;
    }

    float cmd[4] = {scale, scale, back, back};
    float thruster_angle[4] = {-angle, -angle, 0, 0};

    for (int i = 0; i < 4; i++)
    {
        thrusters[i] = std::make_tuple(cmd[i], thruster_angle[i]);
    }

    return thrusters;
}

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
    // distance = sqrt(std::pow(temp[0], 2) + std::pow(temp[1], 2));
    distance = sqrt(std::pow(target[0], 2) + std::pow(target[1], 2));

    // ref_angle = asin(temp[0]/distance);  //angle between euclidean distance vector and north reference in radians
    ref_angle = asin(target[0]/distance);  //angle between euclidean distance vector and north reference in radians
    ref_angle = (ref_angle/M_PI) * 180;     //put in degrees

    // ROS_INFO("distance: %s", std::to_string(distance).c_str());

    // distance = sqrt(std::pow(target[0], 2) + std::pow(target[1], 2));
    
    if(target[1] > 0)
    {
        // ROS_INFO("test: %s", std::to_string(ref_angle).c_str());
        ref_angle = ref_angle  + 360;
        ref_angle = fmod(ref_angle, 360);
    }else
    {
        ref_angle = -1 * ref_angle + 180;
    }

    std::copy(target, target + 2, target_vector);
    target_angle = ref_angle;

    // ROS_INFO("head: %s", std::to_string(heading).c_str());
    // ROS_INFO("ref_angle: %s", std::to_string(ref_angle).c_str());
    // ROS_INFO("difference: %s", std::to_string(CalcRef()).c_str());
    // ROS_INFO("tempx: %s", std::to_string(temp[0]).c_str());
    // ROS_INFO("tempy: %s", std::to_string(temp[1]).c_str());
    // ROS_INFO("tx: %s", std::to_string(target[0]).c_str());
    // ROS_INFO("ty: %s", std::to_string(target[1]).c_str());
    
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

std::array<std::tuple<float, float>, 4> WAMV::MiniControl(float x, float y, float angle, float a, float ratio)
{
    std::array<std::tuple<float, float>, 4> thrusters;
    float scalx;
    float scaly;
    float scala;
    float O_x;
    float O_y;
    float O_a;

    // float x = log*6371000*M_PI/180;
    // float y = lat*6371000*M_PI/180;
    // ROS_INFO("compute");
    // ROS_INFO(std::to_string(x).c_str());
    // ROS_INFO(std::to_string(y).c_str());
    scalx = (2/(1+exp(-0.05 * x))) - 1;
    scaly = (2/(1+exp(-0.05 * y))) - 1;
    scala = (abs(angle)/(abs(angle) + 60));
    // scalx = 1;
    // scaly = 1;
    O_x = scalx;
    O_y = scaly;
    O_a = (angle/abs(angle))* scala;
    // ROS_INFO(std::to_string(O_x).c_str());
    // ROS_INFO(std::to_string(O_y).c_str());

    float cmd[4] = {O_x + O_y + O_a, -O_x + O_y - O_a, -O_x + O_y + O_a, O_x + O_y - O_a};
    float thruster_angles[4] = {-45,45,45,-45};

    float distance = sqrt(std::pow(target_vector[0], 2) + std::pow(target_vector[1] , 2));
    // ROS_INFO(std::to_string(distance).c_str());
    float scalar;
    float max = abs(cmd[0]);

    for(int i = 1; i < 4; i++)
    {
        if(abs(cmd[i]) > max){max = abs(cmd[i]);}
    }

    // scalar = distance / (distance +a);
    scalar = (2/(1+exp(-0.33 * distance))) - 1;
    // scalar = 1;

    for(int i = 0; i < 4; i++)
    {
        // cmd[i] = (cmd[i]/(abs(O_x)+ abs(O_y) + abs(O_a))) * scalar;
        cmd[i] = (cmd[i]/max) * scalar;
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

void WAMV::CalcVelocities()
{
    ros::Time now = ros::Time::now();
    double dt = (now - last_time).toSec();;

    // vel[0] = vel[0] + (linear_acc[0] * dt);
    // vel[1] = vel[1] + (linear_acc[1] * dt);



    offset_distances[0] = (location[0] - prev_locations[0])*6371000*M_PI/180;
    offset_distances[1] = (location[1] - prev_locations[1])*6371000*M_PI/180;
    
    vel[0] = offset_distances[0] / dt;
    vel[1] = offset_distances[1] / dt;

    prev_locations[0] = location[0];
    prev_locations[1] = location[1];

    rec_linear_acc[0] = linear_acc[0];
    rec_linear_acc[1] = linear_acc[1];

    //ROS_INFO("X: %f, Y: %f", offset_distances[0], offset_distances[1]);
    //ROS_INFO("VelX: %f, VelY: %f", vel[0], vel[1]);
    //ROS_INFO("AccX: %f, AccY: %f", linear_acc[0], linear_acc[1]);

    last_time = now;
}

double* WAMV::ReturnDistances()
{
    double times[2];

    //x = -c / m
    times[0] = -vel[0] / rec_linear_acc[0];
    times[1] = -vel[1] / rec_linear_acc[1];

    // d = ut + 0.5at^2
    distances[0] = (vel[0] * times[0]) + (0.5 * rec_linear_acc[0] * std::pow(times[0], 2));
    distances[1] = (vel[1] * times[1]) + (0.5 * rec_linear_acc[1] * std::pow(times[1], 2));

    
    //ROS_INFO("X: %f, Y: %f, VelX: %f, VelY: %f, AccX: %f, AccY: %f", distances[0], distances[1], vel[0], vel[1], rec_linear_acc[0], rec_linear_acc[1]);
    //ROS_INFO("^ TimeX: %f, TimeY: %f", times[0], times[1]);

    return distances;
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