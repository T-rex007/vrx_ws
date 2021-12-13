#ifndef _PID_H_
#define _PID_H_

#include <ros/ros.h>
#include <std_msgs/Float32.h>

class PID
{
public:
    /// @brief Constructor for the PID
    explicit PID(ros::NodeHandle *node_handle);

    /// @brief Destructor for the PID
    ~PID();

private:
    ros::NodeHandle node; // ROS node handler
};

#endif
