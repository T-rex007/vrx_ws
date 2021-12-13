#ifndef _WAMV_H_
#define _WAMV_H_

#include <ros/ros.h>
#include <std_msgs/Float32.h>

class WAMV
{
public:
    /// @brief Constructor for the WAMV
    explicit WAMV(ros::NodeHandle *node_handle);

    /// @brief Destructor for the WAMV
    ~WAMV();

private:
    ros::NodeHandle node; // ROS node handler
};

#endif
