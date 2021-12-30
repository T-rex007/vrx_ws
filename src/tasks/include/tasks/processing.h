#ifndef _PROCESSING_H_
#define _PROCESSING_H_

#include <ros/ros.h>
#include <math.h>

class Processing
{
public:
    /// @brief Constructor for the Processing
    explicit Processing(ros::NodeHandle *node_handle);

    /// @brief Destructor for the Processing
    ~Processing();

    

private:
    ros::NodeHandle node; // ROS node handler
    
};

#endif
