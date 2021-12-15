#ifndef _PID_H_
#define _PID_H_

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <time.h>

class PID
{
public:
    /// @brief Constructor for the PID
    explicit PID(ros::NodeHandle *node_handle);

    /// @brief Destructor for the PID
    ~PID();

    /// @brief Compute output
    float Compute(float input);

    ///@brief Set the new reference of the PID
    void SetRef(float ref);

    ///@brief manually set the gains of the PID controller
    void SetGains(float p, float i, float d);

    ///@brief tune PID to automatically set gains
    void TuneGains();

private:
    ros::NodeHandle node; // ROS node handler
    float setpoint;
    float kp;
    float ki;
    float kd;
    float last_error;
    ros::Time last_time;
};

#endif
