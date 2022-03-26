#ifndef _PID_H_
#define _PID_H_

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <time.h>

#define MANUAL 0        // user sets the PID output
#define AUTOMATIC 1     // PID output is automatically calculated

#define DIRECT 0        // increase in output gives increase in input
#define REVERSE 1       // increase in output gives decrease in input

class PID
{
public:
    /// @brief Constructor for the PID
    explicit PID(ros::NodeHandle *node_handle);

    /// @brief Destructor for the PID
    ~PID();

    /// @brief Compute PID output
    float Compute(float input);

    ///@brief Set the new reference of the PID
    void SetRef(float ref);

    ///@brief Set the mode (manual/automatic) of the PID
    void SetMode(float mode);

    ///@brief Set the direction (direct/reverse) of the PID
    void SetPIDDirection(int direction);

    ///@brief Set the sample time (seconds) of the PID controller
    void SetSampleTime(float t);

    ///@brief Clamp the output of the PID controller
    void SetOutputLimits(float min, float max);

    ///@brief Manually set the gains of the PID controller
    void SetGains(float p, float i, float d);

    ///@brief Tune PID to automatically set gains
    void TuneGains();

private:
    ros::NodeHandle node;
    bool auto_flag;
    int pid_direction;
    float setpoint;
    float kp;
    float ki;
    float kd;
    float i_term;
    float output_min;
    float output_max;
    float pid_output;
    double error_i;
    double current_input;
    double last_input;
    double sample_time;
    ros::Time last_time;
};

#endif
