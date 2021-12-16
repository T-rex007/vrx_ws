#include "controllers/pid.h"

/// @brief Constructor for the PID
PID::PID(ros::NodeHandle *node_handle)
    : node(*node_handle)
{
    return;
}

/// @brief Destructor for the PID
PID::~PID()
{
    return;
}

/// @brief Compute output
float PID::Compute(float input)
{
    ros::Time now;
    double difference;
    double error_p;
    double error_d;
    float output;

    now = ros::Time::now();
    difference = (now - last_time).toSec();
    
    error_p = setpoint - input;
    error_i += (error_p * difference);
    error_d = (error_p - last_error) / difference;

    output = (kp * error_p) + (ki * error_i) + (kd * error_d);

    last_time = now;
    last_error = error_p;

    return output;
}

///@brief Set the new reference of the PID
void PID::SetRef(float ref)
{
    setpoint = ref;
}

///@brief manually set the gains of the PID controller
void PID::SetGains(float p, float i, float d)
{
    kp = p;
    ki = i;
    kd = d;
}

///@brief tune PID to automatically set gains
void PID::TuneGains()
{
    float p;
    float i;
    float d;

    // SetGains(p, i, d);
}

