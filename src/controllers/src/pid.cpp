#include "controllers/pid.h"

/// @brief Constructor for the PID
PID::PID(ros::NodeHandle *node_handle)
    : node(*node_handle)
{
    setpoint = 0;
    auto_flag = false;
    pid_direction = DIRECT;

    return;
}

/// @brief Destructor for the PID
PID::~PID()
{
    return;
}

/// @brief Compute PID output
float PID::Compute(float input)
{
    if (!auto_flag)
    {
        return 0;
    }

    current_input = input;

    ros::Time now;
    double difference;
    double error_p;
    double error_d;

    now = ros::Time::now();
    difference = (now - last_time).toSec();

    if (difference >= sample_time)
    {
        // Calculate error variables
        error_p = setpoint - input; 
        i_term += ki * error_p;
        error_d = input - last_input;

        // Clamp i_term to output limits
        if (i_term > output_max)
        {
            i_term = output_max;
        }
        else if (i_term < output_min)
        {
            i_term = output_min;
        }

        // Compute PID output
        pid_output = (kp * error_p) + (i_term) - (kd * error_d);
        
        // Clamp PID output to output limits
        if (pid_output > output_max)
        {
            pid_output = output_max;
        }
        else if (pid_output < output_min)
        {
            pid_output = output_min;
        }

        // Remember values for next compute iteration
        last_time = now;
        last_input = input;
    }

    return pid_output;
}

///@brief Set the new reference of the PID
void PID::SetRef(float ref)
{
    setpoint = ref;
}

///@brief Set the mode (manual/automatic) of the PID
void PID::SetMode(float mode)
{
    bool new_auto_flag = (mode == AUTOMATIC);

    if (new_auto_flag && !auto_flag)
    {
        last_input = current_input;

        i_term = pid_output;

        if (i_term > output_max)
        {
            i_term = output_max;
        }
        else if (i_term < output_min)
        {
            i_term = output_min;
        }
    }

    auto_flag = new_auto_flag;
}

///@brief Set the direction (direct/reverse) of the PID
void PID::SetPIDDirection(int direction)
{
    pid_direction = direction;
}

///@brief Set the sample time (seconds) of the PID controller 
void PID::SetSampleTime(float t)
{
    sample_time = t;
}

///@brief Clamp the output of the PID controller
void PID::SetOutputLimits(float min, float max)
{
    if (min > max)
    {
        return;
    }
    else
    {
        output_max = max;
        output_min = min;
    }

    if (pid_output > output_max)
    {
        pid_output = output_max;
    }
    else if (pid_output < output_min)
    {
        pid_output = output_min;
    }

    if (i_term > output_max)
    {
        i_term = output_max;
    }
    else if (i_term < output_min)
    {
        i_term = output_min;
    }
}

///@brief Manually set the gains of the PID controller
void PID::SetGains(float p, float i, float d)
{
    if (p < 0 || i < 0 || d < 0)
    {
        return;
    }

    kp = p;
    ki = i * sample_time;
    kd = d / sample_time;

    if (pid_direction == REVERSE)
    {
        kp = 0 - kp;
        ki = 0 - ki;
        kd = 0 - kd;
    }
}

///@brief Tune PID to automatically set gains
void PID::TuneGains()
{
    float p;
    float i;
    float d;

    // SetGains(p, i, d);
}
