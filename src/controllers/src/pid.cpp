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
