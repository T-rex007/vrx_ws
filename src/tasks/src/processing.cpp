#include "tasks/processing.h"

/// @brief Constructor for the Processing
Processing::Processing(ros::NodeHandle *node_handle)
    : node(*node_handle)
{
    return;
}

/// @brief Destructor for the Processing
Processing::~Processing()
{
    return;
}