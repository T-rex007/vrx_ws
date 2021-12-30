#include <ros/ros.h>
#include "tasks/processing.h"

#define SAMPLE 100

int main(int argc, char **argv)
{
    // Initialize a ROS node called 'tasks'
    ros::init(argc, argv, "tasks");

    // Spinners allows ROS messages to be processed during blocking services
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Main access point to communications with the ROS system
    ros::NodeHandle n;
    ros::Rate loop_rate(SAMPLE);

    // Initialise processing class
    Processing process(&n);

    // Inidicate package started successfully
    ROS_INFO("Tasks Package: started successfully");

    // Do pre-processing before Initial State
    ROS_INFO("Tasks Package: pre-processing before Inital State");
    /* <-- Insert Code Here --> */
    while (process.taskState == "NULL"){ loop_rate.sleep(); }

    // Perform processing during Initial State
    ROS_INFO("Tasks Package: processing in Inital State");
    /* <-- Insert Code Here --> */
    process.InitSubsAndPubs();
    while (process.taskState == "initial"){ loop_rate.sleep(); }

    // Perform processing during Ready State
    ROS_INFO("Tasks Package: processing in Ready State");
    /* <-- Insert Code Here --> */
    while (process.taskState == "ready"){ loop_rate.sleep(); }

    // Perform processing during Running State
    ROS_INFO("Tasks Package: processing in Running State");
    /* <-- Insert Code Here --> */
    process.PublishMessages();
    while (ros::ok()){ loop_rate.sleep(); }

    // Wait for this node to be shutdown, whether through Ctrl-C, ros::shutdown() or similar
    ros::waitForShutdown();
    return 0;
}
