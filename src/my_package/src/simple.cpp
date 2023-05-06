// This is a simple C++ program using the Robot Operating System (ROS) 2.0 library to create a ROS node.

// The program starts by including the necessary header files for ROS 2.0 and logging macros. The main function initializes the ROS 2.0 context using rclcpp::init with the provided command line arguments argc and argv.

// It then creates a shared pointer to a rclcpp::Node object named "simple_node". A node is a process that performs computation in the ROS 2.0 graph. The make_shared function creates a new instance of a node, which is managed by a shared pointer.

// After creating the node, the program enters the event loop by calling rclcpp::spin. This function blocks the program until the node is shut down. While in the event loop, the node will listen for incoming messages and handle any callbacks associated with them.

// Finally, the program shuts down the ROS 2.0 context by calling rclcpp::shutdown.

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv); // Initialize the ROS 2.0 context with provided argc and argv

    // Creates a shared pointer to a rclcpp::Node object named "simple_node". A node is a process that performs computation in the ROS 2.0 graph.
    auto node = rclcpp::Node::make_shared("simple_node");

    // Enters the event loop by calling rclcpp::spin. This function blocks the program until the node is shut down.
    rclcpp::spin(node);

    // Shuts down the ROS 2.0 context by calling rclcpp::shutdown.
    rclcpp::shutdown();

}