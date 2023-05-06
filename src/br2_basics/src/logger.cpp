// This is a simple C++ program that creates a ROS node using the ROS 2.0 library. The program uses the rclcpp (ROS Client Library for C++) package to initialize the ROS 2.0 system, create a node, and publish messages. The program uses the RCLCPP_INFO macro to publish a message containing a counter value to the node's logger.
// The program starts by including the rclcpp header file, which contains the necessary classes and functions to work with ROS 2.0.
// The main function initializes the ROS 2.0 system by calling rclcpp::init and passing in the command line arguments. This is necessary before creating any ROS nodes.
// The program creates a shared pointer to a new ROS node using the Node::make_shared function. The node is given the name "logger_node".
// A loop is started that runs as long as the ROS system is OK (rclcpp::ok() returns true). Within the loop, a message is published to the node's logger using the RCLCPP_INFO macro. The message contains a counter value that is incremented with each iteration of the loop.
// After publishing the message, the program calls rclcpp::spin_some to process any incoming messages. This function will return once there are no messages left to handle.
// The program then sleeps for a period of time determined by the loop_rate object, which is set to 250ms. This rate is used to control the speed of the loop so that it doesn't run too fast.
// Finally, when the loop is finished, the program calls rclcpp::shutdown to release any resources used by the ROS 2.0 system and returns 0.
// Overall, this program demonstrates the basic structure of a ROS 2.0 node and how to publish messages to its logger.


// Include the rclcpp header file, which contains the necessary classes and functions to work with ROS 2.0.
#include "rclcpp/rclcpp.hpp"

// Use the std namespace and define a 250ms duration literal for later use.
using namespace std::chrono_literals;

// Define the main function.
int main(int argc, char * argv[])
{
  // Initialize the ROS 2.0 system by calling rclcpp::init and passing in the command line arguments.
  rclcpp::init(argc, argv); 

  // Create a shared pointer to a new ROS node using the Node::make_shared function. The node is given the name "logger_node".
  auto node = rclcpp::Node::make_shared("logger_node");

  // Create an rclcpp::Rate object to control the speed of the loop. Set it to 250ms.
  rclcpp::Rate loop_rate(250ms);

  // Initialize a counter variable to 0.
  int counter = 0;

  // Enter a loop that runs as long as the ROS system is OK.
  while (rclcpp::ok()) {

    // Publish a message to the node's logger using the RCLCPP_INFO macro. The message contains a counter value that is incremented with each iteration of the loop.
    RCLCPP_INFO(node->get_logger(), "Hello %d", counter++);

    // Call rclcpp::spin_some to process any incoming messages. This function will return once there are no messages left to handle.
    rclcpp::spin_some(node);

    // Sleep for a period of time determined by the loop_rate object.
    loop_rate.sleep();
  }

  // Call rclcpp::shutdown to release any resources used by the ROS 2.0 system and return 0.
  rclcpp::shutdown();
  return 0;
}
