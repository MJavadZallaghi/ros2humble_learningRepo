// This code is a simple example of a ROS (Robot Operating System) publisher node written in C++. It includes the necessary header files and uses the "rclcpp" library for ROS 2 to handle communication. 
// The code initializes the ROS 2 node and creates a publisher that publishes messages of type "std_msgs::msg::Int32" to a topic named "int_topic". The publisher is set to have a queue size of 10, meaning it can buffer up to 10 messages before they are dropped.
// Inside the main loop, a message of type "std_msgs::msg::Int32" is created and its data field is set to 0. Then, the message is published using the publisher. The data field of the message is incremented by 1 in each iteration, and the updated message is published again.
// The loop runs as long as the ROS 2 system is in a valid state (rclcpp::ok()). Within each iteration of the loop, the function rclcpp::spin_some() is called to process any pending callbacks and the loop is paused for 500 milliseconds using the loop_rate object.
// After the loop ends, the ROS 2 system is shut down by calling rclcpp::shutdown(). Finally, the program returns 0 to indicate successful execution.


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); // Initialize the ROS 2 node

  auto node = rclcpp::Node::make_shared("publisher_node"); // Create a node
  auto publisher = node->create_publisher<std_msgs::msg::Int32>("int_topic", 10); // Create a publisher that publishes messages of type std_msgs::msg::Int32 on the "int_topic" topic with a queue size of 10

  std_msgs::msg::Int32 message; // Create a message of type std_msgs::msg::Int32
  message.data = 0; // Set the data field of the message to 0

  rclcpp::Rate loop_rate(500ms); // Create a rate object that controls the loop frequency to 2 Hz (500ms period)
  while (rclcpp::ok()) { // Continue the loop as long as ROS 2 is running
    message.data += 1; // Increment the data field of the message by 1
    publisher->publish(message); // Publish the message on the "int_topic" topic

    rclcpp::spin_some(node); // Process any callbacks for this node
    loop_rate.sleep(); // Sleep to maintain the loop rate
  }

  rclcpp::shutdown(); // Shutdown the ROS 2 node
  return 0;
}
