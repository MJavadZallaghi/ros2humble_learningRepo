// This code demonstrates a simple ROS (Robot Operating System) publisher node written in C++. The code includes the necessary header files and namespaces, and defines a class called `PublisherNode` that inherits from `rclcpp::Node`, which represents a ROS node.
// The constructor of `PublisherNode` creates a publisher that publishes messages of type `std_msgs::msg::Int32` to a topic named "int_topic" with a queue size of 10. It also creates a timer that calls the `timer_callback` function every 500 milliseconds.
// The `timer_callback` function increments the data value of the message and publishes it using the publisher. In the `main` function, the ROS node is initialized, an instance of `PublisherNode` is created, and the node is spun to execute its callbacks.
// Finally, the ROS system is shutdown, and the program terminates.

#include "rclcpp/rclcpp.hpp" // Include the ROS C++ library

#include "std_msgs/msg/int32.hpp" // Include the message type that will be published

using namespace std::chrono_literals; // Namespace for time literals like 500ms
using std::placeholders::_1; // Placeholder for callback functions

class PublisherNode : public rclcpp::Node // Define a class that inherits from rclcpp::Node
{
public:
  PublisherNode() // Constructor for the class
  : Node("publisher_node") // Initialize the base Node class with the node name "publisher_node"
  {
    publisher_ = create_publisher<std_msgs::msg::Int32>("int_topic", 10); // Create a publisher that publishes std_msgs::msg::Int32 messages to the topic "int_topic" with a queue size of 10
    timer_ = create_wall_timer(500ms, std::bind(&PublisherNode::timer_callback, this)); // Create a timer that calls the timer_callback() function every 500 milliseconds
  }

  void timer_callback() // Timer callback function
  {
    message_.data += 1; // Increment the data value of the message
    publisher_->publish(message_); // Publish the message using the publisher
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_; // Publisher object for publishing messages
  rclcpp::TimerBase::SharedPtr timer_; // Timer object for timer callbacks
  std_msgs::msg::Int32 message_; // Message object of type std_msgs::msg::Int32
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); // Initialize the ROS node

  auto node = std::make_shared<PublisherNode>(); // Create an instance of the PublisherNode class

  rclcpp::spin(node); // Spin the node, allowing it to execute its callbacks

  rclcpp::shutdown(); // Shutdown the ROS node

  return 0; // Return 0 to indicate successful execution
}
