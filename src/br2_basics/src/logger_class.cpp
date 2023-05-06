// This is a C++ program that creates a ROS2 node called "logger_node". The node is implemented as a class called "LoggerNode" which inherits from rclcpp::Node.
// It has a member variable "timer_" which is a shared pointer to a TimerBase object. The timer is created in the constructor of the LoggerNode class with a period of 500 milliseconds and a callback function called "timer_callback".
// The timer_callback function is called every time the timer expires and it prints a message to the console using the RCLCPP_INFO macro.
// The main function initializes the ROS2 runtime, creates an instance of the LoggerNode class and spins it, which means that it runs until the node is shut down. Finally, the ROS2 runtime is shut down and the program returns 0.

// Include the rclcpp library, which provides a C++ interface to ROS2
#include "rclcpp/rclcpp.hpp"

// Define a namespace for convenience
using namespace std::chrono_literals;

// Define a class called LoggerNode, which inherits from rclcpp::Node
class LoggerNode : public rclcpp::Node
{
public:
  // Define the constructor for LoggerNode
  LoggerNode(): Node("logger_node")
  {
    // Initialize the counter variable to zero
    counter_ = 0;
    // Create a timer with a 500 millisecond period that calls the timer_callback function
    // The std::bind function is used to bind the this pointer to the function, so that
    // the callback can access the member variables of the LoggerNode object
    timer_ = create_wall_timer(500ms, std::bind(&LoggerNode::custom_func_execute, this));
  }

  // Define the timer_callback function
  void timer_callback()
  {
    // Print a message to the console using the RCLCPP_INFO macro
    // The message includes the value of the counter variable
    RCLCPP_INFO(get_logger(), "Hello %d", counter_++);
  }


  // Define a custom function to execute iteratively by the node
  void custom_func_execute()
  {
    // Print a message to the console using the RCLCPP_INFO macro
    // The message includes the value of the counter variable
    std::cout<<"This is a cusstom fixed-rate executed function by Mohammad Javad Zallaghi\n";
    std::cout<<counter_++<<std::endl;
  }

private:
  // Declare a shared pointer to a TimerBase object
  rclcpp::TimerBase::SharedPtr timer_;
  // Declare an integer counter variable
  int counter_;
};

// Define the main function
int main(int argc, char * argv[])
{
  // Initialize the ROS2 runtime
  rclcpp::init(argc, argv);

  // Create an instance of the LoggerNode class using a shared pointer
  auto node = std::make_shared<LoggerNode>();

  // Spin the node, which means that it will run until it is shut down
  rclcpp::spin(node);

  // Shut down the ROS2 runtime
  rclcpp::shutdown();

  // Return zero to indicate successful completion of the program
  return 0;
}
