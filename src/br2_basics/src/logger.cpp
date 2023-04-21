// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


// rclcpp:Rate, node->get_logger(), and spin_some(node) are used.

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); 

  auto node = rclcpp::Node::make_shared("logger_node");

  rclcpp::Rate loop_rate(250ms);  // rclcpp:Rate for speed control of the loop.
  int counter = 0;
  while (rclcpp::ok()) {
    RCLCPP_INFO(node->get_logger(), "Hello %d", counter++); // node->get_logger() returns basic node info

    rclcpp::spin_some(node);  // While spin(node) blocks waiting for new messages, spin_some(node)
                              // returns once there are no messages left to handle. 
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}


// This is a simple C++ program that creates a ROS node using the ROS 2.0 library. The program uses the rclcpp (ROS Client Library for C++) package to initialize the ROS 2.0 system, create a node, and publish messages. The program uses the RCLCPP_INFO macro to publish a message containing a counter value to the node's logger.

// Here's a breakdown of the program:

// The program starts by including the rclcpp header file, which contains the necessary classes and functions to work with ROS 2.0.
// The main function initializes the ROS 2.0 system by calling rclcpp::init and passing in the command line arguments. This is necessary before creating any ROS nodes.
// The program creates a shared pointer to a new ROS node using the Node::make_shared function. The node is given the name "logger_node".
// A loop is started that runs as long as the ROS system is OK (rclcpp::ok() returns true). Within the loop, a message is published to the node's logger using the RCLCPP_INFO macro. The message contains a counter value that is incremented with each iteration of the loop.
// After publishing the message, the program calls rclcpp::spin_some to process any incoming messages. This function will return once there are no messages left to handle.
// The program then sleeps for a period of time determined by the loop_rate object, which is set to 250ms. This rate is used to control the speed of the loop so that it doesn't run too fast.
// Finally, when the loop is finished, the program calls rclcpp::shutdown to release any resources used by the ROS 2.0 system and returns 0.
// Overall, this program demonstrates the basic structure of a ROS 2.0 node and how to publish messages to its logger.
