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
