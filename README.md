# ros2humble_learningRepo
To save and recall the commands, programs, and outputs from ROS2 humble learning. 

## Important node
* Always source two directory:
1.  The main ROS packages
```bash
source /opt/ros/humble/setup.bash 
```
2. Cusstom built packages
```bash
source /home/mjavadzallaghi/bookros2/ros2humble_learningRepo/install/setup.bash 
```

## Explanation of the codes in the packages
```bash
my_packege/src/simple.cpp
```
```cpp
// This is a simple C++ program using the Robot Operating System (ROS) 2.0 library to create a ROS node.
// The program starts by including the necessary header files for ROS 2.0 and logging macros. The main function initializes the ROS 2.0 context using rclcpp::init with the provided command line arguments argc and argv.
// It then creates a shared pointer to a rclcpp::Node object named "simple_node". A node is a process that performs computation in the ROS 2.0 graph. The make_shared function creates a new instance of a node, which is managed by a shared pointer.
// After creating the node, the program enters the event loop by calling rclcpp::spin. This function blocks the program until the node is shut down. While in the event loop, the node will listen for incoming messages and handle any callbacks associated with them.
// Finally, the program shuts down the ROS 2.0 context by calling rclcpp::shutdown.
```
```bash
br2_basics/src/logger.cpp
```
```cpp
// This is a simple C++ program that creates a ROS node using the ROS 2.0 library. The program uses the rclcpp (ROS Client Library for C++) package to initialize the ROS 2.0 system, create a node, and publish messages. The program uses the RCLCPP_INFO macro to publish a message containing a counter value to the node's logger.
// The program starts by including the rclcpp header file, which contains the necessary classes and functions to work with ROS 2.0.
// The main function initializes the ROS 2.0 system by calling rclcpp::init and passing in the command line arguments. This is necessary before creating any ROS nodes.
// The program creates a shared pointer to a new ROS node using the Node::make_shared function. The node is given the name "logger_node".
// A loop is started that runs as long as the ROS system is OK (rclcpp::ok() returns true). Within the loop, a message is published to the node's logger using the RCLCPP_INFO macro. The message contains a counter value that is incremented with each iteration of the loop.
// After publishing the message, the program calls rclcpp::spin_some to process any incoming messages. This function will return once there are no messages left to handle.
// The program then sleeps for a period of time determined by the loop_rate object, which is set to 250ms. This rate is used to control the speed of the loop so that it doesn't run too fast.
// Finally, when the loop is finished, the program calls rclcpp::shutdown to release any resources used by the ROS 2.0 system and returns 0.
// Overall, this program demonstrates the basic structure of a ROS 2.0 node and how to publish messages to its logger.

```

## Important ROS2 commands
A list to save the used commands during the learning process for later simple recalling.

* ROS2 standard cmd struture
```bash
ros2 <command> <verb> [<params>|<option>]
```

* Get the list of all available ros2 packages
```bash
ros2 pkg list
```

* Get the list of all executables of a specific pkg; e.g. name of a executable node
```bash
ros2 pkg executables <pkg-name>
```
    
* To run an executable in a pkg
```bash
ros2 run <pkg-name> <executable>
```
    
* Get the list of all active (running) nodes
```bash
ros2 node list
```
    
* Get the list of all active (running) topics
```bash
ros2 topic list
```
    
* To obtain the information of an active (running) node
```bash
ros2 node info /<node_name>
```
    
* To obtain the information of an active (running) topic
```bash
ros2 topic info /<topic_name>
```
    
* Get the list of available interfaces (pre-defined only?) including standard message types, services, and actions
```bash
ros2 interface list
```
    
* To obtain structure of an interface, defined in a pkg.
```bash
ros2 interface show <pkg_name/interfacec/interfaceI>
```
* Example 1
 ```bash
ros2 interface show std_msgs/msg/String
```   

* Example 2
 ```bash
ros2 interface show rcl_interfaces/msg/Log
``` 

* To echo messages bublished by a topic
```bash
ros2 topic echo /<topic_name>
```

* To echo messages bublished by /rosout topic
e.g. logger.cpp in br2_basics package
```bash
ros2 topic echo /rosout
```
    
* To draw computational graph of the current running ros2 architecture (GUI)
```bash
ros2 run rqt_graph rqt_graph
```

* To check the nodes and messages publsihed to /rosout topic (GUI)
```bash
ros2 run rqt_console rqt_console
```
    
* To create new packages with dependencies
```bash
ros2 pkg create my_package --dependencies rclcpp std_msgs 
```

* To compile all the packages in the src directory
```bash
colcon build --symlink-install
```

* To compile only one spiciific package in the src directory
```bash
colcon build --symlink-install --packages-select br2_basics
```
    



## To do list
1. Learn how to run a node (with a simple task) in a constant rate (frequency)
2. Learn how to run a node (task) iteratively.
