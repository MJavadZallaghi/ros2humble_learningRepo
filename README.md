# ros2humble_learningRepo
To save and recall the commands, programs, and outputs from ROS2 humble learning. 

## Important notes
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
```bash
br2_basics/src/logger_class.cpp
```
```cpp
// This is a C++ program that creates a ROS2 node called "logger_node". The node is implemented as a class called "LoggerNode" which inherits from rclcpp::Node.
// It has a member variable "timer_" which is a shared pointer to a TimerBase object. The timer is created in the constructor of the LoggerNode class with a period of 500 milliseconds and a callback function called "timer_callback".
// The timer_callback function is called every time the timer expires and it prints a message to the console using the RCLCPP_INFO macro.
// The main function initializes the ROS2 runtime, creates an instance of the LoggerNode class and spins it, which means that it runs until the node is shut down. Finally, the ROS2 runtime is shut down and the program returns 0.
```
```bash
br2_basics/src/publisher_class.cpp
```
```cpp
// This code demonstrates a simple ROS (Robot Operating System) publisher node written in C++. The code includes the necessary header files and namespaces, and defines a class called `PublisherNode` that inherits from `rclcpp::Node`, which represents a ROS node.
// The constructor of `PublisherNode` creates a publisher that publishes messages of type `std_msgs::msg::Int32` to a topic named "int_topic" with a queue size of 10. It also creates a timer that calls the `timer_callback` function every 500 milliseconds.
// The `timer_callback` function increments the data value of the message and publishes it using the publisher. In the `main` function, the ROS node is initialized, an instance of `PublisherNode` is created, and the node is spun to execute its callbacks.
// Finally, the ROS system is shutdown, and the program terminates.
```
```bash
br2_basics/src/publisher.cpp
```
```cpp
// This code is a simple example of a ROS (Robot Operating System) publisher node written in C++. It includes the necessary header files and uses the "rclcpp" library for ROS 2 to handle communication. 
// The code initializes the ROS 2 node and creates a publisher that publishes messages of type "std_msgs::msg::Int32" to a topic named "int_topic". The publisher is set to have a queue size of 10, meaning it can buffer up to 10 messages before they are dropped.
// Inside the main loop, a message of type "std_msgs::msg::Int32" is created and its data field is set to 0. Then, the message is published using the publisher. The data field of the message is incremented by 1 in each iteration, and the updated message is published again.
// The loop runs as long as the ROS 2 system is in a valid state (rclcpp::ok()). Within each iteration of the loop, the function rclcpp::spin_some() is called to process any pending callbacks and the loop is paused for 500 milliseconds using the loop_rate object.
// After the loop ends, the ROS 2 system is shut down by calling rclcpp::shutdown(). Finally, the program returns 0 to indicate successful execution.
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
1. <del>Create a Node in cpp and spin it.</del>
2. <del>Run a node (with a simple task) in a constant rate (frequency). </del>
3. <del>Run a node (task) iteratively. </del>
4. <del> Create a publisher node with class structure. </del>
5. <del> Create a publisher node with a timer to control publishing frequency. </del>