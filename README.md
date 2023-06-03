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

* To launch a [Python] launch file (a group of node names. etc to run together)
```bash
ros2 launch [package-name] [launch-file-name].py
```
    



## To do list
1. <del>Create a Node in cpp and spin it.</del>
2. <del>Run a node (with a simple task) in a constant rate (frequency). </del>
3. <del>Run a node (task) iteratively. </del>
4. <del> Create a publisher node with class structure. </del>
5. <del> Create a publisher node with a timer to control publishing frequency. </del>
6. <del> Create a subscriber node which manipulates data. [subscriber_manipulator node created as my first manipulator data node.] </del>
7. <del> Create a node which is a subcriber to topic X and publishes to topic Y [node_xx_subs_yy_pub mode is created to act as simultanous subscriber and publisher.] <</del>>.
8. Create a custom node with parameters.
9. Create an execcutor (two nodes: subscriber and publisher together execution).