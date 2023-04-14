# ros2humble_learningRepo
To save and recall the commands, programs, and outputs from ROS2 humble learning. 
## Important ROS2 commands
A list to save the used commands during the learning process for later simple recalling.

* $ ros2 <command> <verb> [<params>|<option>]*
    * ROS2 standard cmd struture

* $ ros2 pkg list
    * Get the list of all available ros2 packages

* $ ros2 pkg executables <pkg-name>
    * Get the list of all executables of a specific pkg; e.g. name of a executable node

* $ ros2 run <pkg-name> <executable>
    * To run an executable in a pkg

* $ ros2 node list
    * Get the list of all active (running) nodes

* $ ros2 topic list
    * Get the list of all active (running) topics

* $ ros2 node info /<node_name>
    * To obtain the information of an active (running) node

* $ ros2 topic info /<topic_name>
    * To obtain the information of an active (running) topic

* $ ros2 interface list
    * Get the list of available interfaces (pre-defined only?) including standard message types, services, and actions

* $ ros2 interface show <pkg_name/interfacec/interfaceI>
    * To obtain structure of an interface, defined in a pkg.
    e.g. std_msgs/msg/String

* $ ros2 topic echo /<topic_name>
    * To echo messages bublished by a topic

* $ ros2 run rqt graph rqt graph
    * To draw computational graph of the current running ros2 architecture (GUI)

* $ ros2 pkg create my_package --dependencies rclcpp std_msgs 
    * To create new packages with dependencies



## To do list
1. Create a UI for the published command.
2. Learn node developing and share the commands/crated node in the repo.
