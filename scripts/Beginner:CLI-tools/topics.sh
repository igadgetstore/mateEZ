#! /bin/bash

#Open a new terminal and run:
gnome-terminal -- ros2 run turtlesim turtlesim_node

#Open another terminal and run:
gnome-terminal -- ros2 run turtlesim turtle_teleop_key

#Throughout this tutorial, we will use rqt_graph to visualize the changing nodes and topics, as well as the connections between them.
rqt_graph &

#Running the ros2 topic list command in a new terminal will return a list of all the topics currently active in the system:
gnome-terminal -- ros2 topic echo /turtle1/cmd_vel

#Since we know that /teleop_turtle publishes data to /turtlesim over the /turtle1/cmd_vel topic, let’s use echo to introspect that topic:
ros2 topic list 
ros2 topic list -t

#Topics don’t have to only be one-to-one communication; they can be one-to-many, many-to-one, or many-to-many.
#Another way to look at this is running:
ros2 topic info /turtle1/cmd_vel

#This means that in the package geometry_msgs there is a msg called Twist.
#Now we can run ros2 interface show <msg type> on this type to learn its details. Specifically, what structure of data the message expects.
ros2 interface show geometry_msgs/msg/Twist

#Now that you have the message structure, you can publish data onto a topic directly from the command line using:
#It’s important to note that this argument needs to be input in YAML syntax. Input the full command like so:
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

#The turtle (and commonly the real robots which it is meant to emulate) require a steady stream of commands to operate continuously. So, to get the turtle to keep moving, you can run:
gnome-terminal -- ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}" &
#The difference here is the removal of the --once option and the addition of the --rate 1 option, which tells ros2 topic pub to publish the command in a steady stream at 1 Hz.

#You can refresh rqt_graph to see what’s happening graphically. You will see that the ros2 topic pub ... node (/_ros2cli_30358) is publishing over the /turtle1/cmd_vel topic, which is being received by both the ros2 topic echo ... node (/_ros2cli_26646) and the /turtlesim node now.
#Finally, you can run echo on the pose topic and recheck rqt_graph:
gnome-terminal -- ros2 topic echo /turtle1/pose

#For one last introspection on this process, you can view the rate at which data is published using:
ros2 topic hz /turtle1/pose
