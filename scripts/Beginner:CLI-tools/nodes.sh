#! /bin/bash

#The command ros2 run launches an executable from a package.
gnome-terminal -- ros2 run turtlesim turtlesim_node

#ros2 node list will show you the names of all running nodes. This is especially useful when you want to interact with a node, or when you have a system running many nodes and need to keep track of them.
ros2 node list

#Here, we are referring to the turtlesim package again, but this time we target the executable named turtle_teleop_key.
gnome-terminal -- ros2 run turtlesim turtle_teleop_key

#Return to the terminal where you ran ros2 node list and run it again. You will now see the names of two active nodes:
ros2 node lsit

#Remapping allows you to reassign default node properties, like node name, topic names, service names, etc., to custom values. In the last tutorial, you used remapping on turtle_teleop_key to change the cmd_vel topic and target turtle2.
gnome-terminal -- ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle

#Since you’re calling ros2 run on turtlesim again, another turtlesim window will open. However, now if you return to the terminal where you ran ros2 node list, and run it again, you will see three node names:
ros2 node list

#ros2 node info returns a list of subscribers, publishers, services, and actions. i.e. the ROS graph connections that interact with that node. Now that you know the names of your nodes, you can access more information about them with:
ros2 node info /my_turtle

#Now try running the same command on the /teleop_turtle node, and see how its connections differ from my_turtle.
ros2 node info /teleop_turtle
