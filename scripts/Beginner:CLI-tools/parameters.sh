#! /bin/bash

#Start up the two turtlesim nodes, /turtlesim and /teleop_turtle
gnome-terminal -- ros2 run turtlesim turtlesim_node
gnome-terminal -- ros2 ros2 run turtlesim turtlesim_node

#To see the parameters belonging to your nodes, open a new terminal and enter the command:
ros2 param list
#Every node has the parameter use_sim_time; it’s not unique to turtlesim. Based on their names, it looks like /turtlesim’s parameters determine the background color of the turtlesim window using RGB color values.

#To determine a parameter’s type, you can use ros2 param get. Let’s find out the current value of /turtlesim’s parameter background_g:
ros2 param get /turtlesim background_g
#Now you know background_g holds an integer value.

#If you run the same command on background_r and background_b, you will get the values 69 and 255, respectively. Let’s change /turtlesim’s background color:
ros2 param set /turtlesim background_r 150
#Setting parameters with the set command will only change them in your current session, not permanently. 

#However, you can save your settings and reload them the next time you start a node.
#The command prints to the standard output (stdout) by default but you can also redirect the parameter values into a file to save them for later. To save your current configuration of /turtlesim’s parameters into the file turtlesim.yaml, enter the command:
ros2 param dump /turtlesim > turtlesim.yaml

#You can load parameters from a file to a currently running node. 
#To load the turtlesim.yaml file generated with ros2 param dump into /turtlesim node’s parameters, enter the command:
ros2 param load /turtlesim turtlesim.yaml

#To start the same node using your saved parameter values, use: ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
#This is the same command you always use to start turtlesim, with the added flags --ros-args and --params-file, followed by the file you want to load.
#Stop your running turtlesim node, and try reloading it with your saved parameters, using:
ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim.yaml
