#! /bin/bash

#Background
#Turtlesim is a lightweight simulator for learning ROS 2. It illustrates what ROS 2 does at the most basic level to give you an idea of what you will do with a real robot or a robot simulation later on.
#The ros2 tool is how the user manages, introspects, and interacts with a ROS system. It supports multiple commands that target different aspects of the system and its operation. One might use it to start a node, set a parameter, listen to a topic, and many more. The ros2 tool is part of the core ROS 2 installation.
#rqt is a graphical user interface (GUI) tool for ROS 2. Everything done in rqt can be done on the command line, but rqt provides a more user-friendly way to manipulate ROS 2 elements.
#This tutorial touches upon core ROS 2 concepts, like nodes, topics, and services. All of these concepts will be elaborated on in later tutorials; for now, you will simply set up the tools and get a feel for them.

#1 Install turtlesim
sudo apt update

sudo apt install ros-humble-turtlesim #Install the turtlesim package for your ROS 2 distro:

ros2 pkg executables turtlesim #Check that the package is installed

#2 Start turtlesim
ros2 run turtlesim turtlesim_node #To start turtlesim, enter the following command in your terminal

#3 Use turtlesim
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; ros2 run turtlesim turtle_teleop_key; exec bash" #In one terminal, source the setup file and then run a new node to control the turtle in the first node 

#You can see the nodes, and their associated topics, services, and actions, using the list subcommands of the respective commands:
ros2 node list
ros2 topic list
ros2 service list
ros2 action list

#4 install rqt and its plugins
gnome-terminal -- bash -c "sudo apt update; sudo apt install ~nros-humble-rqt*; exec bash" #Open a new terminal to install rqt and its plugins:

rqt #To run rqt

#6 Remapping
#You need a second teleop node in order to control turtle2. However, if you try to run the same command as before, you will notice that this one also controls turtle1. The way to change this behavior is by remapping the cmd_vel topic.
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel; exec bash" #In a new terminal, source ROS 2, and run:

#7 Close turtlesim
#To stop the simulation, you can enter Ctrl + C in the turtlesim_node terminal, and q in the turtle_teleop_key terminals.
