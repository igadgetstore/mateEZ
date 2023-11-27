#! /bin/bash

#Background
#You learned about actions previously in the Understanding actions tutorial. Like the other communication types and their respective interfaces (topics/msg and services/srv), you can also custom-define actions in your packages. This tutorial shows you how to define and build an action that you can use with the action server and action client you will write in the next tutorial.

#1 Defining an action
#An instance of an action is typically referred to as a goal.
#Say we want to define a new action “Fibonacci” for computing the Fibonacci sequence.
cd ~/Smart-Mobility-Engineering-Lab/ros2_ws/src
ros2 pkg create action_tutorials_interfaces
#Create an action directory in our ROS 2 package action_tutorials_interfaces:
cd action_tutorials_interfaces
mkdir action
#Within the action directory, create a file called Fibonacci.action with the following contents:
printf "int32 order\n---\nint32[] sequence\n---\nint32[] partial_sequence" >> action/Fibonacci.action

#2 Building an action
#Before we can use the new Fibonacci action type in our code, we must pass the definition to the rosidl code generation pipeline.
#This is accomplished by adding the following lines to our CMakeLists.txt before the ament_package() line, in the action_tutorials_interfaces:
nano CMakeLists.txt 
#find_package(rosidl_default_generators REQUIRED)
#
#rosidl_generate_interfaces(${PROJECT_NAME}
#  "action/Fibonacci.action"
#)

#We should also add the required dependencies to our package.xml:
nano package.xml 
#<buildtool_depend>rosidl_default_generators</buildtool_depend>
#
#<depend>action_msgs</depend>
#
#<member_of_group>rosidl_interface_packages</member_of_group>

#We should now be able to build the package containing the Fibonacci action definition:
cd ~/Smart-Mobility-Engineering-Lab/ros2_ws/ # Change to the root of the workspace
colcon build # Build

#We can check that our action built successfully with the command line tool:
# Source our workspace
# On Windows: call install/setup.bat
. install/setup.bash
# Check that our action definition exists
ros2 interface show action_tutorials_interfaces/action/Fibonacci
