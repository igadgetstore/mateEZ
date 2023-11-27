#! /bin/bash

cd ~/Smart-Mobility-Engineering-Lab/ros2_ws
mkdir launch
nano launch/turtlesim_mimic_launch.py
cd launch
ros2 launch turtlesim_mimic_launch.py
gnome-terminal -- bash -c "ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"; exec bash"
gnome-terminal -- rqt_graph
