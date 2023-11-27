#! /bin/bash/

cd ~/Smart-Mobility-Engineering/

sudo apt-get install ros-humble-turtle-tf2-py ros-humble-tf2-tools ros-humble-tf-transformations
gnome-terminal -- bash -c "ros2 launch turtle_tf2_py turtle_tf2_demo.launch.py; bash exec"
gnome-terminal -- bash -c "ros2 run turtlesim turtle_teleop_key; bash exec"

ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo turtle2 turtle1
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix --share turtle_tf2_py)/rviz/turtle_rviz.rviz
