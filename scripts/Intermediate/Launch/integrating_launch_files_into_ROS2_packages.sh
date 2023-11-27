#! /bin/bash

cd ~/Smart-Mobility-Engineering-Lab
mkdir -p launch_ws/src
cd launch_ws/src
ros2 pkg create py_launch_example --build-type ament_python
mkdir py_launch_example/launch
nano py_launch_example/setup.py
nano py_launch_example/launch/my_script_launch.py
colcon build
gnome-terminal -- bash -c "cd ~/Smart-Mobility-Engineering-Lab/launch_ws/; source install/local_setup.bash; ros2 launch py_launch_example my_script_launch.py; bash exec"
