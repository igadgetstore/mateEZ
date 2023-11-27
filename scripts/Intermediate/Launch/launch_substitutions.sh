#! /bin/bash

cd ~/Smart-Mobility-Engineering-Lab/launch_ws/src/
ros2 pkg create launch_tutorial --build-type ament_python
mkdir launch_tutorial/launch
nano launch_tutorial/setup.py 
nano launch_tutorial/launch/example_main.launch.py
nano launch_tutorial/launch/example_substitutions.launch.py
cd ..
colcon build
gnome-terminal -- bash -c "cd ~/Smart-Mobility-Engineering-Lab/launch_ws/; source install/local_setup.bash; ros2 launch launch_tutorial example_main.launch.py; ros2 launch launch_tutorial example_substitutions.launch.py --show-args; ros2 launch launch_tutorial example_substitutions.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200; bash exec"
