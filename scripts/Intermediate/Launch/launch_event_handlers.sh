#! /bin/bash

cd ~/Smart-Mobility-Engineering-Lab/launch_ws/src/
nano launch_tutorial/launch/example_event_handlers.launch.py
cd ..
colcon build
gnome-terminal -- bash -c "cd ~/Smart-Mobility-Engineering-Lab/launch_ws/; source install/local_setup.bash; ros2 launch launch_tutorial example_event_handlers.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200; bash exec"
