#! /bin/bash/

cd Smart-Mobility-Engineering-Lab/ros2_ws/src
git clone https://github.com/ros/urdf_tutorial
rosdep install --from-paths src -y --ignore-src
colcon build packages-select urdf_tutorial
gnome-terminal --bash -c "source install/setup.bash; ros2 launch urdf_tutorial display.launch.py model:=urdf/01-myfirst.urdf; ros2 launch urdf_tutorial display.launch.py model:=`ros2 pkg prefix --share urdf_tutorial`/urdf/01-myfirst.urdf; ros2 launch urdf_tutorial display.launch.py model:=urdf/02-multipleshapes.urdf; ros2 launch urdf_tutorial display.launch.py model:=urdf/03-origins.urdf; ros2 launch urdf_tutorial display.launch.py model:=urdf/04-materials.urdf; ros2 launch urdf_tutorial display.launch.py model:=urdf/05-visual.urdf; ros2 launch urdf_tutorial display.launch.py model:=urdf/06-flexible.urdf; exec bash"
