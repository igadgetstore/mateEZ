#! /bin/bash

#Background
#A workspace is a directory containing ROS 2 packages. Before using ROS 2, it’s necessary to source your ROS 2 installation workspace in the terminal you plan to work in. This makes ROS 2’s packages available for you to use in that terminal.

#You also have the option of sourcing an “overlay” - a secondary workspace where you can add new packages without interfering with the existing ROS 2 workspace that you’re extending, or “underlay”. Your underlay must contain the dependencies of all the packages in your overlay. Packages in your overlay will override packages in the underlay. It’s also possible to have several layers of underlays and overlays, with each successive overlay using the packages of its parent underlays.

#1 Source ROS 2 environment
#Your main ROS 2 installation will be your underlay for this tutorial. (Keep in mind that an underlay does not necessarily have to be the main ROS 2 installation.)
source /opt/ros/humble/setup.bash #Depending on how you installed ROS 2 (from source or binaries), and which platform you’re on, your exact source command will vary


#2 Create a new directory
mkdir -p ~/Smart-Mobility-Engineering-Lab/ros2_ws/src #Best practice is to create a new directory for every new workspace. The name doesn’t matter, but it is helpful to have it indicate the purpose of the workspace. Let’s choose the directory name ros2_ws, for “development workspace”:
cd ~/Smart-Mobility-Engineering-Lab/ros2_ws/src #Another best practice is to put any packages in your workspace into the src directory. The above code creates a src directory inside ros2_ws and then navigates into it.


#3 Clone a sample repo
#Ensure you’re still in the ros2_ws/src directory before you clone.

#In the rest of the beginner developer tutorials, you will create your own packages, but for now you will practice putting a workspace together using existing packages.

#If you went through the Beginner: CLI Tools tutorials, you’ll be familiar with turtlesim, one of the packages in ros_tutorials.

#A repo can have multiple branches. You need to check out the one that targets your installed ROS 2 distro. When you clone this repo, add the -b argument followed by that branch.
git clone https://github.com/ros/ros_tutorials.git -b humble #In the ros2_ws/src directory, run the following command


#4 Resolve dependencies
#Before building the workspace, you need to resolve the package dependencies. You may have all the dependencies already, but best practice is to check for dependencies every time you clone. You wouldn’t want a build to fail after a long wait only to realize that you have missing dependencies.
# cd if you're still in the ``src`` directory with the ``ros_tutorials`` clone
cd ..
rosdep install -i --from-path src --rosdistro humble -y #From the root of your workspace (ros2_ws), run the following command:


#5 Build the workspace with colcon
colcon build #From the root of your workspace (ros2_ws), you can now build your packages using the command

ls #Once the build is finished, enter the command in the workspace root (~/ros2_ws):

#6 & 7 Source the overlay & Modify the overlay
#You can modify turtlesim in your overlay by editing the title bar on the turtlesim window. To do this, locate the turtle_frame.cpp file in ~/ros2_ws/src/ros_tutorials/turtlesim/src. Open turtle_frame.cpp with your preferred text editor.

#On line 52 you will see the function setWindowTitle("TurtleSim");. Change the value "TurtleSim" to "MyTurtleSim", and save the file.

#Return to the first terminal where you ran colcon build earlier and run it again.

#Return to the second terminal (where the overlay is sourced) and run turtlesim again:
#Before sourcing the overlay, it is very important that you open a new terminal, separate from the one where you built the workspace. Sourcing an overlay in the same terminal where you built, or likewise building where an overlay is sourced, may create complex issues.

#You can modify turtlesim in your overlay by editing the title bar on the turtlesim window. To do this, locate the turtle_frame.cpp file in ~/ros2_ws/src/ros_tutorials/turtlesim/src. Open turtle_frame.cpp with your preferred text editor.

#On line 52 you will see the function setWindowTitle("TurtleSim");. Change the value "TurtleSim" to "MyTurtleSim", and save the file.

#Return to the first terminal where you ran colcon build earlier and run it again.
colcon build

#Return to the second terminal (where the overlay is sourced) and run turtlesim again:
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; cd ~/Smart-Mobility-Engineering-Lab/ros2_ws; source install/local_setup.bash; nanno ~/Smart-Mobility-Engineering-Lab/ros2_ws/src/ros_tutorials/turtlesim/src/turtle_frame.cpp; ros2 run turtlesim turtlesim_node; exec bash"#In the new terminal, source your main ROS 2 environment as the “underlay”, so you can build the overlay “on top of” it; Go into the root of your workspace; In the root, source your overlay; Now you can run the turtlesim package from the overlay
cd ~/Smart-Mobility-Engineering-Lab/ros2_ws/src
