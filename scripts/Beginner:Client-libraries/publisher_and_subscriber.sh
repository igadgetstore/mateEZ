#! /bin/bash

#Background
#In this tutorial, you will create nodes that pass information in the form of string messages to each other over a topic. The example used here is a simple “talker” and “listener” system; one node publishes data and the other subscribes to the topic so it can receive that data.

#The code used in these examples can be found at https://github.com/ros2/examples/tree/humble/rclpy/topics.


#1 Create a package
#Open a new terminal and source your ROS 2 installation so that ros2 commands will work.

#Navigate into the ros2_ws directory created in a previous tutorial.
cd ~/Smart-Mobility-Engineering-Lab/ros2_ws

cd src
ros2 pkg create --build-type ament_python py_pubsub #Recall that packages should be created in the src directory, not the root of the workspace. So, navigate into ros2_ws/src, and run the package creation command:


#2 Write the publisher node
cd py_pubsub/py_pubsub/ #Navigate into ros2_ws/src/py_pubsub/py_pubsub. Recall that this directory is a Python package with the same name as the ROS 2 package it’s nested in.
wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py #Download the example talker code by entering the following command

#Now there will be a new file named publisher_member_function.py adjacent to __init__.py.
ls
gedit publisher_member_function.py #Open the file using your preferred text editor.
cd ..


#2.2 Add dependencies
#Navigate one level back to the ros2_ws/src/py_pubsub directory, where the setup.py, setup.cfg, and package.xml files have been created for you.
nano package.xml #Open package.xml with your text editor.
#As mentioned in the previous tutorial, make sure to fill in the <description>, <maintainer> and <license> tags:
#<description>Examples of minimal publisher/subscriber using rclpy</description>
#<maintainer email="you@email.com">Your Name</maintainer>
#<license>Apache License 2.0</license>
#After the lines above, add the following dependencies corresponding to your node’s import statements:
#<exec_depend>rclpy</exec_depend>
#<exec_depend>std_msgs</exec_depend>
#This declares the package needs rclpy and std_msgs when its code is executed.
#Make sure to save the file.

#2.3 Add an entry point
nano setup.py #Open the setup.py file. Again, match the maintainer, maintainer_email, description and license fields to your package.xml:
#Add the following line within the console_scripts brackets of the entry_points field:
#entry_points={
#        'console_scripts': [
#                'talker = py_pubsub.publisher_member_function:main',
#        ],
#},
#Don’t forget to save.


#2.4 Check setup.cfg
nano setup.cfg  #The contents of the setup.cfg file should be correctly populated automatically


#3 Write the subscriber node
cd py_pubsub/ #Return to ros2_ws/src/py_pubsub/py_pubsub to create the next node. 
wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py #Enter the following code in your terminal

ls #Now the directory should have these files:

gedit publisher_member_function.py #Open the subscriber_member_function.py with your text editor.


#3.2 Add an entry point
cd .. 
nano setup.py #Reopen setup.py and add the entry point for the subscriber node below the publisher’s entry point. 
#The entry_points field should now look like this:
#entry_points={
#        'console_scripts': [
#                'talker = py_pubsub.publisher_member_function:main',
#                'listener = py_pubsub.subscriber_member_function:main',
#        ],
#},


#4 Build and run
#You likely already have the rclpy and std_msgs packages installed as part of your ROS 2 system. 
cd ~/Smart-Mobility-Engineering-Lab/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y #It’s good practice to run rosdep in the root of your workspace (ros2_ws) to check for missing dependencies before building

sudo apt install python3-pip
pip install setuptools==58.2.0

colcon build --packages-select py_pubsub #Still in the root of your workspace, ros2_ws, build your new package:

gnome-terminal -- bash -c "cd ~/Smart-Mobility-Engineering-Lab/ros2_ws; source install/local_setup.bash; ros2 run py_pubsub talker; exec bash" #Open a new terminal, navigate to ros2_ws, and source the setup files; Now run the talker node
gnome-terminal -- bash -c "cd ~/Smart-Mobility-Engineering-Lab/ros2_ws; source install/local_setup.bash; ros2 run py_pubsub listener; exec bash" #Open another terminal, source the setup files from inside ros2_ws again, and then start the listener node
