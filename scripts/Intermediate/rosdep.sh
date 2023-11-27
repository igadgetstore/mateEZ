#! /bin/bash
#Must read https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html


#rosdep installation
#If you are using rosdep with ROS, it is conveniently packaged along with the ROS distribution. This is the recommended way to get rosdep. You can install it with:
cd ~/Smart-Mobility-Engineering-Lab/ros2_ws
sudo apt-get install python3-rosdep

#rosdep operation
#Now that we have some understanding of rosdep, package.xml, and rosdistro, we’re ready to use the utility itself! Firstly, if this is the first time using rosdep, it must be initialized via:
sudo rosdep init
rosdep update

#Finally, we can run rosdep install to install dependencies. Typically, this is run over a workspace with many packages in a single call to install all dependencies. A call for that would appear as the following, if in the root of the workspace with directory src containing source code.
rosdep install --from-paths src -y --ignore-src
