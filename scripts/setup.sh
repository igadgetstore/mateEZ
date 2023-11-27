#!/bin/bash

#Set locale
#Make sure you have a locale which supports UTF-8. If you are in a minimal environment (such as a docker container), the locale may be something minimal like POSIX. We test with the following settings. However, it should be fine if you’re using a different UTF-8 supported locale.

locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings


#Setup Sources
sudo apt install software-properties-common #You will need to add the ROS 2 apt repository to your system.
sudo add-apt-repository universe #First ensure that the Ubuntu Universe repository is enabled.

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg #Now add the ROS 2 GPG key with apt.

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null #Then add the repository to your sources list.


#Install ROS 2 packages
sudo apt update #Update your apt repository caches after setting up the repositories.

sudo apt upgrade #ROS 2 packages are built on frequently updated Ubuntu systems. It is always recommended that you ensure your system is up to date before installing new packages.

sudo apt install ros-humble-desktop #Desktop Install (Recommended): ROS, RViz, demos, tutorials.

sudo apt install ros-humble-ros-base #ROS-Base Install (Bare Bones): Communication libraries, message packages, command line tools. No GUI tools.

sudo apt install ros-dev-tools #Development tools: Compilers and other tools to build ROS packages


#Environment setup
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash #Sourcing the setup script


#Try some examples
#Talker-listener
#If you installed ros-humble-desktop above you can try some examples.
source /opt/ros/humble/setup.bash
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; ros2 run demo_nodes_cpp talker; exec bash" #In one terminal, source the setup file and then run a C++ talker 
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; ros2 run demo_nodes_py listener; exec bash" #In another terminal source the setup file and then run a Python listener:

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
printenv | grep -i ROS
