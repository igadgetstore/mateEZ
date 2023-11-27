#! /bin/bash

ros2 component types
ros2 run rclcpp_components component_container
gnome-terminal -- bash -c "ros2 component list; ros2 component load /ComponentManager composition composition::Talker; ros2 component load /ComponentManager composition composition::Listener; ros2 component list; exec bash"

ros2 run rclcpp_components component_container
gnome-terminal -- bash -c "ros2 component load /ComponentManager composition composition::Server; ros2 component load /ComponentManager composition composition::Client;  exec bash" 

ros2 run composition manual_composition

ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.so `ros2 pkg prefix composition`/lib/liblistener_component.so

ros2 launch composition composition_demo.launch.py

ros2 run rclcpp_components component_container
ros2 component list
gnome-terminal -- bash -c "ros2 component load /ComponentManager composition composition::Talker; ros2 component load /ComponentManager composition composition::Listener"; ros2 component unload /ComponentManager 1 2; exec bash"

ros2 run rclcpp_components component_container --ros-args -r __node:=MyContainer -r __ns:=/ns
gnome-terminal -- bash -c "ros2 component load /ns/MyContainer composition composition::Listener; exec bash"

ros2 run rclcpp_components component_container
gnome-terminal -- bash -c "ros2 component load /ComponentManager composition composition::Talker --node-name talker2; ros2 component load /ComponentManager composition composition::Talker --node-namespace /ns; ros2 component load /ComponentManager composition composition::Talker --node-name talker3 --node-namespace /ns2; ros2 component list; exec bash"

gnome-terminal -- bash -c "ros2 component load /ComponentManager image_tools image_tools::Cam2Image -p burger_mode:=true; exec bash"

gnome-terminal -- bash -c "gnome-terminal -- bash -c "ros2 component load /ComponentManager image_tools image_tools::Cam2Image -p burger_mode:=true; exec bash"; exec bash"
