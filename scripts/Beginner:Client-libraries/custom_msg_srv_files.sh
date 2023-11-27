#! /bin/bash

#Background
#In previous tutorials you utilized message and service interfaces to learn about topics, services, and simple publisher/subscriber (C++/Python) and service/client (C++/Python) nodes. The interfaces you used were predefined in those cases.

#While it’s good practice to use predefined interface definitions, you will probably need to define your own messages and services sometimes as well. This tutorial will introduce you to the simplest method of creating custom interface definitions.

#1 Create a new package
#For this tutorial you will be creating custom .msg and .srv files in their own package, and then utilizing them in a separate package. Both packages should be in the same workspace.
#Since we will use the pub/sub and service/client packages created in earlier tutorials
cd ~/Smart-Mobility-Engineering-Lab/ros2_ws/src #make sure you are in the same workspace as those packages (ros2_ws/src)

ros2 pkg create --build-type ament_cmake tutorial_interfaces #then run the following command to create a new package:

cd tutorial_interfaces/ #The .msg and .srv files are required to be placed in directories called msg and srv respectively
mkdir msg srv #Create the directories in ros2_ws/src/tutorial_interfaces


#2 Create custom definitions
#2.1 msg definition
printf "int64 num" >> msg/Num.msg # In the tutorial_interfaces/msg directory you just created, make a new file called Num.msg with one line of code declaring its data structure: 
printf "geometry_msgs/Point center\nfloat64 radius" >> msg/Sphere.msg #Also in the tutorial_interfaces/msg directory you just created, make a new file called Sphere.msg with the following content:


#2.2 srv definition
printf "int64 a\nint64 b\nint64 c\n---\nint64 sum" >> srv/AddThreeInts.srv #Back in the tutorial_interfaces/srv directory you just created, make a new file called AddThreeInts.srv with the following request and response structure:


#3 CMakeLists.txt
#To convert the interfaces you defined into language-specific code (like C++ and Python) so that they can be used in those languages, add the following lines to CMakeLists.txt:
nano CMakeLists.txt 
#find_package(geometry_msgs REQUIRED)
#find_package(rosidl_default_generators REQUIRED)
#
#rosidl_generate_interfaces(${PROJECT_NAME}
#  "msg/Num.msg"
#  "msg/Sphere.msg"
#  "srv/AddThreeInts.srv"
#  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
#)


#4 package.xml
#Because the interfaces rely on rosidl_default_generators for generating language-specific code, you need to declare a build tool dependency on it. rosidl_default_runtime is a runtime or execution-stage dependency, needed to be able to use the interfaces later. The rosidl_interface_packages is the name of the dependency group that your package, tutorial_interfaces, should be associated with, declared using the <member_of_group> tag.
#Add the following lines within the <package> element of package.xml:
nano package.xml 
#<depend>geometry_msgs</depend>
#<buildtool_depend>rosidl_default_generators</buildtool_depend>
#<exec_depend>rosidl_default_runtime</exec_depend>
#<member_of_group>rosidl_interface_packages</member_of_group>


#5 Build the tutorial_interfaces package
#Now that all the parts of your custom interfaces package are in place, you can build the package.
cd ~/Smart-Mobility-Engineering-Lab/ros2_ws/ #In the root of your workspace (~/ros2_ws)
colcon build --packages-select tutorial_interfaces # run the following command


#6 Confirm msg and srv creation
#In a new terminal, run the following command from within your workspace (ros2_ws) to source it:
gnome-terminal -- bash -c "cd ~/Smart-Mobility-Engineering-Lab/ros2_ws; source install/local_setup.bash; ros2 interface show tutorial_interfaces/msg/Num; ros2 interface show tutorial_interfaces/msg/Sphere; ros2 interface show tutorial_interfaces/srv/AddThreeInts; exec bash"
#Now you can confirm that your interface creation worked by using the ros2 interface show command:
#should return: int64 num

#And ros2 interface show tutorial_interfaces/msg/Sphere
#should return:
#geometry_msgs/Point center
#        float64 x
#        float64 y
#        float64 z
#float64 radius

#And ros2 interface show tutorial_interfaces/srv/AddThreeInts
#should return:
#int64 a
#int64 b
#int64 c
#---
#int64 sum


#7.1 Testing Num.msg with pub/sub
#With a few modifications to the publisher/subscriber package created in a previous tutorial (C++ or Python), you can see Num.msg in action. Since you’ll be changing the standard string msg to a numerical one, the output will be slightly different.
#Publisher
nano /home/zheka/Smart-Mobility-Engineering-Lab/ros2_ws/src/py_pubsub/py_pubsub/publisher_member_function.py
#Subscriber
nano /home/zheka/Smart-Mobility-Engineering-Lab/ros2_ws/src/py_pubsub/py_pubsub/subscriber_member_function.py
#package.xml
#Add the following line:
nano /home/zheka/Smart-Mobility-Engineering-Lab/ros2_ws/src/py_pubsub/package.xml
#<exec_depend>tutorial_interfaces</exec_depend>

#After making the above edits and saving all the changes, build the package:
colcon build --packages-select py_pubsub
#Then open two new terminals, source ros2_ws in each, and run:
gnome-terminal -- bash -c "cd ~/Smart-Mobility-Engineering-Lab/ros2_ws; source install/local_setup.bash; ros2 run py_pubsub talker; exec bash"
gnome-terminal -- bash -c "cd ~/Smart-Mobility-Engineering-Lab/ros2_ws; source install/local_setup.bash; ros2 run py_pubsub listener; exec bash"


#7.2 Testing AddThreeInts.srv with service/client
#With a few modifications to the service/client package created in a previous tutorial (C++ or Python), you can see AddThreeInts.srv in action. Since you’ll be changing the original two integer request srv to a three integer request srv, the output will be slightly different.
#Service
nano /home/zheka/Smart-Mobility-Engineering-Lab/ros2_ws/src/py_srvcli/py_srvcli/service_member_function.py
#Client
nano /home/zheka/Smart-Mobility-Engineering-Lab/ros2_ws/src/py_srvcli/py_srvcli/client_member_function.py
#package.xml
#Add the following line:
nano /home/zheka/Smart-Mobility-Engineering-Lab/ros2_ws/src/py_srvcli/package.xml
#<exec_depend>tutorial_interfaces</exec_depend>

#After making the above edits and saving all the changes, build the package:
colcon build --packages-select py_pubsub
#Then open two new terminals, source ros2_ws in each, and run:
gnome-terminal -- bash -c "cd ~/Smart-Mobility-Engineering-Lab/ros2_ws; source install/local_setup.bash; ros2 run py_srvcli service; exec bash"
gnome-terminal -- bash -c "cd ~/Smart-Mobility-Engineering-Lab/ros2_ws; source install/local_setup.bash; ros2 run py_srvcli client 2 3 1; exec bash"
