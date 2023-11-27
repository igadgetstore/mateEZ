#! /bin/bash

#Start up the two turtlesim nodes, /turtlesim and /teleop_turtle.
gnome-terminal -- ros2 run turtlesim turtlesim_node
gnome-terminal -- ros2 run turtlesim turtle_teleop_key

#Running the ros2 service list command in a new terminal will return a list of all the services currently active in the system:
ros2 service list -t

#Services have types that describe how the request and response data of a service is structured. Service types are defined similarly to topic types, except service types have two parts: one message for the request and another for the response.
#Let’s take a look at turtlesim’s /clear service. In a new terminal, enter the command: 
ros2 service type /clear
#The Empty type means the service call sends no data when making a request and receives no data when receiving a response.

#If you want to find all the services of a specific type, you can use the command:
ros2 service find std_srvs/srv/Empty

#You can call services from the command line, but first you need to know the structure of the input arguments.
#Try this on the /clear service’s type, Empty:
ros2 interface show std_srvs/srv/Empty
#The --- separates the request structure (above) from the response structure (below). But, as you learned earlier, the Empty type doesn’t send or receive any data. So, naturally, its structure is blank.

#Let’s introspect a service with a type that sends and receives data, like /spawn. From the results of ros2 service list -t, we know /spawn’s type is turtlesim/srv/Spawn.
#To see the request and response arguments of the /spawn service, run the command:
ros2 interface show turtlesim/srv/Spawn
#The information above the --- line tells us the arguments needed to call /spawn. x, y and theta determine the 2D pose of the spawned turtle, and name is clearly optional.
#The information below the line isn’t something you need to know in this case, but it can help you understand the data type of the response you get from the call.

#Now that you know what a service type is, how to find a service’s type, and how to find the structure of that type’s arguments, you can call a service using:
#The <arguments> part is optional. For example, you know that Empty typed services don’t have any arguments:
ros2 service call /clear std_srvs/srv/Empty

#Now let’s spawn a new turtle by calling /spawn and setting arguments. Input <arguments> in a service call from the command-line need to be in YAML syntax.
#Enter the command:
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"
