#! /bin/bash

#Actions are one of the communication types in ROS 2 and are intended for long running tasks. They consist of three parts: a goal, feedback, and a result.

#Actions are built on topics and services. Their functionality is similar to services, except actions can be canceled. They also provide steady feedback, as opposed to services which return a single response.

#Actions use a client-server model, similar to the publisher-subscriber model (described in the topics tutorial). An “action client” node sends a goal to an “action server” node that acknowledges the goal and returns a stream of feedback and a result.


#Start up the two turtlesim nodes, /turtlesim and /teleop_turtle.
gnome-terminal -- ros2 run turtlesim turtlesim_node
gnome-terminal -- ros2 run turtlesim turtle_teleop_key
#Pay attention to the terminal where the /turtlesim node is running. Each time you press one of these keys, you are sending a goal to an action server that is part of the /turtlesim node.

#To see the list of actions a node provides, /turtlesim in this case, open a new terminal and run the command:
ros2 node info /turtlesim

#The /teleop_turtle node has the name /turtle1/rotate_absolute under Action Clients meaning that it sends goals for that action name. To see that, run:
ros2 node info /teleop_turtle

#To identify all the actions in the ROS graph, run the command:
ros2 action list
#This is the only action in the ROS graph right now. It controls the turtle’s rotation, as you saw earlier. You also already know that there is one action client (part of /teleop_turtle) and one action server (part of /turtlesim) for this action from using the ros2 node info <node_name> command.
ros2 action list -t
#In brackets to the right of each action name (in this case only /turtle1/rotate_absolute) is the action type, turtlesim/action/RotateAbsolute. You will need this when you want to execute an action from the command line or from code.

#One more piece of information you will need before sending or executing an action goal yourself is the structure of the action type.
#Recall that you identified /turtle1/rotate_absolute’s type when running the command ros2 action list -t. Enter the following command with the action type in your terminal:
ros2 action info /turtle1/rotate_absolute
#The section of this message above the first --- is the structure (data type and name) of the goal request. The next section is the structure of the result. The last section is the structure of the feedback.

#You can further introspect the /turtle1/rotate_absolute action with the command:
ros2 interface show turtlesim/action/RotateAbsolute

#Now let’s send an action goal from the command line. Keep an eye on the turtlesim window, and enter the following command into your terminal:
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
#All goals have a unique ID, shown in the return message. You can also see the result, a field with the name delta, which is the displacement to the starting position.

#To see the feedback of this goal, add --feedback to the ros2 action send_goal command:
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback

