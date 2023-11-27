#! /bin/bash

#Background
#Actions are a form of asynchronous communication in ROS 2. Action clients send goal requests to action servers. Action servers send goal feedback and results to action clients.

#1 Writing an action server
#Let’s focus on writing an action server that computes the Fibonacci sequence using the action we created in the Creating an action tutorial.
#Until now, you’ve created packages and used ros2 run to run your nodes. To keep things simple in this tutorial, however, we’ll scope the action server to a single file. If you’d like to see what a complete package for the actions tutorials looks like, check out action_tutorials.
cd ~/Smart-Mobility-Engineering-Lab/ros2_ws/src
#Open a new file in your home directory, let’s call it fibonacci_action_server.py, and add the following code:
nano ~/Smart-Mobility-Engineering-Lab/fibonacci_action_server.py
#import rclpy
#from rclpy.action import ActionServer
#from rclpy.node import Node
#
#from action_tutorials_interfaces.action import Fibonacci
#
#
#class FibonacciActionServer(Node):
#
#    def __init__(self):
#        super().__init__('fibonacci_action_server')
#        self._action_server = ActionServer(
#            self,
#            Fibonacci,
#            'fibonacci',
#            self.execute_callback)
#
#    def execute_callback(self, goal_handle):
#        self.get_logger().info('Executing goal...')
#        result = Fibonacci.Result()
#        return result
#
#
#def main(args=None):
#    rclpy.init(args=args)
#
#    fibonacci_action_server = FibonacciActionServer()
#
#    rclpy.spin(fibonacci_action_server)
#
#
#if __name__ == '__main__':
#    main()

#Let’s try running our action server:
python3 ~/Smart-Mobility-Engineering-Lab/fibonacci_action_server.py


#gnome-terminal -- bash -c "ros2 action send_goal --feedback fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"; python3 ~/Smart-Mobility-Engineering-Lab/fibonacci_action_client.py; nano ~/Smart-Mobility-Engineering-Lab/fibonacci_action_client.py; python3 ~/Smart-Mobility-Engineering-Lab/fibonacci_action_client.py; exec bash"
