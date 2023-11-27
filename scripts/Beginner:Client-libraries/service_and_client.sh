#! /bin/bash

#Background
#When nodes communicate using services, the node that sends a request for data is called the client node, and the one that responds to the request is the service node. The structure of the request and response is determined by a .srv file.

#The example used here is a simple integer addition system; one node requests the sum of two integers, and the other responds with the result.


#1 Create a package
source /opt/ros/humble/setup.bash #Open a new terminal and source your ROS 2 installation so that ros2 commands will work.
cd ~/Smart-Mobility-Engineering-Lab/ros2_ws #Navigate into the ros2_ws directory created in a previous tutorial.
cd src #Recall that packages should be created in the src directory, not the root of the workspace. 
ros2 pkg create --build-type ament_python py_srvcli --dependencies rclpy example_interfaces #Navigate into ros2_ws/src and create a new package

#1.1 Update package.xml
#Because you used the --dependencies option during package creation, you don’t have to manually add dependencies to package.xml.
nano package.xml #As always, though, make sure to add the description, maintainer email and name, and license information to package.xml.


#1.2 Update setup.py
nano setup.py #Add the same information to the setup.py file for the maintainer, maintainer_email, description and license fields
nano setup.cfg 


#2 Write the service node
nano  py_srvcli/service_member_function.py #Inside the ros2_ws/src/py_srvcli/py_srvcli directory, create a new file called service_member_function.py 
#and paste the following code within:
#from example_interfaces.srv import AddTwoInts

#import rclpy
#from rclpy.node import Node
#
#
#class MinimalService(Node):
#
#    def __init__(self):
#        super().__init__('minimal_service')
#        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
#
#    def add_two_ints_callback(self, request, response):
#        response.sum = request.a + request.b
#        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
#
#        return response
#
#
#def main():
#    rclpy.init()
#
#    minimal_service = MinimalService()
#
#    rclpy.spin(minimal_service)
#
#    rclpy.shutdown()
#
#
#if __name__ == '__main__':
#    main()


#2.2 Add an entry point
#To allow the ros2 run command to run your node, you must add the entry point to setup.py (located in the ros2_ws/src/py_srvcli directory).
nano setup.py #Add the following line between the 'console_scripts': brackets:


#3 Write the client node
#Inside the ros2_ws/src/py_srvcli/py_srvcli directory, create a new file called client_member_function.py
nano  py_srvcli/client_member_function.py 
#and paste the following code within:
#import sys
#
#from example_interfaces.srv import AddTwoInts
#import rclpy
#from rclpy.node import Node
#
#
#class MinimalClientAsync(Node):
#
#    def __init__(self):
#        super().__init__('minimal_client_async')
#        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
#        while not self.cli.wait_for_service(timeout_sec=1.0):
#            self.get_logger().info('service not available, waiting again...')
#        self.req = AddTwoInts.Request()
#
#    def send_request(self, a, b):
#        self.req.a = a
#        self.req.b = b
#        self.future = self.cli.call_async(self.req)
#        rclpy.spin_until_future_complete(self, self.future)
#        return self.future.result()
#
#
#def main():
#    rclpy.init()
#
#    minimal_client = MinimalClientAsync()
#    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
#    minimal_client.get_logger().info(
#        'Result of add_two_ints: for %d + %d = %d' %
#        (int(sys.argv[1]), int(sys.argv[2]), response.sum))
#
#    minimal_client.destroy_node()
#    rclpy.shutdown()
#
#
#if __name__ == '__main__':
#    main()


#3.2 Add an entry point
nano setup.py #Like the service node, you also have to add an entry point to be able to run the client node.
#The entry_points field of your setup.py file should look like this:
#entry_points={
#    'console_scripts': [
#        'service = py_srvcli.service_member_function:main',
#        'client = py_srvcli.client_member_function:main',
#    ],
#},


#4 Build and run
cd ~/Smart-Mobility-Engineering-Lab/ros2_ws #It’s good practice to run rosdep in the root of your workspace (ros2_ws) 
rosdep install -i --from-path src --rosdistro humble -y #to check for missing dependencies before building

sudo apt install python3-pip
pip install setuptools==58.2.0

colcon build --packages-select py_srvcli #Navigate back to the root of your workspace, ros2_ws, and build your new package:
gnome-terminal -- bash -c "cd ~/Smart-Mobility-Engineering-Lab/ros2_ws; source install/setup.bash; ros2 run py_srvcli service; exec bash" #Open a new terminal, navigate to ros2_ws, and source the setup files; Now run the service node
gnome-terminal -- bash -c "cd ~/Smart-Mobility-Engineering-Lab/ros2_ws; source install/setup.bash; ros2 run py_srvcli client 2 3; exec bash" #Open another terminal and source the setup files from inside ros2_ws again. Start the client node, followed by any two integers separated by a space:
#Enter Ctrl+C in the server terminal to stop the node from spinning.
