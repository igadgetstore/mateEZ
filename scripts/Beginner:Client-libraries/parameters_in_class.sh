#! /bin/bash

#Background
#When making your own nodes you will sometimes need to add parameters that can be set from the launch file.
#This tutorial will show you how to create those parameters in a Python class, and how to set them in a launch file.


#1 Create a package
#Open a new terminal and source your ROS 2 installation so that ros2 commands will work.
#Recall that packages should be created in the src directory, not the root of the workspace. 
cd Smart-Mobility-Engineering-Lab/ros2_ws/src/ #Navigate into ros2_ws/src and create a new package:
ros2 pkg create --build-type ament_python python_parameters --dependencies rclpy

#1.1 Update package.xml
#Because you used the --dependencies option during package creation, you don’t have to manually add dependencies to package.xml or CMakeLists.txt.
#As always, though, make sure to add the description, maintainer email and name, and license information to package.xml.
nano python_parameters/package.xml 


#2 Write the Python node
#Inside the ros2_ws/src/python_parameters/python_parameters directory, create a new file called python_parameters_node.py and paste the following code within:
nano python_parameters/python_parameters/python_parameters_node.py
#import rclpy
#import rclpy.node
#
#class MinimalParam(rclpy.node.Node):
#    def __init__(self):
#        super().__init__('minimal_param_node')
#
#        self.declare_parameter('my_parameter', 'world')
#
#        self.timer = self.create_timer(1, self.timer_callback)
#
#    def timer_callback(self):
#        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value
#
#        self.get_logger().info('Hello %s!' % my_param)
#
#        my_new_param = rclpy.parameter.Parameter(
#            'my_parameter',
#            rclpy.Parameter.Type.STRING,
#            'world'
#        )
#        all_new_parameters = [my_new_param]
#        self.set_parameters(all_new_parameters)
#
#def main():
#    rclpy.init()
#    node = MinimalParam()
#    rclpy.spin(node)
#
#if __name__ == '__main__':
#    main()

#2.2 Add an entry point
#Open the setup.py file. Again, match the maintainer, maintainer_email, description and license fields to your package.xml:
nano python_parameters/setup.py 
#Add the following line within the console_scripts brackets of the entry_points field:
#entry_points={
#    'console_scripts': [
#        'minimal_param_node = python_parameters.python_parameters_node:main',
#    ],
#},


#3 Build and run
cd ..
rosdep install -i --from-path src --rosdistro humble -y #It’s good practice to run rosdep in the root of your workspace (ros2_ws) to check for missing dependencies before building:
colcon build --packages-select python_parameters #Navigate back to the root of your workspace, ros2_ws, and build your new package:
#Now run the node:
gnome-terminal -- bash -c "cd ~/Smart-Mobility-Engineering-Lab/ros2_ws; source install/setup.bash; ros2 run python_parameters minimal_param_node; exec bash" #Open a new terminal, navigate to ros2_ws, and source the setup files:

#3.1 Change via the console
#This part will use the knowledge you have gained from the tutoral about parameters and apply it to the node you have just created.
ros2 param list #inside ros2_ws again, and enter the following line:
#There you will see the custom parameter my_parameter. To change it, simply run the following line in the console:
ros2 param set /minimal_param_node my_parameter earth 

#3.2 Change via a launch file
#You can also set parameters in a launch file, but first you will need to add a launch directory.  
mkdir src/python_parameters/launch #Inside the ros2_ws/src/python_parameters/ directory, create a new directory called launch.
nano src/python_parameters/launch/python_parameters_launch.py #In there, create a new file called python_parameters_launch.py
#from launch import LaunchDescription
#from launch_ros.actions import Node
#
#def generate_launch_description():
#    return LaunchDescription([
#        Node(
#            package='python_parameters',
#            executable='minimal_param_node',
#            name='custom_minimal_param_node',
#            output='screen',
#            emulate_tty=True,
#            parameters=[
#                {'my_parameter': 'earth'}
#            ]
#        )
#    ])

#Now open the setup.py file. Add the import statements to the top of the file, and the other new statement to the data_files parameter to include all launch files:
nano src/python_parameters/setup.py 
#import os
#from glob import glob
## ...
#
#setup(
#  # ...
#  data_files=[
#      # ...
#      (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
#    ]
#  )

#navigate to the root of your workspace, ros2_ws, 
colcon build --packages-select python_parameters #and build your new package:
gnome-terminal -- bash -c "cd ~/Smart-Mobility-Engineering-Lab/ros2_ws; source install/setup.bash; ros2 launch python_parameters python_parameters_launch.py; exec bash" #Now run the node using the launch file we have just created:
