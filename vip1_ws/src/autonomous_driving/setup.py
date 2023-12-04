from setuptools import setup
import os
from glob import glob

package_name = 'autonomous_driving'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), # Launch files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')), # RViz config files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')), # World files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ri Yevgeniy',
    maintainer_email='yevgeniy@inha.edu',
    description='Obstacle avoidance and wall following autonomous driving functionalities.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoidance = autonomous_driving.obstacle_avoidance:main', # Python script (virtual)
            'wall_following = autonomous_driving.wall_following:main', # Python script (virtual)
        ],
    },
)
