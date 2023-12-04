# mateEZ
this is the team project for vip 1 course

## Requirements

- Operating System [Ubuntu 22.04.3 LTS (Jammy Jellyfish)](https://www.releases.ubuntu.com/jammy/)
- Connection to Internet
- Minimum 64 GB Ubuntu disk space
- [ROS2 Humble Hawksbill from Debian binary packages](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).


## Installation

- Open Terminal

- Move into home directory.

```bash
cd ~/
```

- Clone this repository into home directory.

```bash
https://github.com/igadgetstore/mateEZ.git
```

## Building the project

- Move into cloned repo's workspace called vip1_ws

```bash
cd ~/mateEZ/vip1_ws
```
- Resolve dependencies
- 
```bash
rosdep install -i --from-path src --rosdistro humble -y
```
- Build the workspace with colcon
- 
```bash
colcon build
```

## Usage 
- Open new terminal
- Source overlay (in each new terminal)

```bash
source /opt/ros/humble/setup.bash
```
- Go to the root of the workspace
```bash
cd ~/mateEZ/vip1_ws
```
- Source overlay in the root of workspace
```bash
source install/local_setup.bash
```
- Running obstacle avoidance or wall following simulation
```bash
ros2 launch autonomous_driving obstacle_avoidance.launch.py
```
or
```bash
ros2 launch autonomous_driving wall_following.launch.py
```

## Bringup TurtleBot3
- Bringup 
![Bringup](https://github.com/Risimon/Smart-Mobility-Engineering-Lab/blob/d3ea9827cdf2121f8cd3a9628ed4cdaa0d1c46cf/Snapshots/Turtlebot3%20SLAM/Screenshot%202023-11-20%20at%2023.43.41.png)
- Topic and Service list
![Topic and Service List](https://github.com/Risimon/Smart-Mobility-Engineering-Lab/blob/0a230c53af4f4d4413d913d3ebeefe48af204420/Snapshots/Turtlebot3%20SLAM/Screenshot%202023-11-20%20at%2023.45.25.png)

## SLAM
- Run SLAM node alog with teleop node
![cartographer](https://github.com/Risimon/Smart-Mobility-Engineering-Lab/blob/0a230c53af4f4d4413d913d3ebeefe48af204420/Snapshots/Turtlebot3%20SLAM/Screenshot%202023-11-20%20at%2023.40.56.png)
- Map Saving
![Occupancy Grid Map](https://github.com/Risimon/Smart-Mobility-Engineering-Lab/blob/0a230c53af4f4d4413d913d3ebeefe48af204420/Snapshots/Turtlebot3%20SLAM/Screenshot%202023-11-20%20at%2023.42.52.png)
- Demo Video
https://drive.google.com/file/d/1z96bAjeYfz8XRSJqYqAGl1hLWBYjjIgm/view?usp=sharing

## License

[MIT](https://choosealicense.com/licenses/mit/)
