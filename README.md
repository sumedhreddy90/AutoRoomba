# Autonomous Roomba Vaccum Cleaner

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
---
[![BagFile Link](https://readthedocs.org/projects/ansicolortags/badge/?version=latest)](https://drive.google.com/file/d/1ONlMlc4U2EQrxI3sEe9bCFMNWGaF0u8K/view?usp=sharing)

## Introduction

Simulation of Autonomous roomba vacuum cleaner using turtlebot robot in a gazebo environment is done using RoombaWalker class in C++ which takes the input as laser scan from the laser sensor mounted on the turtlebot.


## Dependencies

This package has been tested in a system with following dependencies.
- Ubuntu 20.04 LTS
- ROS-noetic 
- turtlebot3 simulator

## Build Instructions

1) To install ROS-noetic follow the steps mentioned in the official website (http://wiki.ros.org/noetic/Installation/Ubuntu)

2) To install `turtlebot_sumulator` package
```
sudo apt-get install ros-${ROS_DISTRO}-turtlebot3-gazebo
``` 

4) After installing the above mentioned dependencies run the following commands to download this project.
```
source /opt/ros/noetic/setup.bash
mkdir -p ~/workspace/src
cd ~/workspace/src/
git clone https://github.com/sumedhreddy90/AutoRoomba.git
cd ../ 
catkin_make
```

## Run Instructions

1) To launch the program
```
cd ~/workspace/
source devel/setup.bash
roslaunch auto_roomba roomba.launch rosbagRecorder:=false
# Change rosbagRecorde:=true to record a bag file for 15 seconds
```

2) To replay the recorded rosbag file, you can execute the `rosbag play	` command with the roombarecord.bag file present in results folder. Run roscre in a separate terminal
```
cd ~/workspace/
source devel/setup.bash
cd ~/workspace/src/AutoRoomba/results/
rosbag play roombarecord.bag 
```