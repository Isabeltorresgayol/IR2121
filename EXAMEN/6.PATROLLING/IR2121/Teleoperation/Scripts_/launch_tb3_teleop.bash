#!/bin/bash
# Script to launch TurtleBot3 teleop

# Source ROS 2 workspace
source ~/ros2_ws/install/setup.bash

# Launch teleop
ros2 launch turtlebot3_teleop teleop_keyboard.launch.py
