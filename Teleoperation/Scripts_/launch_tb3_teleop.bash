#!/bin/bash
# Script to launch TurtleBot3 teleoperation

# Source the ROS environment
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
# Set environment variable for TurtleBot3
export TURTLEBOT3_MODEL=burger  # Change model if needed

# Launch the teleop keyboard command for TurtleBot3
ros2 run turtlebot3_teleop teleop_keyboard

