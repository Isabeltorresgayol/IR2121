#!/bin/bash
source /opt/ros/humble/setup.bash
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=8
export TURTLEBOT3_MODEL=burger
ros2 launch map_server.launch.py

