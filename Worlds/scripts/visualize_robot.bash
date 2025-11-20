#!/bin/bash
source /opt/ros/humble/setup.bash
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=8
export TURTLEBOT3_MODEL=burger
rviz2 -d config_robot.rviz

