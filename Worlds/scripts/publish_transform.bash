#!/bin/bash
source /opt/ros/humble/setup.bash
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=8
export TURTLEBOT3_MODEL=burger
ros2 run tf2_ros static_transform_publisher --x -4.3 --y 6.5 --z 0 \
  --qx 0 --qy 0 --qz 0.1 --qw 1 --frame-id map --child-frame-id odom
