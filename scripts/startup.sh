#!/bin/bash

# This script is used to start the ROS project. 

# Build the workspace
cd /bot-hoven-ros
colcon build

# Source the workspace setup file
source /bot-hoven-ros/install/setup.bash

# Launch the finger_control nodes
ros2 launch finger_control finger_control.launch.py > log/launch.log 2>&1 &

# Launch the hand_control nodes
ros2 launch hand_control hand_control.launch.py > log/launch.log 2>&1 &

# Launch the home_position nodes
ros2 launch home_position home_position.launch.py > log/launch.log 2>&1 &