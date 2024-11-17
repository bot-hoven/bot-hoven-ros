#!/bin/bash

# This script is used to start the ROS project. 

# Build the workspace
colcon build

# Source the workspace setup file
source /bot-hoven-ros/install/setup.bash

# Launch the finger_control nodes
ros2 launch finger_control finger_control.launch.py > log/launch.log 2>&1 &