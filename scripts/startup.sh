#!/bin/bash

# This script is used to start the ROS project. 

# Build the workspace
echo "Building the workspace..."
cd /bot-hoven-ros
colcon build

# Source the workspace setup file
echo "Sourcing the workspace setup file..."
source /bot-hoven-ros/install/setup.bash

# Launch the finger_control nodes
echo "Launching the finger_control nodes..."
ros2 launch finger_control finger_control.launch.py > log/launch.log 2>&1 &
echo "finger_control nodes launched."

# Launch the hand_control nodes
echo "Launching the hand_control nodes..."
ros2 launch hand_control hand_control.launch.py > log/launch.log 2>&1 &
echo "hand_control nodes launched."

# Launch the home_position nodes
echo "Launching the home_position nodes..."
ros2 launch home_position home_position.launch.py > log/launch.log 2>&1 &
echo "home_position nodes launched."

# Launch the hardware nodes
echo "Launching the hardware nodes..."
ros2 launch hardware hardware.launch.py > log/launch.log 2>&1 &
echo "hardware nodes launched."

echo "All nodes have been launched successfully."