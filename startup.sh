#!/bin/bash

# Build the workspace
colcon build

# Source the workspace setup file
source /bot-hoven-ros/install/setup.bash

# Launch the finger_control nodes
ros2 launch finger_control finger_control.launch.py > log/launch.log 2>&1 &

# Save the PID of the last background process
echo $! > log/finger_control_launch.pid