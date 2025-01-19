#!/bin/bash

# Remove previous build, install, and log directories
echo "Cleaning up build, install, and log directories..."
rm -rf build install log

# Run colcon build
echo "Building the project with colcon..."
if ! colcon build; then
    echo "Build failed. Exiting."
    exit 1
fi

# Source the setup file
echo "Sourcing the setup file..."
source install/setup.bash

# Launch the ROS2 launch file
echo "Launching the ROS2 hardware system..."
ros2 launch hardware hardware.launch.py
