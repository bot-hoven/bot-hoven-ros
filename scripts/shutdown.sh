#!/bin/bash

# This script is used to shutdown the ROS project. 
# It stops all ROS2 nodes and removes the log files.

# Function to check if a process is running
function is_process_running() {
    pgrep -x $1 > /dev/null
    return $?
}

# Kill all ROS2 nodes
echo "Stopping all ROS2 nodes..."
ros2_procs=$(ps aux | grep -v grep | grep /bot-hoven-ros/install | awk '{print $2}')

if [ -z "$ros2_procs" ]; then
    echo "No ROS2 nodes are currently running."
else
    echo "Killing ROS2 processes..."
    for pid in $ros2_procs; do
        kill -9 $pid
        echo "Killed process with PID $pid"
    done
    echo "All ROS2 nodes have been stopped."
fi

echo "All relevant processes have been terminated. ROS2 is now stopped."

# Remove log files
rm /bot-hoven-ros/log/launch.log