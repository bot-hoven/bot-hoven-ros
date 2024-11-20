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

# Define the launch file name
LAUNCH_FILE="hardware.launch.py"

echo "Stopping launch file: $LAUNCH_FILE..."

# Find the process ID
PIDS=$(ps aux | grep $LAUNCH_FILE | grep -v grep | awk '{print $2}')

if [ -z "$PIDS" ]; then
    echo "No running processes found for $LAUNCH_FILE."
else
    # Kill each process
    for PID in $PIDS; do
        kill -9 $PID
        echo "Killed process with PID $PID"
    done
    echo "All processes for $LAUNCH_FILE have been stopped."
fi

# Kill the controller spawner nodes
pkill -f "spawner left_hand_controller"
pkill -f "spawner right_hand_controller"
echo "All controller spawner nodes have been stopped."