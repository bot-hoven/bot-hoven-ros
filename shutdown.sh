# This script is used to shutdown the ROS project

# Kill the finger_control nodes
kill $(cat log/finger_control_launch.pid)
rm log/finger_control_launch.pid