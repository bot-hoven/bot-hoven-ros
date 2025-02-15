#!/bin/bash

ros2 launch hardware hardware.launch.py use_mock_hardware:=true &
PID=$!

# Suspend the process
kill -STOP $PID

# Set the PID with chrt "-f 50" flags
sudo chrt -f -p 50 $PID

# Resume the process
kill -CONT $PID