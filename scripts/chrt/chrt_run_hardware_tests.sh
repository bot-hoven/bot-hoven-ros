#!/bin/bash

# Run the colcon test command in a suspended state and get its PID
colcon test --packages-select hardware --ctest-args -L gtest -R "hardware_robotic_controller_performance_metric" --event-handlers console_direct+ &
PID=$!

# Suspend the process
kill -STOP $PID

# Set the PID with chrt "-f 50" flags
sudo chrt -f -p 50 $PID

# Resume the process
kill -CONT $PID

# Wait for the process to complete
wait $PID