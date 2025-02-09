#!/bin/bash

LOG_FILE="goal_acceptance_times.csv"
echo "Simulated_CPU_Load_Percentage,Actual_CPU_Load_Percentage,Test_Run,Time_us" > $LOG_FILE

CPU_LOADS=(0 5 10 15 20 25 30 35 40 45 50 55 60 65 70 75)
NUM_RUNS=10

# Capture the start time
START_TIME=$(date +%s)

for LOAD in "${CPU_LOADS[@]}"; do
    echo "Setting CPU Load to $LOAD%..."
    
    # Start CPU load generator in the background
    stress-ng --cpu 4 --cpu-load $LOAD --timeout 180 --cpu-method loop & 
    STRESS_PID=$!

    sleep 3  # Allow time for load to stabilize

    for ((i=1; i<=NUM_RUNS; i++)); do
        echo "Running test $i at $LOAD% load..."
        
        # Get actual CPU load percentage
        ACTUAL_CPU_LOAD=$(mpstat 1 5 | awk '/Average/ {print 100 - $NF}')
        
        OUTPUT=$(colcon test --packages-select hardware --ctest-args -L gtest -R "hardware_robotic_controller_performance_metric" --event-handlers console_direct+ | grep "Time to accept goal")
        TIME_US=$(echo $OUTPUT | awk '{print $(NF-1)}') # Extract time value at second last position

        echo "$LOAD,$ACTUAL_CPU_LOAD,$i,$TIME_US" >> $LOG_FILE
    done

    # Stop stress test
    kill $STRESS_PID
    sleep 3  # Allow system to stabilize
done

# Capture the end time
END_TIME=$(date +%s)

# Calculate the total time taken
TOTAL_TIME=$((END_TIME - START_TIME))

echo "Test completed. Results saved in $LOG_FILE."
echo "Total time taken: $TOTAL_TIME seconds."
