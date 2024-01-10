#!/bin/bash

# Array to store background task PIDs
task_pids=()

# Function to handle cleanup and terminate tasks
cleanup() {
    echo "Cleaning up and terminating tasks..."
    # Terminate all background tasks
    for pid in "${task_pids[@]}"; do
        kill -9 "$pid" 2>/dev/null
    done
    exit
}

# Function to simulate a task
run_magnum() {
    echo "Run magnum on pub $1 and sub $2"
    mc-rtc-magnum --tcp-pub-port $1 --tcp-sub-port $2
}

# Set up trap to catch exit signal (SIGINT)
trap 'cleanup' INT

# Run tasks in the background and store their PIDs
run_magnum 4242 4343 &
task_pids+=($!)
run_magnum 4444 4545 &
task_pids+=($!)
run_magnum 4646 4747 &
task_pids+=($!)
run_magnum 4848 4949 &
task_pids+=($!)
mc_rtc_ticker -f ~/.config/mc_rtc/mc_rtc_h1.yaml &
task_pids+=($!)
mc_rtc_ticker -f ~/.config/mc_rtc/mc_rtc_h2.yaml &
task_pids+=($!)

# Wait for all background tasks to complete
wait

echo "All tasks completed"