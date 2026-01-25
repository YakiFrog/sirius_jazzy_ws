#!/bin/bash

# Configuration
WS_DIR=$(cd "$(dirname "$0")/../.." && pwd)
UNITY_SIM_LAUNCH="unity_sim.launch.py"
PACKAGE_NAME="sirius_description"
ROS_TCP_SCRIPT="$WS_DIR/bash/startup_bash/start_ros_tcp.sh"

# Function to kill child processes on exit
cleanup() {
    echo "Shutting down..."
    kill 0
}
trap cleanup SIGINT SIGTERM EXIT

echo "Sourcing workspace in $WS_DIR..."
source "$WS_DIR/install/setup.bash"

# 1. Start ROS-TCP Endpoint
echo "Starting ROS-TCP Endpoint..."
# We run it in background
"$ROS_TCP_SCRIPT" &
PID_TCP=$!
sleep 2

# 2. Start Unity Sim Launch (Robot State Publisher, Rviz)
echo "Starting Unity Sim Launch..."
ros2 launch "$PACKAGE_NAME" "$UNITY_SIM_LAUNCH" &
PID_LAUNCH=$!
sleep 5

# 3. Optional: Run Unity Executable if path provided
if [ -n "$1" ]; then
    UNITY_EXE="$1"
    if [ -f "$UNITY_EXE" ]; then
        echo "Starting Unity Simulation: $UNITY_EXE"
        chmod +x "$UNITY_EXE"
        "$UNITY_EXE" &
        PID_UNITY=$!
    else
        echo "Error: Unity executable not found at $UNITY_EXE"
    fi
else
    echo "No Unity executable path provided. Waiting for manual Unity start..."
    echo "Usage: ./launch_simulation.sh [path_to_unity_executable]"
fi

# Wait for processes
wait $PID_TCP $PID_LAUNCH ${PID_UNITY:-}
