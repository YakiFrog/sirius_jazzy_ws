#!/bin/bash

# Get the path to the workspace root
WS_DIR=$(cd "$(dirname "$0")/../.." && pwd)

echo "Sourcing workspace in $WS_DIR..."
source "$WS_DIR/install/setup.bash"

echo "Starting ROS-TCP Endpoint..."
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1
