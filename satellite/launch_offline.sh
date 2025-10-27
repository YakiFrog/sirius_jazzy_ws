#!/bin/bash

# Offline Map Launch Script
# This script starts the tile server and RViz with offline map

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Check if tiles directory exists
if [ ! -d "tiles" ]; then
    echo "Error: tiles directory not found!"
    echo "Please download tiles first using: python3 download_tiles.py"
    exit 1
fi

# Check if tile server script exists
if [ ! -f "tile_server.py" ]; then
    echo "Error: tile_server.py not found!"
    exit 1
fi

echo "Starting offline map demo..."
echo "Working directory: $SCRIPT_DIR"
echo ""

# Start tile server in background
echo "Starting tile server on port 8000..."
python3 tile_server.py 8000 ./tiles &
TILE_SERVER_PID=$!

# Wait for tile server to start
sleep 2

# Check if tile server is running
if ! ps -p $TILE_SERVER_PID > /dev/null; then
    echo "Error: Failed to start tile server"
    exit 1
fi

echo "Tile server started (PID: $TILE_SERVER_PID)"
echo "Server URL: http://localhost:8000/{z}/{x}/{y}.png"
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down..."
    echo "Stopping tile server (PID: $TILE_SERVER_PID)..."
    kill $TILE_SERVER_PID 2>/dev/null
    wait $TILE_SERVER_PID 2>/dev/null
    echo "Done."
    exit 0
}

# Set trap to cleanup on CTRL+C
trap cleanup SIGINT SIGTERM

# Launch ROS2 with offline configuration
echo "Launching ROS2 with offline map..."
ros2 launch demo_offline.launch.xml

# Cleanup after ROS2 exits
cleanup
