#!/bin/bash

# ROS2 setup
if [ -f "$HOME/sirius_jazzy_ws/install/setup.bash" ]; then
    source "$HOME/sirius_jazzy_ws/install/setup.bash"
else
    source /opt/ros/jazzy/setup.bash
fi

echo "------------------------------------------------"
echo "  RTAB-MAP & SAM3 Bridge Startup"
echo "------------------------------------------------"

# Ask user if they want to include the background
read -p "RTAB-MAP生成時にセマンティック対象外の背景を含めますか？ (Y/n): " choice

# Convert response to lowercase
choice=$(echo "$choice" | tr '[:upper:]' '[:lower:]')

# Default to true (Y)
INCLUDE_BG="true"
if [ "$choice" = "n" ] || [ "$choice" = "no" ]; then
    INCLUDE_BG="false"
fi

echo "------------------------------------------------"
echo "起動中... (include_background:=$INCLUDE_BG)"
echo "------------------------------------------------"

ros2 launch sirius_navigation sam3_rtabmap.launch.py use_sim_time:=true include_background:="$INCLUDE_BG"
