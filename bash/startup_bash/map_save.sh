#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
cd ~/sirius_jazzy_ws
while : ;do
    read -p "Press [Enter] key to start map save..."
    source install/setup.bash
    echo "Input map name (without .yaml): "
    read map_name
    ros2 run nav2_map_server map_saver_cli -f ~/sirius_jazzy_ws/maps_waypoints/maps/$map_name
done