#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
cd ~/sirius_jazzy_ws
while : ;do
    read -p "Press [Enter] key to start nav2 bringup..."
    source install/setup.bash
    echo "Input map name (without .yaml): "
    read map_name
    ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=True \
    map:=${HOME}/sirius_jazzy_ws/maps_waypoints/maps/$map_name.yaml \
    params_file:=${HOME}/sirius_jazzy_ws/params/nav2_params_sim.yaml \
    use_composition:=False
done