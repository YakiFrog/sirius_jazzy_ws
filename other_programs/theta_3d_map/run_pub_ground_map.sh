#!/bin/bash
# 最新の ground_map.ply / ground_map.png を Foxglove 用に配信（/ground_map, /ground_map_image）。
set -u
WS="$HOME/sirius_jazzy_ws"
source "$WS/install/setup.bash" 2>/dev/null || source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-56}"
PLY=$(ls -t "$HOME"/rosbag2_data/*/ground_map.ply /media/*/rosbag2_data/*/ground_map.ply 2>/dev/null | head -1)
PNG=$(ls -t "$HOME"/rosbag2_data/*/ground_map.png /media/*/rosbag2_data/*/ground_map.png 2>/dev/null | head -1)
if [ -z "$PLY" ]; then echo "ground_map.ply が見つかりません（先に theta_kiss_ground_map を実行）"; read -r -p "Enterで閉じます..." _; exit 1; fi
echo "配信: ply=$PLY"
[ -n "$PNG" ] && echo "      png=$PNG"
ros2 run sirius_navigation theta_ply_publisher --ros-args -p ply_file:="$PLY" -p frame_id:=ground_map -p topic:=/ground_map &
[ -n "$PNG" ] && python3 "$WS/other_programs/theta_3d_map/theta_image_publisher.py" --ros-args -p image_file:="$PNG" -p topic:=/ground_map_image -p frame_id:=ground_map &
echo "Foxglove: ws://<このPCのIP>:8766  → 3D: /ground_map (frame ground_map) / Image: /ground_map_image"
echo "Ctrl+C で停止"
wait
