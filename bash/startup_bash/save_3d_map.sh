#!/bin/bash
# 現在の3D点群を保存し、2D占有格子へ投影する。
# 既定トピックは FAST-LIO2 の累積マップ /Laser_map。
#   - FAST-LIO2利用時: /Laser_map （camera_init=地図原点）
#   - RTAB-Map利用時:  /cloud_map （mapフレーム）
# 出力: maps_waypoints/maps3d/<name>/<name>.npy .pcd .pgm .yaml
set -u
WS="$HOME/sirius_jazzy_ws"
name="${1:-3dmap_$(date +%m%d_%H%M)}"
topic="${2:-}"

cd "$WS" || exit 1
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

if [ -z "$topic" ]; then
    if ros2 topic list 2>/dev/null | grep -qx "/cloud_map"; then
        topic="/cloud_map"
    else
        topic="/Laser_map"
    fi
fi
dir="$WS/maps_waypoints/maps3d/$name"
mkdir -p "$dir"

echo "3D地図を保存します"
echo "  トピック: $topic"
echo "  出力: $dir/$name.(npy|pcd|pgm|yaml)"
echo "  ※/cloud_map があれば優先、無ければ FAST-LIO2 の /Laser_map を使用"
echo "  ※トピックを明示する場合: save_3d_map.sh <name> <topic>"
exec ros2 run sirius_navigation save_cloud_node --ros-args \
    -p topic:="$topic" \
    -p output_base:="$dir/$name" \
    -p target_frame:="" \
    -p project_2d:=true
