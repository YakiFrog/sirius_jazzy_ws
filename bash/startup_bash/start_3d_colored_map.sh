#!/bin/bash
# THETAで色付き3D点群マップを生成する（FAST-LIO2/lio3d と併用）。
# 30秒ごと＋停止時(SIGINT)に maps_waypoints/maps3d/<name>/<name>.ply へ保存する。
set -u
WS="$HOME/sirius_jazzy_ws"
name="${1:-colored_$(date +%m%d_%H%M)}"
dir="$WS/maps_waypoints/maps3d/$name"
mkdir -p "$dir"

cd "$WS" || exit 1
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

echo "色付き3D点群マップを生成します"
echo "  保存先: $dir/$name.ply (30秒ごと＋停止時に保存)"
echo "  ※ 先に lio3d と tf_real を起動しておいてください"
echo "  ※ 色付けには THETA (/dev/theta_capture) が必要です"
exec ros2 launch sirius_navigation theta_3d_colored_mapping.launch.py \
    save_path:="$dir/$name.ply"
