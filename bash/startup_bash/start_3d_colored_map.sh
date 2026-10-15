#!/bin/bash
# THETAで色付き3D点群マップを生成する（FAST-LIO2/lio3d と併用）。
# 使い方: start_3d_colored_map.sh [sim|real] [name]
#   sim  : Webシミュレータ。点群はブラウザ側で色付け済み(/velodyne_pointsのrgb)を使う
#   real : 実機THETA(/dev/theta_capture)で色付け（既定）
set -u
WS="$HOME/sirius_jazzy_ws"
MODE="${1:-real}"
NAME="${2:-colored_$(date +%m%d_%H%M)}"

if [ "$MODE" != "sim" ] && [ "$MODE" != "real" ]; then
    echo "使い方: $0 [sim|real] [name]"
    exit 1
fi

dir="$WS/maps_waypoints/maps3d/$NAME"
mkdir -p "$dir"

cd "$WS" || exit 1
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

if [ "$MODE" = "sim" ]; then
    echo "色付き3D点群マップを生成します [Webシミュ]"
    echo "  保存先: $dir/$NAME.ply (30秒ごと＋停止時に保存)"
    echo "  ※ Webシミュの「THETA 360°映像で色付け」をONにすると色が付きます（OFF時はリング色）"
    echo "  ※ 事前に先に lio3d_sim を起動しておいてください"
    exec ros2 run sirius_navigation theta_keyframe_map_node --ros-args \
        -r __node:=theta_keyframe_map_sim \
        -p use_sim_time:=true \
        -p cloud_topic:=/velodyne_points \
        -p pose_source:=odom \
        -p odom_topic:=/Odometry \
        -p body_frame:=body \
        -p odom_frame:=sirius3/odom \
        -p 'map_frame:=""' \
        -p output_topic:=/colored_map_3d \
        -p voxel:=0.05 \
        -p min_range:=0.5 \
        -p max_range:=80.0 \
        -p rebuild_period:=5.0 \
        -p autosave_period:=30.0 \
        -p save_path:="$dir/$NAME.ply"
else
    echo "色付き3D点群マップを生成します [実機THETA]"
    echo "  保存先: $dir/$NAME.ply (30秒ごと＋停止時に保存)"
    echo "  ※ 先に lio3d と tf_real を起動しておいてください"
    echo "  ※ 色付けには THETA (/dev/theta_capture) が必要です"
    exec ros2 launch sirius_navigation theta_3d_colored_mapping.launch.py \
        save_path:="$dir/$NAME.ply"
fi
