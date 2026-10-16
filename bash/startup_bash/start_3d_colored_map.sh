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

# 同じROS domainの /colored_map_3d は一つのプロセスだけが配信する。
# FD 9はros2終了まで保持する。スクリプト自体も残してランチャーの署名検出を保つ。
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/sirius_colored3d_${UID}_${ROS_DOMAIN_ID:-0}.lock"
if ! flock -n 9; then
    echo "色付き3D地図は既に起動しています (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0})"
    exit 1
fi

if [ "$MODE" = "sim" ]; then
    # lio3d_simと同じ設定から body(IMU) <- LiDAR を読む。
    # TFツリー経由の推測や固定値の重複を避ける。
    PROFILE_DIR=$(mktemp -d "${TMPDIR:-/tmp}/sirius_color_profile.XXXXXX")
    trap 'rm -rf "$PROFILE_DIR"' EXIT
    python3 "$WS/other_programs/theta_3d_map/web_sensor_profile.py" \
        --template "$WS/params/fast_lio_sirius_sim.yaml" --output "$PROFILE_DIR/profile.yaml" >/dev/null || exit 1
    extrinsics=$(python3 - "$PROFILE_DIR/profile.yaml" <<'PY'
import json
import sys
import yaml

with open(sys.argv[1]) as stream:
    mapping = yaml.safe_load(stream)['/**']['ros__parameters']['mapping']
print(json.dumps(mapping['extrinsic_T']))
print(json.dumps(mapping['extrinsic_R']))
PY
    ) || exit 1
    mapfile -t lidar_extrinsics <<< "$extrinsics"
    echo "色付き3D点群マップを生成します [Webシミュ]"
    echo "  形状: /Laser_map のXYZを保持し、RGBのみ付与します"
    echo "  保存先: $dir/$NAME.ply (30秒ごと＋停止時に保存)"
    echo "  ※ Webシミュの「THETA 360°映像で色付け」をONにすると色が付きます（OFF時はリング色）"
    echo "  ※ 事前に先に lio3d_sim を起動しておいてください"
    ros2 run sirius_navigation theta_keyframe_map_node --ros-args \
        -r __node:=theta_keyframe_map_sim \
        -p use_sim_time:=true \
        -p cloud_topic:=/velodyne_points \
        -p pose_source:=odom \
        -p odom_topic:=/Odometry \
        -p body_frame:=body \
        -p pose_tolerance:=0.002 \
        -p use_lidar_extrinsic:=true \
        -p lidar_frame:=sirius3/lidar_link \
        -p lidar_translation:="${lidar_extrinsics[0]}" \
        -p lidar_rotation:="${lidar_extrinsics[1]}" \
        -p odom_frame:=sirius3/odom \
        -p 'map_frame:=""' \
        -p output_topic:=/colored_map_3d \
        -p geometry_topic:=/Laser_map \
        -p color_max_distance:=0.5 \
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
    ros2 launch sirius_navigation theta_3d_colored_mapping.launch.py \
        save_path:="$dir/$NAME.ply"
fi
