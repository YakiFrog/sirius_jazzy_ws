#!/bin/bash
# ==============================================================================
# rosbag(色付き点群) -> ループ閉じ込めSLAMで色付き3D地図(PLY)を生成するオフライン処理。
#   rtabmap(ICP, force3DoF) が map->odom を最適化し、theta_keyframe_map_node が
#   その最新姿勢で地図全体を再描画して保存する（壁の2重を解消）。
#
# 使い方: run_theta_slam_mapping.sh <bag_dir> [out.ply] [rate]
# ==============================================================================
set -u
WS_DIR="${HOME}/sirius_jazzy_ws"
set +u
source "${WS_DIR}/install/setup.bash" 2>/dev/null || source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-53}"

BAG="${1:-}"
ROSBAG_DIR="${HOME}/rosbag2_data"
if [ -z "$BAG" ]; then
    mapfile -t BAGS < <(find "$ROSBAG_DIR" -maxdepth 2 -name metadata.yaml -printf '%h\n' 2>/dev/null | sort -r)
    [ ${#BAGS[@]} -eq 0 ] && mapfile -t BAGS < <(find "$ROSBAG_DIR" -maxdepth 2 -name '*.mcap' -printf '%h\n' 2>/dev/null | sort -u -r)
    if [ ${#BAGS[@]} -eq 0 ]; then
        echo "bagが見つかりません: $ROSBAG_DIR"
        read -r -p "Enterで閉じます..." _; exit 1
    fi
    echo "処理するbagを選択:"
    for i in "${!BAGS[@]}"; do printf "  [%d] %s\n" "$((i + 1))" "$(basename "${BAGS[$i]}")"; done
    read -r -p "番号 (既定1): " SEL; SEL=${SEL:-1}
    if ! [[ "$SEL" =~ ^[0-9]+$ ]] || [ "$SEL" -lt 1 ] || [ "$SEL" -gt "${#BAGS[@]}" ]; then
        echo "不正な選択です"; read -r -p "Enterで閉じます..." _; exit 1
    fi
    BAG="${BAGS[$((SEL - 1))]}"
fi
if [ ! -d "$BAG" ]; then
    echo "bagディレクトリがありません: $BAG"
    read -r -p "Enterで閉じます..." _
    exit 1
fi
OUT="${2:-${BAG}/colored_map_slam.ply}"
RATE="${3:-1.0}"

echo "================================================="
echo " ループ閉じ込めSLAM オフライン地図生成"
echo "   bag : $BAG"
echo "   out : $OUT"
echo "   rate: $RATE"
echo "================================================="

ros2 launch sirius_navigation theta_slam_mapping.launch.py \
    use_sim_time:=true save_path:="$OUT" &
LAUNCH_PID=$!

sleep 6
echo "[play] ros2 bag play --clock (rate=$RATE)"
ros2 bag play "$BAG" --clock --rate "$RATE" || true

echo "[wait] rtabmap/SLAMの最適化待ち..."
sleep 8
echo "[stop] 保存のためキーフレームノードへSIGINT..."
pkill -INT -f theta_keyframe_map_node 2>/dev/null || true
sleep 10
kill -INT "$LAUNCH_PID" 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true

echo "================================================="
if [ -f "$OUT" ]; then
    echo "✓ 完成: $OUT ($(du -h "$OUT" | cut -f1))"
else
    echo "✗ 保存ファイルがありません: $OUT"
fi
echo "================================================="
read -r -p "Enterで閉じます..." _
