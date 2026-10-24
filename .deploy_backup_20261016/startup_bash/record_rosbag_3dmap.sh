#!/bin/bash
# ==============================================================================
# 色付き3D地図 / オフラインSLAM用 ROS2 bag 記録スクリプト
#   /velodyne_points_colored(色付き) + /velodyne_points(生: deskew/ring/time) +
#   /imu + /magnetometer + THETA画像 + TF + odom を同期録画する。
#   用途:
#     - 色付き地図: theta_bag_to_ply.py / theta_offline_slam.py (KISS-ICP)
#     - LiDAR-IMU SLAM: FAST-LIO2 等（生点群+IMUを使用）
#
# 使い方: record_rosbag_3dmap.sh [sim|real]
#   - real(既定): システム時刻。/clock 不要。
#   - sim       : /clock を含める。
# 事前に VLP16(/velodyne_points) と THETA(theta_capture)・色付け
#   (theta_colorize_cloud_node) が動いていること。
# ==============================================================================
MODE="${1:-real}"
ROSBAG_PID=""
CURRENT_BAG_PATH=""
RESULT_SHOWN=false

WS_DIR="${HOME}/sirius_jazzy_ws"
ROSBAG_DIR="${HOME}/rosbag2_data"

set +u
if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
else
    source /opt/ros/jazzy/setup.bash
fi
set -u

if [ "$MODE" = "sim" ]; then
    REQUIRE_CLOCK=true
    SOURCE_HINT="THETAはWebシミュレータの配信を使う（実機キャプチャ不要）"
else
    REQUIRE_CLOCK=false
    SOURCE_HINT="実機: theta_capture + theta_colorize_cloud_node が稼働していること"
fi

show_result() {
    [ "$RESULT_SHOWN" = true ] && return
    RESULT_SHOWN=true
    echo ""
    echo "================================================="
    if [ -d "$CURRENT_BAG_PATH" ]; then
        echo "✓ 録画を保存しました: $CURRENT_BAG_PATH"
        du -sh "$CURRENT_BAG_PATH" 2>/dev/null || true
        echo ""
        echo "次のコマンドで色付き3D地図(PLY)を生成できます:"
        echo "  python3 ${WS_DIR}/other_programs/theta_3d_map/theta_bag_to_ply.py \\"
        echo "      \"$CURRENT_BAG_PATH\" -o \"${CURRENT_BAG_PATH}/colored_map.ply\""
    else
        echo "録画データは作成されませんでした。"
    fi
    echo "================================================="
}

cleanup() {
    local status="${1:-0}"
    if [ -n "$ROSBAG_PID" ]; then
        echo ""
        echo "録画を停止・保存しています... (Ctrl+Cを受け付けました)"
        kill -INT "$ROSBAG_PID" 2>/dev/null || true
        wait "$ROSBAG_PID" 2>/dev/null || true
        ROSBAG_PID=""
        # mcap索引の修復 + メタデータ生成（既存スクリプトと同様の保険）
        if [ -d "$CURRENT_BAG_PATH" ]; then
            mcap_file=$(find "$CURRENT_BAG_PATH" -name "*.mcap" | head -n 1)
            if [ -n "$mcap_file" ] && [ -x "${WS_DIR}/mcap" ]; then
                tmp="${mcap_file}.tmp"
                if "${WS_DIR}/mcap" recover "$mcap_file" -o "$tmp" >/dev/null 2>&1; then
                    mv "$tmp" "$mcap_file"
                else
                    rm -f "$tmp"
                fi
            fi
            if [ ! -f "$CURRENT_BAG_PATH/metadata.yaml" ]; then
                ros2 bag reindex "$CURRENT_BAG_PATH" -s mcap >/dev/null 2>&1 || true
            fi
        fi
    fi
    show_result
    read -r -p "Enterで閉じます..." _ || true
    exit "$status"
}

trap 'cleanup 130' INT TERM HUP

check_topic() {
    local topic="$1" type="$2"
    local actual
    actual=$(ros2 topic type "$topic" 2>/dev/null | head -n 1)
    if [ "$actual" != "$type" ]; then
        echo "  ✗ $topic がないか型不一致 ($actual)"
        return 1
    fi
    echo "  ✓ $topic"
    return 0
}

preflight() {
    local errors=0
    echo ""
    echo "録画前チェック"
    check_topic /velodyne_points_colored sensor_msgs/msg/PointCloud2 || errors=$((errors + 1))
    check_topic /velodyne_points sensor_msgs/msg/PointCloud2 || errors=$((errors + 1))
    check_topic /imu sensor_msgs/msg/Imu || errors=$((errors + 1))
    check_topic /theta/dual_fisheye/image_raw/compressed sensor_msgs/msg/CompressedImage || errors=$((errors + 1))
    check_topic /tf tf2_msgs/msg/TFMessage || errors=$((errors + 1))
    check_topic /tf_static tf2_msgs/msg/TFMessage || errors=$((errors + 1))
    if [ "$REQUIRE_CLOCK" = true ]; then
        check_topic /clock rosgraph_msgs/msg/Clock || errors=$((errors + 1))
    fi
    echo "  ※ $SOURCE_HINT"
    if [ "$errors" -ne 0 ]; then
        echo "✗ $errors 件の問題があるため録画を中止します。"
        return 1
    fi
    return 0
}

echo "================================================="
echo "  色付き3D地図用 録画ツール [$MODE]"
echo "================================================="
if ! preflight; then
    read -r -p "必要な情報が揃っていません。check_3dmap_ready で確認後、Enterで閉じます..." _
    exit 1
fi

read -r -p "ファイル名 [theta_3dmap_$(date +%Y%m%d_%H%M%S)]: " BAG_NAME
BAG_NAME=${BAG_NAME:-theta_3dmap_$(date +%Y%m%d_%H%M%S)}
if [[ ! "$BAG_NAME" =~ ^[A-Za-z0-9._-]+$ ]]; then
    echo "エラー: ファイル名には英数字、.-_ のみ使用できます。"
    exit 1
fi
CURRENT_BAG_PATH="${ROSBAG_DIR}/${BAG_NAME}"
if [ -e "$CURRENT_BAG_PATH" ]; then
    echo "エラー: 既に存在します（上書きしません）: $CURRENT_BAG_PATH"
    exit 1
fi
read -r -p "実験メモ [theta colored 3D map]: " MEMO
MEMO=${MEMO:-theta colored 3D map}

RECORD_TOPICS=(
    /velodyne_points_colored
    /velodyne_points
    /imu
    /magnetometer
    /theta/dual_fisheye/image_raw/compressed
    /tf /tf_static
    /odom /odom/filtered
    /experiment_metadata
)
if [ "$REQUIRE_CLOCK" = true ]; then
    RECORD_TOPICS+=( /clock )
fi

echo ""
echo "録画開始: $CURRENT_BAG_PATH"
echo "  色付き点群+生点群+IMU+THETA+TF+odom を録画します（オフラインSLAM対応）"
echo "停止は Ctrl+C（1回）。"
ros2 bag record -s mcap -o "$CURRENT_BAG_PATH" "${RECORD_TOPICS[@]}" &
ROSBAG_PID=$!
sleep 1
ros2 topic pub --once /experiment_metadata std_msgs/msg/String "data: '$MEMO'" >/dev/null 2>&1 &
wait "$ROSBAG_PID" || true
ROSBAG_PID=""
cleanup 0
