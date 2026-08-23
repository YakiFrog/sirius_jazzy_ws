#!/usr/bin/env bash
# Real-robot recording for offline SAM3 mapping. Does not auto-start dependencies.

WS_DIR="${HOME}/sirius_jazzy_ws"
ROSBAG_DIR="${HOME}/rosbag2_data"
CHECK_SCRIPT="${WS_DIR}/bash/startup_bash/check_offline_real_ready.sh"
ROSBAG_PID=""
CURRENT_BAG_PATH=""
RESULT_SHOWN=false

if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
else
    source /opt/ros/jazzy/setup.bash
fi
set -u
mkdir -p "$ROSBAG_DIR"

show_result() {
    if [ "$RESULT_SHOWN" = true ]; then
        return
    fi
    RESULT_SHOWN=true
    echo ""
    echo "================================================="
    if [ -d "$CURRENT_BAG_PATH" ]; then
        echo "✓ 実機録画を保存しました: $CURRENT_BAG_PATH"
        du -sh "$CURRENT_BAG_PATH"
        ros2 bag info "$CURRENT_BAG_PATH" 2>/dev/null | grep -E \
            "Topic: /camera/stereo_sbs/compressed|Topic: /scan3|Topic: /tf " || true
    else
        echo "録画データは作成されませんでした。"
    fi
    echo "================================================="
}

cleanup() {
    if [ -n "$ROSBAG_PID" ]; then
        echo ""
        echo "録画を停止・保存しています..."
        kill -INT "$ROSBAG_PID" 2>/dev/null || true
        wait "$ROSBAG_PID" 2>/dev/null || true
        ROSBAG_PID=""
    fi
    show_result
}

trap 'cleanup; exit 0' INT TERM HUP

echo "================================================="
echo "  実機 SAM3オフラインマッピング録画"
echo "================================================="
echo "このスクリプトはセンサーやSLAM Toolboxを自動起動しません。"
echo ""

if ! bash "$CHECK_SCRIPT"; then
    echo ""
    echo "事前チェックに失敗したため録画を中止しました。"
    exit 1
fi

echo ""
read -r -p "ファイル名 [offline_real_$(date +%Y%m%d_%H%M%S)]: " BAG_NAME
BAG_NAME=${BAG_NAME:-offline_real_$(date +%Y%m%d_%H%M%S)}
if [[ ! "$BAG_NAME" =~ ^[A-Za-z0-9._-]+$ ]]; then
    echo "エラー: ファイル名には英数字、.-_ のみ使用できます。"
    exit 1
fi

CURRENT_BAG_PATH="${ROSBAG_DIR}/${BAG_NAME}"
if [ -e "$CURRENT_BAG_PATH" ]; then
    echo "エラー: 既に存在します。既存データ保護のため上書きしません: $CURRENT_BAG_PATH"
    exit 1
fi

read -r -p "実験メモ [Real robot offline semantic mapping]: " EXPERIMENT_MEMO
EXPERIMENT_MEMO=${EXPERIMENT_MEMO:-Real robot offline semantic mapping}

echo ""
echo "録画を開始します。停止は Ctrl+C です。"
echo "保存先: $CURRENT_BAG_PATH"

ros2 bag record -s mcap -o "$CURRENT_BAG_PATH" \
    /camera/stereo_sbs/compressed \
    /camera/stereo_params \
    /tf \
    /tf_static \
    /odom \
    /odom/filtered \
    /roboteq/odom \
    /scan3 \
    /imu \
    /magnetometer \
    /joint_states \
    /cmd_vel \
    /cmd_vel_nav \
    /map \
    /experiment_metadata &
ROSBAG_PID=$!

sleep 1
ros2 topic pub --once /experiment_metadata std_msgs/msg/String \
    "data: '$EXPERIMENT_MEMO'" >/dev/null 2>&1 || true

wait "$ROSBAG_PID" || true
ROSBAG_PID=""
show_result
