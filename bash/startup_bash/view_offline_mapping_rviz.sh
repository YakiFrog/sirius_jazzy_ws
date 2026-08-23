#!/bin/bash
# ==============================================================================
# SAM3 オフラインマッピング リアルタイム確認用 RViz2 起動スクリプト
# ==============================================================================

WS_DIR="${HOME}/sirius_jazzy_ws"
cd "$WS_DIR"

# 実行中プロセスの ROS_DOMAIN_ID を自動検出（なければデフォルト 42）
DETECTED_DOMAIN=$(pgrep -a -f "sam3_offline_mapping.launch.py" | head -n 1 | grep -o "ROS_DOMAIN_ID=[0-9]*" | cut -d'=' -f2)
export ROS_DOMAIN_ID="${DETECTED_DOMAIN:-${ROS_DOMAIN_ID:-42}}"

source "$WS_DIR/install/setup.bash" 2>/dev/null || source /opt/ros/jazzy/setup.bash

RVIZ_CONFIG="$WS_DIR/src/sirius/sirius_navigation/rviz/sam3_offline_view.rviz"

echo "================================================="
echo "  SAM3 オフラインマッピング 確認用 RViz2"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "================================================="

if [ ! -f "$RVIZ_CONFIG" ]; then
    RVIZ_CONFIG="$WS_DIR/install/sirius_navigation/share/sirius_navigation/rviz/sam3_offline_view.rviz"
fi

echo "RViz2 を起動しています: $RVIZ_CONFIG"
ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true
