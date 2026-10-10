#!/bin/bash
# ==============================================================================
# 色付き3D地図の録画に必要な情報が揃っているか検査する。
#   - 必須トピック（型・配信レート）
#   - TF（base_footprint->lidar_link, odom->base_footprint, lidar_link->theta_link）
#   - /tf_static のPublisher有無（robot_state_publisher / rviz2real が要る）
# 使い方: check_theta_3dmap_ready.sh
# ==============================================================================
set -u
WS_DIR="${HOME}/sirius_jazzy_ws"
set +u
if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
else
    source /opt/ros/jazzy/setup.bash
fi
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-53}"

OK=0; NG=0
pass() { echo "  ✓ $1"; OK=$((OK + 1)); }
fail() { echo "  ✗ $1"; NG=$((NG + 1)); }
warn() { echo "  ▲ $1"; }

check_topic() {
    local topic="$1" type="$2" required="${3:-yes}"
    local actual
    actual=$(timeout 6 ros2 topic type "$topic" 2>/dev/null | head -n1)
    if [ "$actual" = "$type" ]; then
        pass "$topic ($type)"
    elif [ -z "$actual" ]; then
        if [ "$required" = "yes" ]; then fail "$topic がありません"; else warn "$topic がありません（任意）"; fi
    else
        fail "$topic の型が不一致: $actual"
    fi
}

check_rate() {
    local topic="$1"
    local out
    out=$(timeout 7 ros2 topic hz "$topic" 2>/dev/null | grep "average rate:" | tail -n1)
    if [ -n "$out" ]; then
        pass "$topic レート $out"
    else
        fail "$topic のメッセージを受信できません"
    fi
}

check_tf() {
    local parent="$1" child="$2"
    local out
    out=$(timeout 6 ros2 run tf2_ros tf2_echo "$parent" "$child" 2>&1 | grep -m1 "Translation")
    if [ -n "$out" ]; then
        pass "TF $parent -> $child : ${out#*- }"
    else
        fail "TF $parent -> $child が取得できません"
    fi
}

echo "================================================="
echo " 色付き3D地図 録画前チェック  (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
echo "================================================="
echo "[必須トピック]"
check_topic /velodyne_points sensor_msgs/msg/PointCloud2
check_topic /imu sensor_msgs/msg/Imu
check_topic /tf tf2_msgs/msg/TFMessage
check_topic /tf_static tf2_msgs/msg/TFMessage
check_topic /odom/filtered nav_msgs/msg/Odometry no
check_topic /odom nav_msgs/msg/Odometry no
check_topic /theta/dual_fisheye/image_raw/compressed sensor_msgs/msg/CompressedImage no
check_topic /velodyne_points_colored sensor_msgs/msg/PointCloud2 no

echo "[配信レート]"
check_rate /velodyne_points
check_rate /velodyne_points_colored || true

echo "[TF]"
check_tf sirius3/base_footprint sirius3/lidar_link
check_tf sirius3/lidar_link sirius3/theta_link
check_tf sirius3/odom sirius3/base_footprint

echo "================================================="
if [ "$NG" -eq 0 ]; then
    echo "✓ 必須項目OK。録画できます（record_3dmap）。"
else
    echo "✗ $NG 件がNGです。上記を解消してから録画してください。"
    echo "  ヒント: rviz2real(robot_state_publisher) / theta_capture / theta_colorize が起動しているか確認。"
fi
echo "================================================="
read -r -p "Enterで閉じます..." _
exit $([ "$NG" -eq 0 ] && echo 0 || echo 1)
