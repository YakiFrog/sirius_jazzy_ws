#!/usr/bin/env bash
# Validate every essential input before starting a real-robot offline-mapping bag.

WS_DIR="${HOME}/sirius_jazzy_ws"
if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
else
    source /opt/ros/jazzy/setup.bash
fi
set -u

ERRORS=0

pass() {
    echo "  ✓ $1"
}

fail() {
    echo "  ✗ $1"
    ERRORS=$((ERRORS + 1))
}

topic_type() {
    ros2 topic type "$1" 2>/dev/null | head -n 1
}

check_topic() {
    local topic="$1"
    local expected_type="$2"
    local actual_type
    actual_type=$(topic_type "$topic")
    if [ "$actual_type" = "$expected_type" ]; then
        pass "$topic ($expected_type)"
        return 0
    elif [ -z "$actual_type" ]; then
        fail "$topic にPublisherがありません"
    else
        fail "$topic の型が不正です: $actual_type (期待: $expected_type)"
    fi
    return 1
}

check_tf() {
    local parent="$1"
    local child="$2"
    local label="$3"
    local output_file
    output_file=$(mktemp "/tmp/sirius_tf_check.XXXXXX")
    timeout 5s ros2 run tf2_ros tf2_echo "$parent" "$child" >"$output_file" 2>&1 || true
    if grep -q "Translation:" "$output_file"; then
        pass "$label ($parent → $child)"
    else
        fail "$label が取得できません ($parent → $child)"
    fi
    gio trash "$output_file" 2>/dev/null || unlink "$output_file"
}

echo "================================================="
echo "  実機オフラインマッピング 録画前チェック"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
echo "================================================="

CAMERA_READY=false
TF_READY=false
if check_topic /camera/stereo_sbs/compressed sensor_msgs/msg/CompressedImage; then
    CAMERA_READY=true
fi
check_topic /camera/stereo_params std_msgs/msg/String
check_topic /scan3 sensor_msgs/msg/LaserScan
check_topic /odom/filtered nav_msgs/msg/Odometry
if check_topic /tf tf2_msgs/msg/TFMessage; then
    TF_READY=true
fi
check_topic /tf_static tf2_msgs/msg/TFMessage

if [ "$TF_READY" = true ]; then
    check_tf map sirius3/base_footprint "SLAM Toolbox補正済みTF"
    check_tf sirius3/base_footprint sirius3/zed_camera_link "ZED取付TF"
fi

if [ "$CAMERA_READY" = true ]; then
    echo ""
    echo "カメラ受信レートを測定中 (約6秒)..."
    RATE_OUTPUT=$(timeout 6s ros2 topic hz /camera/stereo_sbs/compressed 2>/dev/null || true)
    RATE_LINE=$(printf '%s\n' "$RATE_OUTPUT" | grep "average rate:" | tail -n 1)
    if [ -n "$RATE_LINE" ]; then
        pass "カメラ $RATE_LINE"
    else
        fail "カメラ画像が継続受信できません"
    fi
fi

FREE_GB=$(df -BG "${HOME}" | awk 'NR==2 {gsub(/G/, "", $4); print $4}')
if [ -n "$FREE_GB" ] && [ "$FREE_GB" -ge 10 ]; then
    pass "空き容量 ${FREE_GB} GB"
else
    fail "空き容量が10 GB未満です (${FREE_GB:-不明} GB)"
fi

echo "================================================="
if [ "$ERRORS" -eq 0 ]; then
    echo "✓ 録画準備完了"
    exit 0
fi

echo "✗ $ERRORS 項目に問題があります。録画は開始しないでください。"
exit 1
