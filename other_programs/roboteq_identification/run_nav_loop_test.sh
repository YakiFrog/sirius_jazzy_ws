#!/usr/bin/env bash
# 同コース(15F-0925)自律走行テスト + rosbag + 安全監視
# 使い方: ./run_nav_loop_test.sh [mode] [waypoints]
#   mode: normal(既定)/fast/safe/...  waypoints: 15F-0925(既定)
set -eo pipefail

WS="$HOME/sirius_jazzy_ws"
MODE="${1:-normal}"
WP="${2:-15F-0925}"
STAMP="$(date +%Y%m%d_%H%M%S)"

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
cd "$WS"

# 走行モード適用
bash bash/startup_bash/change_nav_mode.sh "$MODE" >/dev/null 2>&1 || true

echo "mode=$MODE waypoints=$WP"
echo "closed_loop=$(ros2 param get /roboteq_ros2_driver closed_loop 2>/dev/null | tail -1)"
echo "vx_max=$(ros2 param get /controller_server FollowPath.vx_max 2>/dev/null | tail -1)"
echo "imu_rate:"; timeout 5 ros2 topic hz /imu 2>/dev/null | grep average | head -1 || true

BAG="$HOME/rosbag2_data/nav_${MODE}_${STAMP}"
echo "BAG=$BAG"

python3 other_programs/roboteq_identification/safety_monitor.py > "/tmp/safety_${STAMP}.log" 2>&1 &
SAFE=$!
ros2 bag record -o "$BAG" \
  /cmd_vel /cmd_vel_nav /cmd_vel_smoothed /odom /odom/filtered /imu /magnetometer \
  /roboteq/status /amcl_pose /scan3 /tf /tf_static /plan /stop \
  > "/tmp/bag_${STAMP}.log" 2>&1 &
BAGREC=$!
sleep 4

ros2 run sirius_navigation move_goal --waypoints "$WP" --count 1 --threshold 2.0 || true

kill -INT "$BAGREC" 2>/dev/null || true
kill -9 "$SAFE" 2>/dev/null || true
wait "$BAGREC" 2>/dev/null || true
echo "done. bag=$BAG"
echo "FF2 events: $(grep -c 'FF=2' "/tmp/safety_${STAMP}.log" 2>/dev/null || echo 0)"
echo "解析: python3 other_programs/roboteq_identification/analyze_nav_bag.py $BAG"
