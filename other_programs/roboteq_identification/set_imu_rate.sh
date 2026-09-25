#!/usr/bin/env bash
# IMU(HWT905)の出力周波数を変更する（witmotionノードを停止→設定→再起動）
# 使い方: ./set_imu_rate.sh [hz]   hz: 10/20(既定)/50/100/125/200
set -eo pipefail

WS="$HOME/sirius_jazzy_ws"
HZ="${1:-20}"

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
cd "$WS"

echo "IMU rate -> ${HZ}Hz"
# witmotionノードを停止（パターンは自分自身に一致しないようブラケット記法）
pkill -9 -f 'witmotion_ros_nod[e]' || true
sleep 2

# 9軸を維持しつつ出力周波数を設定
python3 other_programs/roboteq_identification/imu_set_axis.py 9 --rate "$HZ"
sleep 1

# witmotionノード再起動（デタッチ）
setsid -f ros2 run witmotion_ros witmotion_ros_node --ros-args \
    -r __node:=witmotion --params-file params/wt905.yaml \
    > /tmp/witmotion.log 2>&1 < /dev/null
sleep 6

echo "--- /imu rate ---"
timeout 8 ros2 topic hz /imu 2>/dev/null | grep average | head -1 || echo "no /imu"
