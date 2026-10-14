#!/bin/bash
# 3D地図生成・自律移動スタック（small_gicp + FAST-LIO2）の導入スクリプト。
set -u
WS="$HOME/sirius_jazzy_ws"

source /opt/ros/jazzy/setup.bash
cd "$WS" || exit 1

echo "==== [1/4] small_gicp (Python bindings) ===="
if python3 -c "import small_gicp" 2>/dev/null; then
    echo "導入済み"
else
    pip install --break-system-packages small_gicp || {
        echo "small_gicp の導入に失敗しました"
        exit 1
    }
fi

echo "==== [2/4] livox_ros_driver2 互換インターフェース ===="
if [ -f "$WS/src/livox_ros_driver2/package.xml" ]; then
    echo "OK (リポジトリ同梱のメッセージIFを使用)"
else
    echo "不足: $WS/src/livox_ros_driver2"
    exit 1
fi

echo "==== [3/4] FAST_LIO_ROS2 (submodule) ===="
if [ -d "$WS/src/FAST_LIO_ROS2" ] && [ -n "$(ls -A "$WS/src/FAST_LIO_ROS2" 2>/dev/null)" ]; then
    git -C "$WS/src/FAST_LIO_ROS2" submodule update --init --recursive || exit 1
else
    git -C "$WS" submodule update --init --recursive src/FAST_LIO_ROS2 || exit 1
fi

echo "==== [4/4] colcon build ===="
colcon build --symlink-install --packages-select livox_ros_driver2 fast_lio sirius_navigation || exit 1

echo "完了。fast_lio / small_gicp が使える状態になりました。"
