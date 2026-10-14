#!/bin/bash
# 3D地図生成・自律移動スタック（small_gicp + FAST-LIO2）の導入スクリプト。
set -u
WS="$HOME/sirius_jazzy_ws"

set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "$WS" || exit 1

echo "==== [1/5] small_gicp (Python bindings) ===="
if python3 -c "import small_gicp" 2>/dev/null; then
    echo "導入済み"
else
    pip install --break-system-packages small_gicp || {
        echo "small_gicp の導入に失敗しました"
        exit 1
    }
fi

echo "==== [2/5] livox_ros_driver2 互換インターフェース ===="
if [ -f "$WS/src/livox_ros_driver2/package.xml" ]; then
    echo "OK (リポジトリ同梱のメッセージIFを使用)"
else
    echo "不足: $WS/src/livox_ros_driver2"
    exit 1
fi

echo "==== [3/5] FAST_LIO_ROS2 (submodule) ===="
if [ -d "$WS/src/FAST_LIO_ROS2" ] && [ -n "$(ls -A "$WS/src/FAST_LIO_ROS2" 2>/dev/null)" ]; then
    git -C "$WS/src/FAST_LIO_ROS2" submodule update --init --recursive || exit 1
else
    git -C "$WS" submodule update --init --recursive src/FAST_LIO_ROS2 || exit 1
fi

echo "==== [4/5] colcon build (3Dスタック) ===="
colcon build --symlink-install --packages-select livox_ros_driver2 fast_lio sirius_navigation || exit 1

echo "==== [5/5] diagnostic_updater ABIに合わせて依存パッケージを再ビルド ===="
# システムの diagnostic_updater 更新でコンストラクタABIが変わることがある。
# 旧版でビルド済みのパッケージは起動時に symbol lookup error になるため、
# ABI依存パッケージをクリーン再ビルドして整合させる（数十秒）。
ABI_PKGS="velodyne_driver velodyne_pointcloud velodyne_laserscan urg_node2 nav2_lifecycle_manager"
for p in $ABI_PKGS; do
    rm -rf "$WS/build/$p" "$WS/install/$p"
done
colcon build --symlink-install --packages-select $ABI_PKGS || exit 1

echo "完了。fast_lio / small_gicp / 3D自己位置推定 が使える状態になりました。"
