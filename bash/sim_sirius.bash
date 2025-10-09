#!/bin/bash

# Sirius3ロボットシミュレーション起動スクリプト
# 使用方法: ./sim_sirius.bash または source sim_sirius.bash

echo "=== Sirius3 Robot Simulation Launcher ==="
echo "ワークスペースをセットアップしています..."

# ワークスペースのパスを設定（スクリプトの場所から自動検出）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_PATH="$(dirname "$SCRIPT_DIR")"

# ワークスペースディレクトリに移動
cd $WORKSPACE_PATH

# ROS2環境をセットアップ
echo "ROS2環境をセットアップしています..."
source /opt/ros/jazzy/setup.bash

# ワークスペースをビルド（必要に応じて）
echo "ワークスペースの状態をチェックしています..."
if [ ! -d "$WORKSPACE_PATH/install" ]; then
    echo "installディレクトリが見つかりません。ワークスペースをビルドしています..."
    colcon build --symlink-install
fi

# ワークスペースの環境をセットアップ
echo "ワークスペース環境をセットアップしています..."
source $WORKSPACE_PATH/install/setup.bash

# Gazebo環境変数を設定
echo "Gazebo環境変数を設定しています..."
export GZ_SIM_RESOURCE_PATH=$WORKSPACE_PATH/src/sirius_description:$GZ_SIM_RESOURCE_PATH

# シミュレーション起動
echo "=== Sirius3シミュレーションを起動しています ==="
echo "Ctrl+C で終了できます"
echo ""

# sim.launch.pyを起動
ros2 launch sirius_description sim.launch.py