#!/bin/bash
# Sirius Assisted Teleop テストスクリプト

echo "========================================"
echo "Sirius Assisted Teleop テスト"
echo "========================================"
echo ""

# ワークスペースをソース
echo "1. ワークスペースをソース中..."
source /home/kotantu-desktop/sirius_jazzy_ws/install/setup.bash
echo "   ✓ 完了"
echo ""

# 実行可能ファイルが存在するか確認
echo "2. 実行可能ファイルを確認中..."
if command -v ros2 &> /dev/null; then
    if ros2 pkg executables sirius_navigation | grep -q assisted_teleop; then
        echo "   ✓ assisted_teleop が見つかりました"
    else
        echo "   ✗ assisted_teleop が見つかりません"
        echo "   パッケージをビルドしてください: colcon build --packages-select sirius_navigation"
        exit 1
    fi
else
    echo "   ✗ ROS2が見つかりません"
    exit 1
fi
echo ""

# 使用方法を表示
echo "========================================"
echo "使用方法"
echo "========================================"
echo ""
echo "【前提条件】"
echo "以下が起動している必要があります:"
echo "  1. シミュレーション環境"
echo "     ros2 launch sirius_description sim.launch.py"
echo ""
echo "  2. Nav2スタック"
echo "     ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true"
echo ""
echo "【起動方法】"
echo ""
echo "方法1: Launchファイル使用（推奨）"
echo "  ros2 launch sirius_navigation assisted_teleop.launch.py"
echo ""
echo "方法2: 個別起動"
echo "  ターミナル1: ros2 run sirius_navigation assisted_teleop"
echo "  ターミナル2: ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo "【キーボード操作】"
echo "  u i o    : 前進方向"
echo "  j k l    : 回転"
echo "  m , .    : 後退方向"
echo "  q/z      : 線速度増減"
echo "  w/x      : 角速度増減"
echo "  space    : 緊急停止"
echo ""
echo "========================================"
echo ""

# 起動確認
read -p "Assisted Teleopを起動しますか？ (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo ""
    echo "Assisted Teleopを起動します..."
    echo "終了するには Ctrl+C を押してください"
    echo ""
    sleep 2
    ros2 launch sirius_navigation assisted_teleop.launch.py
fi
