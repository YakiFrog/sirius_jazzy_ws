#!/bin/bash
# 好きなタイミングで実験メモを送信するスクリプト

if [ -z "$1" ]; then
    echo "使用方法: pub_memo \"送信したいメッセージ\""
    exit 1
fi

# ROS2 ワークスペースのセットアップ
if [ -f ~/sirius_jazzy_ws/install/setup.bash ]; then
    source ~/sirius_jazzy_ws/install/setup.bash
fi

# トピックのパブリッシュ
ros2 topic pub --once /experiment_metadata std_msgs/msg/String "data: '$1'"
