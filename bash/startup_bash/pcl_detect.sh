#!/bin/bash
# PCL Object Detection Node Startup Script
# -------------------------------------------------------------
trap 'echo ""; echo "PCL Object Detection を終了しました。"' SIGINT

cd ~/sirius_jazzy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "=========================================================="
echo "  PCL Object Detection (点群ベースの3D物体検出・クラスタリング)"
echo "=========================================================="
echo "このノードは、入力された3D点群(PointCloud2)を距離情報に基づいて"
echo "クラスタ(最大6個)に分割し、それぞれの点群をトピック(cluster_0〜5)に"
echo "出力するほか、Rviz2で可視化するための境界ボックス(viz)を出力します。"
echo "----------------------------------------------------------"
echo "入力となる点群トピックを選択してください:"
echo "  [1] /velodyne_points (LiDAR 3D点群 - 推奨)"
echo "  [2] /sam3/obstacles   (SAM3 障害物点群)"
echo "  [3] カスタムトピック名を入力する"
echo "=========================================================="
echo -n "番号を入力してください (1-3): "
read choice

INPUT_TOPIC="/velodyne_points"

if [ "$choice" = "2" ]; then
    INPUT_TOPIC="/sam3/obstacles"
elif [ "$choice" = "3" ]; then
    echo -n "トピック名を入力してください (例: /my_points): "
    read custom_topic
    if [ ! -z "$custom_topic" ]; then
        INPUT_TOPIC="$custom_topic"
    fi
fi

echo "----------------------------------------------------------"
echo "起動中..."
echo "  入力トピック: $INPUT_TOPIC"
echo "  出力バウンディングボックス: /viz (MarkerArray)"
echo "  出力個別クラスタ: /cluster_0 〜 /cluster_5"
echo "  (Rviz2で確認可能です)"
echo "----------------------------------------------------------"

ros2 run pcl_object_detection pcl_object_detection_node --ros-args --remap filtered_clouds:=$INPUT_TOPIC
