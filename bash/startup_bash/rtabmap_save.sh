#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
cd ~/sirius_jazzy_ws
while : ;do
    echo "------------------------------------------------"
    echo "RTAB-Map 保存スクリプト"
    echo "------------------------------------------------"
    read -p "Press [Enter] key to start RTAB-Map save..."
    
    # ROS環境の読み込み
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
    else
        source /opt/ros/jazzy/setup.bash
    fi

    echo "保存するマップ名を入力してください (例: my_map): "
    read map_name
    
    if [ -z "$map_name" ]; then
        echo "エラー: マップ名が空です。"
        continue
    fi

    MAP_DIR="$HOME/sirius_jazzy_ws/maps_waypoints/maps"
    mkdir -p "$MAP_DIR"

    # 1. 2Dグリッドマップの保存 (/rtabmap/grid_map を指定)
    echo "[1/2] 2Dグリッドマップを保存中..."
    ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/rtabmap_$map_name" --ros-args -r map:=/rtabmap/grid_map
    
    # 2. 3D点群マップの書き出し
    echo "[2/2] 3D点群マップ (PLY) をデータベースから書き出し中..."
    DB_PATH="$HOME/.ros/rtabmap.db"
    
    if [ -f "$DB_PATH" ]; then
        # rtabmap-export を使用してデータベースから PLY を作成
        # --opt 2: すでにDBにある最適化済みポーズを使用（稼働中の競合回避）
        # --max_range 0: 距離制限を無効化
        # --decimation 1: 全ての点を出力
        rtabmap-export --ply --cloud --scan --opt 2 --max_range 0 --decimation 1 --uinfo "$DB_PATH"
        
        if [ -f "cloud.ply" ]; then
            mv cloud.ply "$MAP_DIR/rtabmap_$map_name.ply"
            echo "SUCCESS: 3Dマップを保存しました: $MAP_DIR/rtabmap_$map_name.ply"
        else
            echo "WARNING: 3Dマップの書き出しに失敗しました（cloud.ply が生成されませんでした）。"
        fi
    else
        echo "ERROR: RTAB-Map データベースが見つかりません: $DB_PATH"
    fi
    
    echo "------------------------------------------------"
    echo "保存プロセスが完了しました。"
done
