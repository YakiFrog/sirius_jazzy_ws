#!/bin/bash
# ==============================================================================
# オフラインSAM3セマンティックマッピング専用 ROS2 bag 記録スクリプト
# Unityシミュレーションおよび実機からのステレオ映像・SLAMデータを同期録画します
# ==============================================================================

WS_DIR="${HOME}/sirius_jazzy_ws"
ROSBAG_DIR="${HOME}/rosbag2_data"
mkdir -p "$ROSBAG_DIR"

source "$WS_DIR/install/setup.bash" 2>/dev/null || source /opt/ros/jazzy/setup.bash

ROSBAG_PID=""
BRIDGE_PID=""
CURRENT_BAG_PATH=""
RESULT_SHOWN=false
experiment_memo="Offline Semantic Mapping"

cleanup() {
    echo ""
    echo "========================================="
    echo "記録を停止・保存しています..."
    if [ -n "$ROSBAG_PID" ]; then
        kill -INT $ROSBAG_PID 2>/dev/null
        wait $ROSBAG_PID 2>/dev/null
    fi
    if [ -n "$BRIDGE_PID" ]; then
        kill -INT $BRIDGE_PID 2>/dev/null
    fi
    show_result
    exit 0
}

trap cleanup INT TERM HUP

# 未インデックス対策 & 結果表示
show_result() {
    if [ "$RESULT_SHOWN" = true ]; then
        return
    fi
    RESULT_SHOWN=true
    
    echo ""
    echo "========================================="
    echo "✓ 録画が完了しました"
    echo "  保存先: $CURRENT_BAG_PATH"
    
    if [ -d "$CURRENT_BAG_PATH" ]; then
        local mcap_file=$(find "$CURRENT_BAG_PATH" -name "*.mcap" | head -n 1)
        if [ -n "$mcap_file" ] && [ -f "$mcap_file" ]; then
            if [ -f "${WS_DIR}/mcap" ]; then
                echo "  [MCAP] インデックスの整合性を自動チェック・修復中..."
                local tmp_mcap="${mcap_file}.tmp"
                if "${WS_DIR}/mcap" recover "$mcap_file" -o "$tmp_mcap" >/dev/null 2>&1; then
                    mv "$tmp_mcap" "$mcap_file"
                    echo "  [MCAP] ✓ インデックス処理完了"
                else
                    rm -f "$tmp_mcap"
                fi
            fi
        fi
        if [ ! -f "$CURRENT_BAG_PATH/metadata.yaml" ]; then
            echo "  [Rosbag] metadata.yaml を生成・インデックス登録中..."
            ros2 bag reindex "$CURRENT_BAG_PATH" -s mcap >/dev/null 2>&1
            echo "  [Rosbag] ✓ メタデータ生成完了"
        fi
        size=$(du -sh "$CURRENT_BAG_PATH" | cut -f1)
        echo "  データサイズ: $size"
    fi
    echo "========================================="
}

# 1. Unity Stereo Bridge (HTTP -> ROS2 CompressedImage) をバックグラウンド起動
echo "========================================="
echo "  オフライン・セマンティックマッピング 録画ツール"
echo "========================================="
echo "※ SLAM Toolboxは自動起動しません。録画前にLauncherから slamtoolbox を起動してください。"
echo "Unity画像レシーバー (unity_stereo_bridge: port 8080) を起動中..."
ros2 run sirius_navigation unity_stereo_bridge >/dev/null 2>&1 &
BRIDGE_PID=$!
sleep 1

input_filename() {
    echo ""
    echo "記録対象トピック:"
    echo "  - /camera/stereo_sbs/compressed (Unity ステレオSBS圧縮画像)"
    echo "  - /camera/stereo_params (カメラ内部パラメータ)"
    echo "  - /tf, /tf_static (座標変換)"
    echo "  - /odom, /odom/filtered (オドメトリ)"
    echo "  - /scan3 (2D LiDAR)"
    echo "  - /clock, /imu, /magnetometer"
    echo ""
    echo "========================================="
    echo -n "ファイル名を入力してください (例: sim_campus_01): "
    read bag_name
    
    if [ -z "$bag_name" ]; then
        bag_name="offline_mapping_$(date +%Y%m%d_%H%M%S)"
        echo "デフォルト名を使用: $bag_name"
    fi

    echo -n "実験内容のメモを入力してください: "
    read input_memo
    if [ -n "$input_memo" ]; then
        experiment_memo="$input_memo"
    fi
    
    if [ -d "$ROSBAG_DIR/$bag_name" ]; then
        echo "警告: $bag_name は既に存在します"
        echo -n "上書きしますか? (y/N): "
        read overwrite
        if [[ ! "$overwrite" =~ ^[Yy]$ ]]; then
            echo "キャンセルしました"
            return 1
        fi
        rm -rf "$ROSBAG_DIR/$bag_name"
    fi
    
    selected_name="$bag_name"
}

start_recording() {
    CURRENT_BAG_PATH="$ROSBAG_DIR/$1"
    RESULT_SHOWN=false
    
    echo "========================================="
    echo "記録を開始します..."
    echo "  保存先: $CURRENT_BAG_PATH"
    echo "  停止するには Ctrl+C を押してください"
    echo "========================================="
    echo ""
    
    ros2 topic pub --once /experiment_metadata std_msgs/msg/String "data: '$experiment_memo'" >/dev/null 2>&1 &

    ros2 bag record -s mcap -o "$CURRENT_BAG_PATH" \
        /camera/stereo_sbs/compressed \
        /camera/stereo_params \
        /tf \
        /tf_static \
        /odom \
        /odom/filtered \
        /scan3 \
        /clock \
        /imu \
        /magnetometer \
        /cmd_vel \
        /cmd_vel_nav \
        /experiment_metadata &
    
    ROSBAG_PID=$!
    wait $ROSBAG_PID
}

input_filename
if [ $? -eq 0 ] && [ -n "$selected_name" ]; then
    start_recording "$selected_name"
fi

cleanup
