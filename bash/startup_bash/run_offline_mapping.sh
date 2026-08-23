#!/bin/bash
# ==============================================================================
# オフライン SAM3 + RTAB-Map セマンティックマッピング 実行・保存スクリプト
# 記録済み Rosbag を再生し、GPU で SAM3 推論 ＋ 2D カラー地図を生成・保存します
# ==============================================================================

WS_DIR="${HOME}/sirius_jazzy_ws"
ROSBAG_DIR="${HOME}/rosbag2_data"
SAM3_SERVER_DIR="${HOME}/sam3_zed_server"

# Set domain isolation to prevent collisions with running Unity simulation
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}

source "$WS_DIR/install/setup.bash" 2>/dev/null || source /opt/ros/jazzy/setup.bash

echo "================================================="
echo "  SAM3 オフライン・セマンティックマッピング (Domain: $ROS_DOMAIN_ID)"
echo "================================================="

# 0. SAM3 GPU サーバー (Docker) の状態確認
if [ -d "$SAM3_SERVER_DIR" ]; then
    echo "SAM3 GPU サーバー (Docker: port 8080) の状態を確認中..."
    if ! docker ps | grep -q "sam3_zed_container"; then
        echo "SAM3 コンテナが起動していません。起動します..."
        docker start sam3_zed_container 2>/dev/null || (cd "$SAM3_SERVER_DIR" && docker compose up -d sam3-zed-merged)
    else
        echo "✓ SAM3 GPU サーバー (sam3_zed_container) は稼働中です。"
    fi

    echo -n "SAM3 サーバーの準備完了を待機中..."
    READY=false
    for i in {1..30}; do
        if curl -s -m 1 http://localhost:8080/ >/dev/null 2>&1; then
            READY=true
            break
        fi
        echo -n "."
        sleep 1
    done
    echo ""
    if [ "$READY" = true ]; then
        echo "✓ SAM3 サーバーの準備が完了しました！"
    else
        echo "⚠️ 警告: SAM3 サーバーの起動待機がタイムアウトしました。ログを確認してください。"
    fi
fi

# 1. 録画データ (Rosbag) の選択
BAG_LIST=($(find "$ROSBAG_DIR" -maxdepth 1 -mindepth 1 -type d | sort -r))

if [ ${#BAG_LIST[@]} -eq 0 ]; then
    echo ""
    echo "エラー: $ROSBAG_DIR に Rosbag データが見つかりません。"
    echo "先に ./record_rosbag_offline.sh で走行データを録画してください。"
    exit 1
fi

echo ""
echo "利用可能な Rosbag 一覧:"
for i in "${!BAG_LIST[@]}"; do
    bag_base=$(basename "${BAG_LIST[$i]}")
    echo "  [$((i+1))] $bag_base"
done

echo ""
read -p "使用する Rosbag 番号を選択してください [1]: " choice
choice=${choice:-1}
index=$((choice-1))

if [ $index -lt 0 ] || [ $index -ge ${#BAG_LIST[@]} ]; then
    echo "無効な選択です。"
    exit 1
fi

SELECTED_BAG="${BAG_LIST[$index]}"
BAG_NAME=$(basename "$SELECTED_BAG")
echo "選択された Rosbag: $SELECTED_BAG"

# 2. 再生速度の選択
echo ""
read -p "再生速度を選択してください (例: 0.5, 1.0) [0.5]: " PLAY_RATE
PLAY_RATE=${PLAY_RATE:-0.5}

# 3. プロンプト（クラス定義）の設定
echo ""
DEFAULT_PROMPT="grass, tactile paving, roadway, sidewalk"
read -p "SAM3 認識プロンプト (カンマ区切り) [$DEFAULT_PROMPT]: " PROMPT_INPUT
PROMPT_INPUT=${PROMPT_INPUT:-$DEFAULT_PROMPT}

# サーバー側プロンプトをHTTP POSTで更新
if curl -s -X POST http://localhost:8080/prompt -H "Content-Type: application/json" -d "{\"prompt\": \"$PROMPT_INPUT\"}" >/dev/null 2>&1; then
    echo "✓ SAM3 サーバーのプロンプトを更新しました: $PROMPT_INPUT"
fi

# 4. RViz2 確認画面の起動選択
echo ""
read -p "RViz2 確認画面（リアルタイム描画）を自動起動しますか？ (Y/n) [Y]: " RVIZ_CHOICE
RVIZ_CHOICE=${RVIZ_CHOICE:-y}
RVIZ_CHOICE=$(echo "$RVIZ_CHOICE" | tr '[:upper:]' '[:lower:]')
USE_RVIZ_FLAG="false"
if [ "$RVIZ_CHOICE" != "n" ] && [ "$RVIZ_CHOICE" != "no" ]; then
    USE_RVIZ_FLAG="true"
fi

echo "================================================="
echo "マッピングパイプラインを起動しています..."
echo "  Rosbag: $BAG_NAME"
echo "  再生速度: ${PLAY_RATE}x"
echo "  プロンプト: $PROMPT_INPUT"
echo "  RViz2 表示: $USE_RVIZ_FLAG"
echo "  SLAM Toolbox: 起動しない (bag内の補正済みTFを使用)"
echo "================================================="

# Launch mapping nodes in background
ros2 launch sirius_navigation sam3_offline_mapping.launch.py \
    use_sim_time:=true \
    run_slam_toolbox:=false \
    include_background:=true \
    use_docker_backend:=true \
    prompt:="$PROMPT_INPUT" \
    rviz:="$USE_RVIZ_FLAG" &
LAUNCH_PID=$!

sleep 5

cleanup() {
    echo ""
    echo "マッピングノードを停止しています..."
    kill -INT $LAUNCH_PID 2>/dev/null
    wait $LAUNCH_PID 2>/dev/null
    exit 0
}
trap cleanup INT TERM

echo ""
echo "================================================="
echo "Rosbag 再生を開始します..."
echo "================================================="
if [ ! -f "$SELECTED_BAG/metadata.yaml" ]; then
    echo "Rosbag メタデータ (metadata.yaml) を生成・再インデックス中..."
    ros2 bag reindex "$SELECTED_BAG" -s mcap
fi

if ros2 bag info "$SELECTED_BAG" 2>/dev/null | grep -q "Topic: /clock"; then
    echo "✓ 録画データ内の /clock を使用して再生します"
    ros2 bag play "$SELECTED_BAG" --rate "$PLAY_RATE"
else
    echo "✓ --clock オプションを有効にして再生します"
    ros2 bag play "$SELECTED_BAG" --clock --rate "$PLAY_RATE"
fi

echo ""
echo "================================================="
echo "✓ Rosbag の再生が完了しました！"
echo "================================================="
echo ""
read -p "生成されたセマンティック地図を保存しますか？ (Y/n): " save_choice
save_choice=$(echo "$save_choice" | tr '[:upper:]' '[:lower:]')

if [ "$save_choice" != "n" ] && [ "$save_choice" != "no" ]; then
    MAP_SAVE_SCRIPT="$WS_DIR/bash/startup_bash/rtabmap_save.sh"
    if [ -f "$MAP_SAVE_SCRIPT" ]; then
        echo "地図保存スクリプトを実行中..."
        bash "$MAP_SAVE_SCRIPT"
    else
        echo "保存先ディレクトリ: $WS_DIR/maps_waypoints"
        mkdir -p "$WS_DIR/maps_waypoints"
        ros2 run nav2_map_server map_saver_cli -f "$WS_DIR/maps_waypoints/offline_semantic_${BAG_NAME}" --ros-args -p map_topic:=/sam3/colored_map_grid
        echo "✓ 地図を保存しました: $WS_DIR/maps_waypoints/offline_semantic_${BAG_NAME}"
    fi
fi

cleanup
