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

# 0. SAM3 GPU サーバー (Docker) をクリーンな状態で起動
if [ -d "$SAM3_SERVER_DIR" ]; then
    echo "SAM3 GPU サーバー (Docker: port 8080) を初期化中..."
    if docker container inspect sam3_zed_container >/dev/null 2>&1; then
        echo "前回の画像・点群・GPU状態を消去するため、SAM3コンテナを再起動します..."
        if ! docker restart sam3_zed_container >/dev/null; then
            echo "エラー: SAM3コンテナを再起動できませんでした。"
            exit 1
        fi
    else
        echo "SAM3コンテナを新規起動します..."
        if ! (cd "$SAM3_SERVER_DIR" && docker compose up -d sam3-zed-merged); then
            echo "エラー: SAM3コンテナを起動できませんでした。"
            exit 1
        fi
    fi

    echo -n "SAM3 サーバーの準備完了を待機中..."
    READY=false
    for i in {1..60}; do
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
        echo "エラー: SAM3 サーバーの起動待機がタイムアウトしました。"
        echo "docker logs sam3_zed_container を確認してください。"
        exit 1
    fi

    # A restarted backend must not contain a frame from an earlier Unity run.
    # Refuse to continue if another sender has already populated the buffer.
    if ! curl -fsS -m 2 http://localhost:8080/debug_state | python3 -c '
import json
import sys

state = json.load(sys.stdin)
raise SystemExit(1 if state.get("has_network_frame", False) else 0)
'; then
        echo "エラー: SAM3に再起動後のrosbag以外の画像入力を検出しました。"
        echo "Unityなど、port 8080へ画像を送信するプログラムを停止してください。"
        exit 1
    fi
    echo "✓ SAM3の前回フレームが消去されていることを確認しました。"
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

# カメラや補正済みTFが欠けたbagを再生すると、SAM3サーバーに残った古いフレームで
# 誤った地図を生成する可能性があるため、推論設定を変更する前に拒否します。
BAG_VALIDATOR="$WS_DIR/bash/startup_bash/validate_offline_mapping_bag.py"
if [ -f "$BAG_VALIDATOR" ]; then
    echo ""
    echo "Rosbagの必須情報を検証中..."
    VALIDATION_STATUS=0

    # metadata.yaml can still look healthy when the MCAP footer or final chunk
    # is incomplete. Check the complete MCAP structure before trusting topic
    # counts from metadata or starting a long offline mapping run.
    if [ -x "$WS_DIR/mcap" ]; then
        mapfile -d '' SELECTED_MCAP_FILES < <(
            find "$SELECTED_BAG" -maxdepth 1 -type f -name '*.mcap' -print0 | sort -z
        )
        if [ "${#SELECTED_MCAP_FILES[@]}" -gt 0 ]; then
            echo "MCAPファイル構造を確認中..."
            for mcap_file in "${SELECTED_MCAP_FILES[@]}"; do
                if ! "$WS_DIR/mcap" doctor "$mcap_file" >/dev/null 2>&1; then
                    echo "✗ MCAP破損または未完了を検出: $(basename "$mcap_file")"
                    VALIDATION_STATUS=2
                    break
                fi
            done
        fi
    fi

    if [ "$VALIDATION_STATUS" -eq 0 ]; then
        python3 "$BAG_VALIDATOR" "$SELECTED_BAG"
        VALIDATION_STATUS=$?
    fi

    # Exit code 2 means that rosbag2 could not open the storage. This commonly
    # happens when recording was interrupted before MCAP wrote its footer. In
    # that case only, recover into a new directory and validate it again. Exit
    # code 1 means readable data with missing inputs and must not be repaired.
    if [ "$VALIDATION_STATUS" -eq 2 ]; then
        RECOVERY_SCRIPT="$WS_DIR/bash/startup_bash/recover_mcap_rosbag.sh"
        if [ ! -f "$RECOVERY_SCRIPT" ]; then
            echo "エラー: MCAP自動復旧スクリプトがありません: $RECOVERY_SCRIPT"
            exit 1
        fi

        echo ""
        echo "Rosbagを開けないため、MCAP自動復旧を試します..."
        if ! RECOVERED_BAG=$(bash "$RECOVERY_SCRIPT" "$SELECTED_BAG"); then
            echo ""
            echo "エラー: MCAPを自動復旧できませんでした。元のRosbagは変更していません。"
            exit 1
        fi

        SELECTED_BAG="$RECOVERED_BAG"
        BAG_NAME=$(basename "$SELECTED_BAG")
        echo ""
        echo "復旧したRosbagを再検証中: $SELECTED_BAG"
        python3 "$BAG_VALIDATOR" "$SELECTED_BAG"
        VALIDATION_STATUS=$?
    fi

    if [ "$VALIDATION_STATUS" -ne 0 ]; then
        echo ""
        echo "エラー: 必須情報が不足しているため、このRosbagは再生しません。"
        echo "別のRosbagを選ぶか、録画機能で再録画してください。"
        exit 1
    fi
fi

# 2. 再生速度の選択
echo ""
read -p "再生速度を選択してください (例: 0.5, 1.0) [0.5]: " PLAY_RATE
PLAY_RATE=${PLAY_RATE:-0.5}

# 3. プロンプト（クラス定義）の設定
echo ""
DEFAULT_PROMPT="grass, tactile paving, roadway, sidewalk"
read -p "SAM3 認識プロンプト (カンマ区切り) [$DEFAULT_PROMPT]: " PROMPT_INPUT
PROMPT_INPUT=${PROMPT_INPUT:-$DEFAULT_PROMPT}

# オンライン実験と同じSAM3推論設定を明示的に適用します。入力元と深度方式は
# rosbag SBS画像用、点群色だけはテクスチャ保存用の実RGBへ切り替えます。
SAM3_PROMPT_JSON=$(python3 -c 'import json,sys; print(json.dumps({"prompt":sys.argv[1]}))' "$PROMPT_INPUT")
SAM3_SETTING_FAILURES=0

post_sam3_setting() {
    local endpoint="$1"
    local payload="$2"
    local label="$3"
    if curl -fsS -X POST "http://localhost:8080/${endpoint}" \
        -H "Content-Type: application/json" -d "$payload" >/dev/null; then
        echo "  ✓ $label"
    else
        echo "  ✗ $label の設定に失敗"
        SAM3_SETTING_FAILURES=$((SAM3_SETTING_FAILURES + 1))
    fi
}

echo "オンライン実験と同じSAM3推論設定を適用中（点群色のみ実RGB）..."
post_sam3_setting prompt "$SAM3_PROMPT_JSON" "prompt=$PROMPT_INPUT"
post_sam3_setting threshold '{"threshold":0.5}' "confidence=0.5"
post_sam3_setting sam3_resolution '{"resolution":512}' "resolution=512"
# Keep inference/class IDs identical to online mapping, but carry the original
# camera RGB in the point cloud. The semantic map is built from semantic_id,
# while the separate texture map can robustly fuse real floor colors.
post_sam3_setting color_mode '{"mode":"real"}' "color_mode=real (実RGBテクスチャ用)"
post_sam3_setting fast_iters '{"iters":4}' "FastStereo iterations=4"
post_sam3_setting depth_downsample '{"downsample":4}' "depth downsample=4"
post_sam3_setting max_distance '{"distance":15.0}' "max distance=15.0m"
post_sam3_setting depth_mode '{"mode":"fast_stereo"}' "offline depth=FastStereo"
post_sam3_setting source_mode '{"mode":"network"}' "source=rosbag network"

if [ "$SAM3_SETTING_FAILURES" -ne 0 ]; then
    echo "エラー: SAM3設定を適用できないため、条件不一致の実験は開始しません。"
    exit 1
fi

echo ""
read -p "SAM3専用UIで設定を確認・変更しますか？ (y/N) [N]: " SAM3_UI_CHOICE
SAM3_UI_CHOICE=$(echo "${SAM3_UI_CHOICE:-n}" | tr '[:upper:]' '[:lower:]')
if [ "$SAM3_UI_CHOICE" = "y" ] || [ "$SAM3_UI_CHOICE" = "yes" ]; then
    bash "$WS_DIR/bash/startup_bash/open_sam3_settings_ui.sh"
    echo "ブラウザで設定を変更できます。変更内容は今回の再生に使用されます。"
    read -p "設定が完了したら Enter を押してください: " _SAM3_UI_DONE
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

START_PAUSED_FLAG="false"
if [ "$USE_RVIZ_FLAG" = "true" ]; then
    echo ""
    read -p "RVizを確認してからRosbag再生を開始しますか？ (Y/n) [Y]: " PAUSED_CHOICE
    PAUSED_CHOICE=$(echo "${PAUSED_CHOICE:-y}" | tr '[:upper:]' '[:lower:]')
    if [ "$PAUSED_CHOICE" != "n" ] && [ "$PAUSED_CHOICE" != "no" ]; then
        START_PAUSED_FLAG="true"
    fi
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
echo "再生中のキー操作: Space=一時停止/再開、→=1メッセージ進む、↑/↓=速度変更"
if [ "$START_PAUSED_FLAG" = "true" ]; then
    echo "一時停止状態で起動します。RVizの準備後、この端末でSpaceを押してください。"
fi
if [ ! -f "$SELECTED_BAG/metadata.yaml" ]; then
    echo "Rosbag メタデータ (metadata.yaml) を生成・再インデックス中..."
    ros2 bag reindex "$SELECTED_BAG" -s mcap
fi

PLAY_OPTIONS=(--rate "$PLAY_RATE")
if [ "$START_PAUSED_FLAG" = "true" ]; then
    PLAY_OPTIONS+=(--start-paused)
fi

if ros2 bag info "$SELECTED_BAG" 2>/dev/null | grep -q "Topic: /clock"; then
    echo "✓ 録画データ内の /clock を使用して再生します"
    ros2 bag play "$SELECTED_BAG" "${PLAY_OPTIONS[@]}"
else
    echo "✓ --clock オプションを有効にして再生します"
    ros2 bag play "$SELECTED_BAG" --clock "${PLAY_OPTIONS[@]}"
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
        AUTO_MAP_NAME="semantic_${BAG_NAME}"
        echo "地図保存スクリプトを実行中..."
        echo "  自動地図名: $AUTO_MAP_NAME"
        bash "$MAP_SAVE_SCRIPT" "$AUTO_MAP_NAME"
    else
        echo "保存先ディレクトリ: $WS_DIR/maps_waypoints"
        mkdir -p "$WS_DIR/maps_waypoints"
        ros2 run nav2_map_server map_saver_cli -f "$WS_DIR/maps_waypoints/offline_semantic_${BAG_NAME}" --ros-args -p map_topic:=/sam3/colored_map_grid
        echo "✓ 地図を保存しました: $WS_DIR/maps_waypoints/offline_semantic_${BAG_NAME}"
    fi
fi

cleanup
