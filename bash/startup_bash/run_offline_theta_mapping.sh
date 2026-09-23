#!/bin/bash
# ==============================================================================
# THETA オフライン路面マッピング 実行・保存スクリプト
# 記録済み Rosbag を再生し、THETA BEV -> 地面点群 -> RTAB-Map で路面地図を生成します。
# SAM3・ステレオ・深度は使用しません。
# ==============================================================================

WS_DIR="${HOME}/sirius_jazzy_ws"
ROSBAG_DIR="${HOME}/rosbag2_data"

# Set domain isolation to prevent collisions with running Unity simulation
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}

source "$WS_DIR/install/setup.bash" 2>/dev/null || source /opt/ros/jazzy/setup.bash

echo "================================================="
echo "  THETA オフライン路面マッピング (Domain: $ROS_DOMAIN_ID)"
echo "================================================="

# 0. 実行モードの選択（重ね合わせのみならマッピングを起動しない）
echo ""
echo "実行内容を選択してください:"
echo "  [1] Rosbagを再生して新規マッピング"
echo "  [2] 既存の生成済み地図をSLAM Toolbox地図と重ね合わせる"
read -p "選択 [1]: " RUN_MODE
RUN_MODE=${RUN_MODE:-1}
if [ "$RUN_MODE" = "2" ]; then
    exec bash "$WS_DIR/bash/startup_bash/rebase_existing_map.sh"
fi

# 1. 録画データ (Rosbag) の選択
BAG_LIST=($(find "$ROSBAG_DIR" -maxdepth 1 -mindepth 1 -type d | sort -r))

if [ ${#BAG_LIST[@]} -eq 0 ]; then
    echo ""
    echo "エラー: $ROSBAG_DIR に Rosbag データが見つかりません。"
    echo "先に ./record_rosbag_offline_theta.sh で走行データを録画してください。"
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

# 選択したフォルダ直下にMCAPがなければ、日付などでまとめた親フォルダとみなし、
# その中のRosbagフォルダ（MCAPを含むもの）を一覧表示して選ばせる。
if ! compgen -G "$SELECTED_BAG/*.mcap" >/dev/null; then
    INNER_BAG_LIST=()
    while IFS= read -r inner_dir; do
        if compgen -G "$inner_dir/*.mcap" >/dev/null; then
            INNER_BAG_LIST+=("$inner_dir")
        fi
    done < <(find "$SELECTED_BAG" -maxdepth 1 -mindepth 1 -type d | sort)

    if [ ${#INNER_BAG_LIST[@]} -gt 0 ]; then
        echo ""
        echo "$(basename "$SELECTED_BAG") 内の Rosbag 一覧:"
        for i in "${!INNER_BAG_LIST[@]}"; do
            echo "  [$((i+1))] $(basename "${INNER_BAG_LIST[$i]}")"
        done
        read -p "使用する Rosbag 番号を選択してください [1]: " inner_choice
        inner_choice=${inner_choice:-1}
        inner_index=$((inner_choice-1))
        if [ $inner_index -lt 0 ] || [ $inner_index -ge ${#INNER_BAG_LIST[@]} ]; then
            echo "無効な選択です。"
            exit 1
        fi
        SELECTED_BAG="${INNER_BAG_LIST[$inner_index]}"
    fi
fi

BAG_NAME=$(basename "$SELECTED_BAG")
echo "選択された Rosbag: $SELECTED_BAG"

# 補正済みTFやTHETA画像が欠けたbagを再生すると誤った地図を生成するため、事前に拒否する。
BAG_VALIDATOR="$WS_DIR/bash/startup_bash/validate_theta_offline_mapping_bag.py"
if [ -f "$BAG_VALIDATOR" ]; then
    echo ""
    echo "Rosbagの必須情報を検証中..."
    VALIDATION_STATUS=0

    # metadata.yaml can still look healthy when the MCAP footer or final chunk
    # is incomplete. Check the complete MCAP structure before trusting topic counts.
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

    # Exit code 2 means rosbag2 could not open the storage (interrupted recording).
    # Recover into a new directory and validate again. Exit code 1 means readable
    # data with missing inputs and must not be repaired.
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

# 2b. RViz2プレビューの選択
echo ""
read -p "RViz2でプレビューしながら実行しますか？ (Y/n) [Y]: " RVIZ_CHOICE
RVIZ_CHOICE=$(echo "${RVIZ_CHOICE:-y}" | tr '[:upper:]' '[:lower:]')
USE_RVIZ_FLAG="false"
if [ "$RVIZ_CHOICE" != "n" ] && [ "$RVIZ_CHOICE" != "no" ]; then
    USE_RVIZ_FLAG="true"
fi

# 2c. SAM3セマンティック分類の選択（要 sam3 docker サーバ）
echo ""
read -p "SAM3でセマンティック分類も行いますか？ (y/N) [N]: " SAM3_CHOICE
SAM3_CHOICE=$(echo "${SAM3_CHOICE:-n}" | tr '[:upper:]' '[:lower:]')
USE_SAM3_FLAG="false"
if [ "$SAM3_CHOICE" = "y" ] || [ "$SAM3_CHOICE" = "yes" ]; then
    USE_SAM3_FLAG="true"
    echo "※ sam3 docker サーバ (port 8080) が起動している必要があります。"
fi

# 2d. デバッグ可視化の選択（検出画像・意味色点群をRVizで同時表示）
echo ""
read -p "デバッグ可視化（検出画像/意味色点群をRViz表示）を使いますか？ (y/N) [N]: " DEBUG_CHOICE
DEBUG_CHOICE=$(echo "${DEBUG_CHOICE:-n}" | tr '[:upper:]' '[:lower:]')
DEBUG_ARGS=()
if [ "$DEBUG_CHOICE" = "y" ] || [ "$DEBUG_CHOICE" = "yes" ]; then
    DEBUG_ARGS=(publish_debug:=true publish_raw:=true "rviz_config:=$WS_DIR/src/sirius/sirius_navigation/rviz/theta_debug.rviz")
    USE_RVIZ_FLAG="true"
    echo "※ デバッグRViz表示のため RViz2 を有効にします。"
fi

# 2e. 地面点群の半径(幅)フィルタの選択
echo ""
echo "地面点群の半径(幅)フィルタを選択してください（床を広く取るには無効化）:"
echo "  [1] 既定の半径を使う [推奨]  (sim=4.8m / 実機=3.0m)"
echo "  [2] 半径を数値で指定"
echo "  [3] 半径フィルタを無効化 (BEV全域・床を広くマッピング)"
read -p "選択 [1]: " RADIUS_CHOICE
RADIUS_CHOICE=${RADIUS_CHOICE:-1}

# 3. マッピングノードを起動（sirius_navigationパッケージ）
# 実機bag（real_*mapping_* / real_*theta*）は実機校正YAMLを自動適用し、
# 地面点群の半径ベース値を3.0mにする。scan3(VLP-16 2.5D)ゲートも併用。
# ゲートは高さ不一致(0.85m vs 1.135m)による過剰除外を避けるため、
# 「自機ビーム除外・時刻合わせ・保留率<50%なら自動無効」のガード付き。
CALIB_ARGS=()
GATE_ARGS=()
BASE_MAX_RADIUS=""
BASE_GRID_RANGE=""
CALIB_REAL="$WS_DIR/src/sirius/sirius_navigation/config/theta_calibration_real.yaml"
case "$BAG_NAME" in
    real_theta_mapping*|real_both_mapping*|real_*)
        [ -f "$CALIB_REAL" ] && CALIB_ARGS=(calibration:="$CALIB_REAL")
        BASE_MAX_RADIUS="3.0"
        BASE_GRID_RANGE="3.0"
        GATE_ARGS=(lidar_topic:=/scan3 lidar_gate:=true)
        ;;
esac

RADIUS_ARGS=()
GRID_ARGS=()
RADIUS_DESCRIPTION=""
case "$RADIUS_CHOICE" in
    2)
        read -p "最大半径[m] (0.1〜7.0): " radius_input
        if ! awk -v r="$radius_input" \
            'BEGIN { exit !(r ~ /^[0-9]+([.][0-9]+)?$/ && r >= 0.1 && r <= 7.0) }'; then
            echo "エラー: 最大半径は 0.1〜7.0 の数値で指定してください。"
            exit 1
        fi
        RADIUS_ARGS=(max_radius:="$radius_input")
        RADIUS_DESCRIPTION="${radius_input}m"
        # RTAB-Map の地図範囲が半径より狭いと床が切れるため追随させる
        # （simの既定 grid_range_max=7.0 は半径上限7.0を覆うため変更不要）
        if [ -n "$BASE_GRID_RANGE" ] && awk -v r="$radius_input" -v base="$BASE_GRID_RANGE" \
            'BEGIN { exit !(r > base) }'; then
            GRID_ARGS=(grid_range_max:="$radius_input")
        fi
        ;;
    3)
        RADIUS_ARGS=(max_radius:=0.0)
        # BEV(10m四方)全域を覆うようRTAB-Mapの地図範囲を広げる
        GRID_ARGS=(grid_range_max:=7.0)
        RADIUS_DESCRIPTION="無効 (0.0)"
        ;;
    *)
        [ -n "$BASE_MAX_RADIUS" ] && RADIUS_ARGS=(max_radius:="$BASE_MAX_RADIUS")
        [ -n "$BASE_GRID_RANGE" ] && GRID_ARGS=(grid_range_max:="$BASE_GRID_RANGE")
        RADIUS_DESCRIPTION="既定 (${BASE_MAX_RADIUS:-4.8}m)"
        ;;
esac

LIDAR_ARGS=("${GATE_ARGS[@]}" "${RADIUS_ARGS[@]}" "${GRID_ARGS[@]}")

echo "================================================="
echo "路面マッピングパイプラインを起動しています..."
echo "  Rosbag: $BAG_NAME"
echo "  再生速度: ${PLAY_RATE}x"
echo "  RViz2 プレビュー: $USE_RVIZ_FLAG"
echo "  SAM3セマンティック: $USE_SAM3_FLAG"
echo "  デバッグ可視化: $DEBUG_CHOICE"
echo "  半径(幅)フィルタ: $RADIUS_DESCRIPTION"
echo "  姿勢TF: bag内の補正済みTFを使用"
echo "================================================="

if [ ${#CALIB_ARGS[@]} -gt 0 ]; then
    echo "実機校正を使用: $CALIB_REAL"
fi
if [ ${#LIDAR_ARGS[@]} -gt 0 ]; then
    echo "地面点群/LiDAR引数: ${LIDAR_ARGS[*]}"
fi
ros2 launch sirius_navigation theta_offline_mapping.launch.py \
    use_sim_time:=true rviz:="$USE_RVIZ_FLAG" sam3:="$USE_SAM3_FLAG" "${DEBUG_ARGS[@]}" "${CALIB_ARGS[@]}" "${LIDAR_ARGS[@]}" &
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
if [ ! -f "$SELECTED_BAG/metadata.yaml" ]; then
    echo "Rosbag メタデータ (metadata.yaml) を生成・再インデックス中..."
    ros2 bag reindex "$SELECTED_BAG" -s mcap
fi

PLAY_OPTIONS=(--rate "$PLAY_RATE")

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
read -p "生成された路面地図を保存しますか？ (Y/n): " save_choice
save_choice=$(echo "$save_choice" | tr '[:upper:]' '[:lower:]')

if [ "$save_choice" != "n" ] && [ "$save_choice" != "no" ]; then
    # SLAM Toolbox地図を構造ベースにした重ね合わせ版を作るか選択（ZEDと同じ）
    SLAM_BASE_YAML=""
    echo ""
    read -p "SLAM Toolbox地図と重ね合わせた版も作りますか？ (y/N) [N]: " SLAM_OVERLAY_CHOICE
    SLAM_OVERLAY_CHOICE=$(echo "${SLAM_OVERLAY_CHOICE:-n}" | tr '[:upper:]' '[:lower:]')
    if [ "$SLAM_OVERLAY_CHOICE" = "y" ] || [ "$SLAM_OVERLAY_CHOICE" = "yes" ]; then
        mapfile -t SLAM_MAP_YAMLS < <(
            find "$WS_DIR/maps_waypoints/maps" -maxdepth 1 -type f -name '*.yaml' 2>/dev/null | sort
        )
        if [ "${#SLAM_MAP_YAMLS[@]}" -eq 0 ]; then
            echo "警告: SLAM Toolbox地図YAMLが見つかりません。重ね合わせは行いません。"
        else
            echo "利用可能なSLAM Toolbox地図:"
            for i in "${!SLAM_MAP_YAMLS[@]}"; do
                echo "  [$((i+1))] $(basename "${SLAM_MAP_YAMLS[$i]}")"
            done
            read -p "構造ベースにする地図番号 [1]: " SLAM_MAP_CHOICE
            SLAM_MAP_CHOICE=${SLAM_MAP_CHOICE:-1}
            SLAM_MAP_INDEX=$((SLAM_MAP_CHOICE-1))
            if [ "$SLAM_MAP_INDEX" -ge 0 ] && [ "$SLAM_MAP_INDEX" -lt "${#SLAM_MAP_YAMLS[@]}" ]; then
                SLAM_BASE_YAML="${SLAM_MAP_YAMLS[$SLAM_MAP_INDEX]}"
                echo "SLAM Toolbox構造地図: $SLAM_BASE_YAML"
            else
                echo "警告: 無効な選択です。重ね合わせは行いません。"
            fi
        fi
    fi

    MAP_SAVE_SCRIPT="$WS_DIR/bash/startup_bash/rtabmap_save.sh"
    if [ -f "$MAP_SAVE_SCRIPT" ]; then
        AUTO_MAP_NAME="theta_road_${BAG_NAME}"
        echo "地図保存スクリプトを実行中 (indexed地図: /theta/save_indexed_map)..."
        echo "  自動地図名: $AUTO_MAP_NAME"
        COLORIZER_SCRIPT="$WS_DIR/src/sirius/sirius_navigation/sirius_navigation/theta_colorize_map.py" \
            INDEXED_SAVE_TOPIC=/theta/save_indexed_map \
            REBASE_ARGS="" \
            bash "$MAP_SAVE_SCRIPT" "$AUTO_MAP_NAME" "$SLAM_BASE_YAML"
    else
        echo "保存先ディレクトリ: $WS_DIR/maps_waypoints"
        mkdir -p "$WS_DIR/maps_waypoints"
        ros2 run nav2_map_server map_saver_cli -f "$WS_DIR/maps_waypoints/theta_road_${BAG_NAME}" --ros-args \
            -r map:=/rtabmap/grid_map -p map_subscribe_transient_local:=true -p save_map_timeout:=10000.0
        echo "✓ 地図を保存しました: $WS_DIR/maps_waypoints/theta_road_${BAG_NAME}"
    fi
fi

cleanup
