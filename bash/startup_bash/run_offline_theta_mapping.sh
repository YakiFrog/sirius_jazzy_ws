#!/bin/bash
# ==============================================================================
# THETA オフライン路面マッピング 実行・保存スクリプト
# 通常は記録画像を逐次処理し、品質保護・暗所認識・時系列追跡・領域補正を適用します。
# 始終点の画像ループ閉合を追加する場合は open・closed を別保存します。
# ==============================================================================

WS_DIR="${HOME}/sirius_jazzy_ws"
ROSBAG_DIR="${HOME}/rosbag2_data"
ROAD_SEMANTIC_DIR="${ROAD_SEMANTIC_DIR:-$WS_DIR/maps_waypoints/maps/road_semantic}"

# Set domain isolation to prevent collisions with running Unity simulation
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}

source "$WS_DIR/install/setup.bash" 2>/dev/null || source /opt/ros/jazzy/setup.bash

echo "================================================="
echo "  THETA オフライン路面マッピング (Domain: $ROS_DOMAIN_ID)"
echo "================================================="

# 0. 実行モードの選択
echo ""
echo "実行内容を選択してください:"
echo "  [1] 改善済みの逐次処理で新規マッピング（全画像・推奨）"
echo "  [4] 始終点の画像ループ閉合を試す（open・closedを別保存）"
if [ "${1:-}" = "--visual-loop" ]; then
    RUN_MODE=4
else
    read -r -p "選択 [1]: " RUN_MODE
fi
RUN_MODE=${RUN_MODE:-1}
if [ "$RUN_MODE" != "1" ] && [ "$RUN_MODE" != "4" ]; then
    echo "無効な実行内容です。"
    exit 1
fi

# 1. 録画データ (Rosbag) の選択
# HDDに移したBAGへのリンクも一覧に含める。
mapfile -t BAG_LIST < <(find -L "$ROSBAG_DIR" -maxdepth 1 -mindepth 1 -type d | sort -r)

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

# metadata.yaml が残っていても、中断した MCAP はフッタや末尾が欠けていることがある。
# トピックと撮影時刻の TF は、後段の逐次生成が検証する。
if [ -x "$WS_DIR/mcap" ]; then
    mapfile -d '' SELECTED_MCAP_FILES < <(
        find "$SELECTED_BAG" -maxdepth 1 -type f -name '*.mcap' -print0 | sort -z
    )
    for mcap_file in "${SELECTED_MCAP_FILES[@]}"; do
        echo "MCAPファイル構造を確認中..."
        if "$WS_DIR/mcap" doctor "$mcap_file" >/dev/null 2>&1; then
            continue
        fi
        echo "✗ MCAP破損または未完了を検出: $(basename "$mcap_file")"
        RECOVERY_SCRIPT="$WS_DIR/bash/startup_bash/recover_mcap_rosbag.sh"
        if [ ! -f "$RECOVERY_SCRIPT" ]; then
            echo "エラー: MCAP自動復旧スクリプトがありません: $RECOVERY_SCRIPT"
            exit 1
        fi
        echo "MCAP自動復旧を試します..."
        if ! RECOVERED_BAG=$(bash "$RECOVERY_SCRIPT" "$SELECTED_BAG"); then
            echo "エラー: MCAPを自動復旧できませんでした。元のRosbagは変更していません。"
            exit 1
        fi
        SELECTED_BAG="$RECOVERED_BAG"
        BAG_NAME=$(basename "$SELECTED_BAG")
        echo "復旧したRosbag: $SELECTED_BAG"
        break
    done
fi

# 記録画像を逐次処理し、SAM3の推論速度に依存したフレーム落ちを防ぐ。
ROAD_CONFIG="${THETA_ROAD_CONFIG:-$WS_DIR/params/theta_road_mapping.yaml}"
DEFAULT_CALIB="$WS_DIR/src/sirius/sirius_navigation/config/theta_calibration.yaml"
case "$BAG_NAME" in
    real_*|*_real_*) DEFAULT_CALIB="$WS_DIR/src/sirius/sirius_navigation/config/theta_calibration_real.yaml" ;;
esac
# Prefer the actual recording's snapshot, including when using a recovered BAG.
CAPTURE_CALIB="$SELECTED_BAG/capture/theta_calibration.yaml"
if [ ! -f "$CAPTURE_CALIB" ]; then
    CAPTURE_CALIB="${SELECTED_BAG%_recovered*}/capture/theta_calibration.yaml"
fi
if [ -f "$CAPTURE_CALIB" ]; then
    DEFAULT_CALIB="$CAPTURE_CALIB"
fi
echo ""
echo "共通設定: $ROAD_CONFIG"
read -r -p "校正YAML [$DEFAULT_CALIB]: " ROAD_CALIB
ROAD_CALIB="${ROAD_CALIB:-$DEFAULT_CALIB}"
read -r -p "SAM3セマンティック分類も行いますか？ (Y/n) [Y]: " ROAD_SAM3
ROAD_ARGS=()
case "${ROAD_SAM3,,}" in n|no) ROAD_ARGS+=(--no-semantic) ;; esac
if [ "$RUN_MODE" = "4" ]; then
    read -r -e -p "保存済みSLAM Toolboxグラフ（.posegraphファイルまたは接頭辞）: " LOOP_GRAPH
    LOOP_GRAPH="${LOOP_GRAPH/#\~/$HOME}"
    LOOP_GRAPH="${LOOP_GRAPH%.posegraph}"
    if [ -z "$LOOP_GRAPH" ] || [ ! -f "$LOOP_GRAPH.posegraph" ] || [ ! -f "$LOOP_GRAPH.data" ]; then
        echo "同じBAGから保存した .posegraph と .data を指定してください。"
        exit 1
    fi
    read -r -p "短時間補間用のオドメトリトピック [/odom]: " LOOP_ODOM
    read -r -e -p "連続EKF軌跡NPZ（任意、空欄ならBAGのオドメトリ）: " LOOP_POSES
    LOOP_POSES="${LOOP_POSES/#\~/$HOME}"
    if [ -n "$LOOP_POSES" ]; then
        ROAD_ARGS+=(--pose-npz "$LOOP_POSES")
    fi
    ROAD_OUTPUT=$(python3 "$WS_DIR/bash/startup_bash/road_map_output.py" \
        "$ROAD_SEMANTIC_DIR" "$SELECTED_BAG" theta sequential) || exit 1
    echo "出力先: $ROAD_OUTPUT/{open,closed}"
    exec python3 "$WS_DIR/bash/startup_bash/run_theta_visual_loop.py" \
        "$SELECTED_BAG" "$ROAD_OUTPUT" --graph "$LOOP_GRAPH" --calibration "$ROAD_CALIB" \
        --config "$ROAD_CONFIG" --odom-topic "${LOOP_ODOM:-/odom}" "${ROAD_ARGS[@]}"
fi
ROAD_OUTPUT=$(python3 "$WS_DIR/bash/startup_bash/road_map_output.py" \
    "$ROAD_SEMANTIC_DIR" "$SELECTED_BAG" theta sequential) || exit 1
echo "全画像を順番に処理します。設定・元の分類・推論キャッシュも保存します。"
echo "出力先: $ROAD_OUTPUT"
if ! python3 "$WS_DIR/bash/startup_bash/generate_theta_road_map.py" \
    "$SELECTED_BAG" "$ROAD_OUTPUT" --calibration "$ROAD_CALIB" \
    --config "$ROAD_CONFIG" --reserved-output --map-prefix map "${ROAD_ARGS[@]}"; then
    echo "生成に失敗しました。原因と途中データは上記の出力先で確認できます。"
    exit 1
fi
MAP_SELECT_NAME="${ROAD_OUTPUT#"$WS_DIR/maps_waypoints/maps/"}"
echo "✓ 地図生成が完了しました。Map Select: $MAP_SELECT_NAME/map"
echo "比較画像: $ROAD_OUTPUT/images.html"
