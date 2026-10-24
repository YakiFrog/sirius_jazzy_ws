#!/bin/bash
# 自律移動用ROS2 bag記録（FAST-LIO結果・TF・SAM3地図とコストマップ）
# 使い方: ./record_rosbag.sh
#         ファイル名を入力して記録開始

WS="${SIRIUS_WS_DIR:-$HOME/sirius_jazzy_ws}"
SUPPORT="$WS/bash/startup_bash/navigation_recording.py"
mapfile -t TOPICS < <(python3 "$SUPPORT" --topics)
if [ "${#TOPICS[@]}" -eq 0 ]; then echo "記録トピックを取得できません" >&2; exit 1; fi

ROSBAG_PID=""
CURRENT_BAG_PATH=""
RESULT_SHOWN=false
experiment_memo="No description"

stop_recording() {
    # One Ctrl+C is enough; further interrupts must not interrupt MCAP finalization.
    trap '' INT TERM HUP
    if [ -n "$ROSBAG_PID" ]; then
        echo ""
        echo "記録を停止・保存しています...（追加のCtrl+Cは不要です）"
        kill -INT "$ROSBAG_PID" 2>/dev/null || true
        wait "$ROSBAG_PID" 2>/dev/null || true
        ROSBAG_PID=""
    fi
    show_result
    exit 0
}
trap stop_recording INT TERM HUP

ROSBAG_DIR="${HOME}/rosbag2_data"

# 保存ディレクトリ作成
mkdir -p "$ROSBAG_DIR"

# 結果表示（1回だけ表示）
show_result() {
    if [ "$RESULT_SHOWN" = true ]; then
        return
    fi
    RESULT_SHOWN=true
    if [ -z "$CURRENT_BAG_PATH" ]; then return 0; fi
    if [ -n "${CAPTURE_PID:-}" ]; then
        for child in $(jobs -pr); do
            if [ "$child" = "$CAPTURE_PID" ]; then kill -TERM "$child" 2>/dev/null || true; fi
        done
        wait "$CAPTURE_PID" 2>/dev/null || true
        CAPTURE_PID=""
    fi
    if [ -n "${METADATA_PID:-}" ]; then
        for child in $(jobs -pr); do
            if [ "$child" = "$METADATA_PID" ]; then kill -TERM "$child" 2>/dev/null || true; fi
        done
    fi
    if [ ! -f "$CURRENT_BAG_PATH/metadata.yaml" ]; then
        echo "記録を確定できませんでした。保存先に metadata.yaml がありません。"
        return 1
    fi
    
    echo ""
    echo "========================================="
    echo "✓ 記録が完了しました"
    echo "  保存先: $CURRENT_BAG_PATH"
    
    # Clean rosbag shutdown already writes the MCAP summary/footer. Do not rewrite
    # the entire recording on every stop. Recovery is an explicit separate operation.
    local mcap_file
    while IFS= read -r -d '' mcap_file; do
        if [ -x "$WS/mcap" ] && ! timeout 3s "$WS/mcap" info "$mcap_file" >/dev/null 2>&1; then
            echo "  注意: MCAP確認が完了しませんでした。元データは変更していません。"
            echo "  必要に応じて recover_mcap_rosbag.sh で別フォルダへ復旧できます。"
        fi
    done < <(find "$CURRENT_BAG_PATH" -maxdepth 1 -name '*.mcap' -print0)

    # ファイルサイズ表示
    if [ -d "$CURRENT_BAG_PATH" ]; then
        size=$(du -sh "$CURRENT_BAG_PATH" | cut -f1)
        echo "  サイズ: $size"
    fi
    echo "========================================="
    echo "記録は保存済みです。補助情報を保存します（最大8秒）..."
    timeout 8s python3 "$SUPPORT" --capture "$CURRENT_BAG_PATH.capture/finish" --workspace "$WS" ||
        echo "補助情報の取得を打ち切りました。BAG本体は保存済みです。"
}

# ファイル名を入力
input_filename() {
    echo "========================================="
    echo "  自律移動用 ROS2 bag 記録"
    echo "========================================="
    echo ""
    echo "保存先: $ROSBAG_DIR"
    echo ""
    python3 "$SUPPORT" --check
    echo "静的地図・TFは保持済みデータも受信する設定です。"
    echo "使用中の地図ファイルと設定は .capture フォルダにも保存します。"
    echo "※ /velodyne_points (3D点群) は除外されます"
    echo ""
    echo "========================================="
    echo -n "ファイル名を入力してください (例: outdoor_test): "
    read bag_name
    
    if [ -z "$bag_name" ]; then
        # デフォルト名: 日時
        bag_name="rosbag_$(date +%Y%m%d_%H%M%S)"
        echo "デフォルト名を使用: $bag_name"
    fi

    echo -n "実験内容のメモを入力してください (例: SLAM精度検証): "
    read input_memo
    if [ -n "$input_memo" ]; then
        experiment_memo="$input_memo"
    else
        experiment_memo="No description"
    fi
    
    # 既存チェック
    if [ -d "$ROSBAG_DIR/$bag_name" ]; then
        echo ""
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
    echo ""
}

# 記録開始
start_recording() {
    CURRENT_BAG_PATH="$ROSBAG_DIR/$1"
    RESULT_SHOWN=false
    
    echo "========================================="
    echo "記録を開始します..."
    echo "  保存先: $CURRENT_BAG_PATH"
    echo "  実験メモ: $experiment_memo"
    echo "  停止するには Ctrl+C を押してください"
    echo "========================================="
    echo ""
    
    # 録画開始の瞬間にのみ実験メタデータを1回パブリッシュ
    memo_payload=$(python3 -c 'import json,sys; print(json.dumps({"data":sys.argv[1]}))' "$experiment_memo")
    ros2 topic pub --once /experiment_metadata std_msgs/msg/String "$memo_payload" >/dev/null 2>&1 &

    METADATA_PID=$!

    python3 "$SUPPORT" --exec-recorder -s mcap --storage-preset-profile zstd_fast \
        --qos-profile-overrides-path "$WS/params/rosbag_navigation_qos.yaml" \
        --include-hidden-topics -o "$CURRENT_BAG_PATH" --topics "${TOPICS[@]}" &
    
    ROSBAG_PID=$!
    mkdir -p "$CURRENT_BAG_PATH.capture"
    timeout 12s python3 "$SUPPORT" --capture "$CURRENT_BAG_PATH.capture/start" --workspace "$WS" &
    CAPTURE_PID=$!
    printf '%s\n' "$experiment_memo" > "$CURRENT_BAG_PATH.capture/experiment_memo.txt"
    wait "$ROSBAG_PID"
    ROSBAG_PID=""
    
    show_result
}

# メイン処理（ループ）
cd "$WS"
source install/setup.bash

while : ; do
    echo ""
    read -p "Press [Enter] key to start recording..."
    
    selected_name=""
    input_filename
    
    if [ $? -eq 0 ] && [ -n "$selected_name" ]; then
        start_recording "$selected_name"
    fi
    
    echo ""
    echo "-----------------------------------------"
done
