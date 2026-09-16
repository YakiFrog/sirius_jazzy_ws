#!/bin/bash
# ==============================================================================
# THETA オフライン路面マッピング専用 ROS2 bag 記録スクリプト
# Webシミュレータのデュアルフィッシュ映像・LiDAR・SLAM補正TFを同期録画します。
# SAM3・ステレオ・Unity stereo bridge は使用しません。
# ==============================================================================

WS_DIR="${HOME}/sirius_jazzy_ws"
ROSBAG_DIR="${HOME}/rosbag2_data"
mkdir -p "$ROSBAG_DIR"

source "$WS_DIR/install/setup.bash" 2>/dev/null || source /opt/ros/jazzy/setup.bash

ROSBAG_PID=""
HEARTBEAT_PID=""
CAPTURE_PID=""
CURRENT_BAG_PATH=""
RESULT_SHOWN=false
CLEANUP_IN_PROGRESS=false
VALIDATOR_SCRIPT="${WS_DIR}/bash/startup_bash/validate_theta_offline_mapping_bag.py"
THETA_TOPIC="/theta/dual_fisheye/image_raw/compressed"
experiment_memo="THETA Offline Road Mapping"

# シミュ用(sim) / 実機用(real)。実機は /clock を要求せず use_sim_time=false。
MODE="${1:-sim}"
if [ "$MODE" != "sim" ] && [ "$MODE" != "real" ]; then
    echo "使い方: $0 [sim|real]"
    exit 1
fi
if [ "$MODE" = "real" ]; then
    USE_SIM_TIME=false
    REQUIRE_CLOCK=false
    MODE_LABEL="実機"
    THETA_SOURCE_MSG="THETAキャプチャnodeが $THETA_TOPIC を配信していること"
    DEFAULT_BAG_NAME="real_theta_mapping_$(date +%Y%m%d_%H%M%S)"
else
    USE_SIM_TIME=true
    REQUIRE_CLOCK=true
    MODE_LABEL="Webシミュレータ"
    THETA_SOURCE_MSG="アプリの「THETA映像をROS2へ送信」をONにし、rosbridgeへ接続しておいてください"
    DEFAULT_BAG_NAME="sim_theta_mapping_$(date +%Y%m%d_%H%M%S)"
fi

cleanup() {
    local exit_status="${1:-0}"
    if [ "$CLEANUP_IN_PROGRESS" = true ]; then
        echo ""
        echo "⚠ 保存処理は進行中です。完了表示までそのままお待ちください。"
        return
    fi
    CLEANUP_IN_PROGRESS=true
    trap 'echo ""; echo "⚠ 保存処理中です。Ctrl+Cを連打せず、完了表示までお待ちください。"' INT TERM HUP

    echo ""
    echo "================================================="
    echo "■ Ctrl+Cを受け付けました。録画を安全に停止します。"
    echo "  MCAPの索引保存・検証に数秒かかることがあります。"
    echo "================================================="

    stop_recording_heartbeat
    if [ -n "$ROSBAG_PID" ]; then
        echo "[1/3] ros2 bag recordへ停止要求を送信..."
        kill -INT "$ROSBAG_PID" 2>/dev/null || true
        wait_for_process "$ROSBAG_PID" "Rosbag終了・MCAP索引書き込み"
        wait "$ROSBAG_PID" 2>/dev/null || true
        ROSBAG_PID=""
        echo "  ✓ Rosbag記録プロセスが終了しました"
    fi
    if [ -n "$CAPTURE_PID" ]; then
        echo "[2/3] THETAキャプチャnodeを停止..."
        kill -INT "$CAPTURE_PID" 2>/dev/null || true
        wait "$CAPTURE_PID" 2>/dev/null || true
        CAPTURE_PID=""
        echo "  ✓ THETAキャプチャnodeを停止しました"
    fi
    show_result
    echo "================================================="
    echo "✓ すべての録画終了処理が完了しました。"
    echo "================================================="
    exit "$exit_status"
}

trap 'cleanup 130' INT TERM HUP

wait_for_process() {
    local pid="$1"
    local label="$2"
    local elapsed=0
    local state
    while kill -0 "$pid" 2>/dev/null; do
        state=$(ps -o stat= -p "$pid" 2>/dev/null | tr -d ' ')
        if [[ "$state" == Z* ]] || [ -z "$state" ]; then
            break
        fi
        sleep 1
        elapsed=$((elapsed + 1))
        if [ "$elapsed" -eq 1 ] || [ $((elapsed % 3)) -eq 0 ]; then
            echo "  … $label 処理中 (${elapsed}秒)"
        fi
    done
}

run_with_progress() {
    local label="$1"
    shift
    "$@" &
    local command_pid=$!
    wait_for_process "$command_pid" "$label"
    wait "$command_pid" 2>/dev/null
}

recording_heartbeat() {
    local started_at
    local elapsed
    local size
    started_at=$(date +%s)
    while [ -n "$ROSBAG_PID" ] && kill -0 "$ROSBAG_PID" 2>/dev/null; do
        sleep 10
        if ! kill -0 "$ROSBAG_PID" 2>/dev/null; then
            break
        fi
        elapsed=$(($(date +%s) - started_at))
        size="準備中"
        if [ -d "$CURRENT_BAG_PATH" ]; then
            size=$(du -sh "$CURRENT_BAG_PATH" 2>/dev/null | cut -f1)
        fi
        echo "[録画中] ${elapsed}秒経過 / bagサイズ: ${size} / 停止: Ctrl+Cを1回"
    done
}

start_recording_heartbeat() {
    recording_heartbeat &
    HEARTBEAT_PID=$!
}

stop_recording_heartbeat() {
    if [ -n "$HEARTBEAT_PID" ]; then
        kill -TERM "$HEARTBEAT_PID" 2>/dev/null || true
        wait "$HEARTBEAT_PID" 2>/dev/null || true
        HEARTBEAT_PID=""
    fi
}

check_topic_message() {
    local topic="$1"
    local expected_type="$2"
    local wait_seconds="$3"
    local actual_type
    actual_type=$(ros2 topic type "$topic" 2>/dev/null | head -n 1)
    if [ "$actual_type" != "$expected_type" ]; then
        if [ -z "$actual_type" ]; then
            echo "  ✗ $topic にPublisherがありません"
        else
            echo "  ✗ $topic の型が不正です: $actual_type"
        fi
        return 1
    fi
    if timeout "${wait_seconds}s" ros2 topic echo --once "$topic" >/dev/null 2>&1; then
        echo "  ✓ $topic の実メッセージを受信"
        return 0
    fi
    echo "  ✗ $topic にPublisherはありますが、実メッセージを受信できません"
    return 1
}

check_tf_chain() {
    local parent="$1"
    local child="$2"
    local label="$3"
    local output
    output=$(timeout 7s ros2 run tf2_ros tf2_echo "$parent" "$child" 2>&1 || true)
    if printf '%s\n' "$output" | grep -q "Translation:"; then
        echo "  ✓ $label ($parent → $child)"
        return 0
    fi
    echo "  ✗ $label が取得できません ($parent → $child)"
    return 1
}

preflight_check() {
    local errors=0
    local rate_output
    echo ""
    echo "========================================="
    echo "録画開始前の必須入力チェック"
    echo "========================================="
    check_topic_message "$THETA_TOPIC" sensor_msgs/msg/CompressedImage 15 || errors=$((errors + 1))
    check_topic_message /scan3 sensor_msgs/msg/LaserScan 4 || errors=$((errors + 1))
    check_topic_message /odom/filtered nav_msgs/msg/Odometry 4 || errors=$((errors + 1))
    if [ "$REQUIRE_CLOCK" = true ]; then
        check_topic_message /clock rosgraph_msgs/msg/Clock 4 || errors=$((errors + 1))
    fi
    check_tf_chain map sirius3/base_footprint "SLAM Toolbox補正済みTF" || errors=$((errors + 1))
    check_tf_chain sirius3/base_footprint sirius3/theta_link "THETA取付TF" || errors=$((errors + 1))

    if ros2 topic type "$THETA_TOPIC" >/dev/null 2>&1; then
        rate_output=$(timeout 6s ros2 topic hz "$THETA_TOPIC" 2>/dev/null || true)
        if printf '%s\n' "$rate_output" | grep -q "average rate:"; then
            echo "  ✓ THETA $(printf '%s\n' "$rate_output" | grep "average rate:" | tail -n 1)"
        else
            echo "  ✗ THETAの連続受信レートを確認できません"
            errors=$((errors + 1))
        fi
    fi

    echo "========================================="
    if [ "$errors" -ne 0 ]; then
        echo "✗ $errors 項目に問題があるため録画を開始しません。"
        return 1
    fi
    echo "✓ 必須情報をすべて確認しました。録画を開始できます。"
    return 0
}

show_result() {
    if [ "$RESULT_SHOWN" = true ]; then
        return
    fi
    RESULT_SHOWN=true

    if [ -z "$CURRENT_BAG_PATH" ]; then
        echo ""
        echo "========================================="
        echo "録画データは作成されませんでした。"
        echo "========================================="
        return
    fi

    echo ""
    echo "========================================="
    echo "✓ 録画が完了しました"
    echo "  保存先: $CURRENT_BAG_PATH"

    if [ -d "$CURRENT_BAG_PATH" ]; then
        local mcap_file=$(find "$CURRENT_BAG_PATH" -name "*.mcap" | head -n 1)
        if [ -n "$mcap_file" ] && [ -f "$mcap_file" ]; then
            if [ -f "${WS_DIR}/mcap" ]; then
                echo "[1/3] MCAPインデックスの整合性をチェック・修復..."
                local tmp_mcap="${mcap_file}.tmp"
                if run_with_progress "MCAPインデックス修復" \
                    "${WS_DIR}/mcap" recover "$mcap_file" -o "$tmp_mcap"; then
                    mv "$tmp_mcap" "$mcap_file"
                    echo "  [MCAP] ✓ インデックス処理完了"
                else
                    rm -f "$tmp_mcap"
                fi
            fi
        fi
        if [ ! -f "$CURRENT_BAG_PATH/metadata.yaml" ]; then
            echo "  [Rosbag] metadata.yaml を生成・インデックス登録..."
            run_with_progress "Rosbagメタデータ生成" \
                ros2 bag reindex "$CURRENT_BAG_PATH" -s mcap
            echo "  [Rosbag] ✓ メタデータ生成完了"
        fi
        size=$(du -sh "$CURRENT_BAG_PATH" | cut -f1)
        echo "  データサイズ: $size"
        echo ""
        echo "[2/3] 録画内容を検証..."
        if [ -f "$VALIDATOR_SCRIPT" ]; then
            if [ "$REQUIRE_CLOCK" = true ]; then
                run_with_progress "必須トピック・TF検証" \
                    python3 "$VALIDATOR_SCRIPT" "$CURRENT_BAG_PATH" --require-clock || true
            else
                run_with_progress "必須トピック・TF検証" \
                    python3 "$VALIDATOR_SCRIPT" "$CURRENT_BAG_PATH" || true
            fi
        fi
    fi
    echo "========================================="
}

echo "========================================="
echo "  THETA オフライン路面マッピング 録画ツール [$MODE_LABEL]"
echo "========================================="
echo "※ SLAM Toolboxは自動起動しません。録画前にLauncherから slamtoolbox を起動してください。"
echo "※ $THETA_SOURCE_MSG"

# 実機: THETA HDMIキャプチャノードが未起動なら自動起動する（YUYV/5fps）。
if [ "$MODE" = "real" ]; then
    if ! ros2 topic type "$THETA_TOPIC" >/dev/null 2>&1; then
        echo "THETAキャプチャnode (theta_capture_node) を起動します..."
        ros2 run sirius_navigation theta_capture_node --ros-args \
            -p device:=/dev/theta_capture -p fourcc:=YUYV -p fps:=5.0 &
        CAPTURE_PID=$!
        sleep 3
        if ! kill -0 "$CAPTURE_PID" 2>/dev/null; then
            echo "エラー: THETAキャプチャnodeを起動できませんでした。"
            CAPTURE_PID=""
            cleanup 1
        fi
    fi
fi

if ! preflight_check; then
    cleanup 1
fi

input_filename() {
    echo ""
    echo "記録対象トピック:"
    echo "  - $THETA_TOPIC (Dual Fisheye圧縮画像)"
    echo "  - /tf, /tf_static (座標変換)"
    echo "  - /odom, /odom/filtered (オドメトリ)"
    echo "  - /scan3 (2D LiDAR)"
    if [ "$REQUIRE_CLOCK" = true ]; then
        echo "  - /clock, /imu, /magnetometer"
    else
        echo "  - /imu, /magnetometer, /roboteq/odom"
    fi
    echo ""
    echo "========================================="
    echo -n "ファイル名を入力してください (例: ${MODE}_theta_01): "
    read bag_name

    if [ -z "$bag_name" ]; then
        bag_name="$DEFAULT_BAG_NAME"
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
    echo "  停止するには Ctrl+C を1回押してください"
    echo "  録画中は10秒ごと、停止後は処理段階を表示します"
    echo "========================================="
    echo ""

    ros2 topic pub --once /experiment_metadata std_msgs/msg/String "data: '$experiment_memo'" >/dev/null 2>&1 &

    RECORD_TOPICS=(
        "$THETA_TOPIC"
        /tf /tf_static
        /odom /odom/filtered
        /scan3
        /imu /magnetometer
        /cmd_vel /cmd_vel_nav
        /experiment_metadata
    )
    if [ "$REQUIRE_CLOCK" = true ]; then
        RECORD_TOPICS+=( /clock )
    else
        RECORD_TOPICS+=( /roboteq/odom )
    fi

    ros2 bag record -s mcap -o "$CURRENT_BAG_PATH" "${RECORD_TOPICS[@]}" &

    ROSBAG_PID=$!
    start_recording_heartbeat
    wait $ROSBAG_PID
    stop_recording_heartbeat
}

input_filename
if [ $? -eq 0 ] && [ -n "$selected_name" ]; then
    start_recording "$selected_name"
fi

cleanup 0
