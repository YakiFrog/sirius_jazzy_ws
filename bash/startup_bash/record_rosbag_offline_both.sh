#!/bin/bash
# ==============================================================================
# オフライン マッピング用 統合録画: ZED(ステレオ) + THETA(デュアル魚眼)
#   - ZED と THETA の両方を1つのbagに記録する
#   - どちらか（または両方）が無くてもOK: 実際に配信中のトピックだけを記録する
#   - SIM(Webシミュレータ) / 実機 の両対応
#
# 使い方:
#   record_rosbag_offline_both.sh [sim|real]
#     sim : /clock を要求し、ZEDトピックが無ければ unity_stereo_bridge を自動起動
#     real: /clock を要求せず、THETAトピックが無ければ theta_capture_node を自動起動
#
# 停止: Ctrl+C を1回。索引保存・（可能なら）検証まで自動で行う。
# ==============================================================================

WS_DIR="${HOME}/sirius_jazzy_ws"
ROSBAG_DIR="${HOME}/rosbag2_data"
mkdir -p "$ROSBAG_DIR"

source "$WS_DIR/install/setup.bash" 2>/dev/null || source /opt/ros/jazzy/setup.bash

ROSBAG_PID=""
HEARTBEAT_PID=""
CAPTURE_PID=""
BRIDGE_PID=""
CURRENT_BAG_PATH=""
RESULT_SHOWN=false
CLEANUP_IN_PROGRESS=false
HAD_ZED=false
HAD_THETA=false
SAM3_CONTAINER="sam3_zed_container"
SAM3_WAS_RUNNING=false
UNITY_BRIDGE_PORT=8080
THETA_TOPIC="/theta/dual_fisheye/image_raw/compressed"
ZED_TOPIC="/camera/stereo_sbs/compressed"
ZED_PARAMS_TOPIC="/camera/stereo_params"
THETA_VALIDATOR="${WS_DIR}/bash/startup_bash/validate_theta_offline_mapping_bag.py"
ZED_VALIDATOR="${WS_DIR}/bash/startup_bash/validate_offline_mapping_bag.py"
experiment_memo="Offline Mapping (ZED+THETA)"

MODE="${1:-sim}"
if [ "$MODE" != "sim" ] && [ "$MODE" != "real" ]; then
    echo "使い方: $0 [sim|real]"
    exit 1
fi
if [ "$MODE" = "real" ]; then
    REQUIRE_CLOCK=false
    MODE_LABEL="実機"
    DEFAULT_BAG_NAME="real_both_mapping_$(date +%Y%m%d_%H%M%S)"
else
    REQUIRE_CLOCK=true
    MODE_LABEL="Webシミュレータ"
    DEFAULT_BAG_NAME="sim_both_mapping_$(date +%Y%m%d_%H%M%S)"
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
        echo "[1/4] ros2 bag recordへ停止要求を送信..."
        kill -INT "$ROSBAG_PID" 2>/dev/null || true
        wait_for_process "$ROSBAG_PID" "Rosbag終了・MCAP索引書き込み"
        wait "$ROSBAG_PID" 2>/dev/null || true
        ROSBAG_PID=""
        echo "  ✓ Rosbag記録プロセスが終了しました"
    fi
    if [ -n "$CAPTURE_PID" ]; then
        echo "[2/4] THETAキャプチャnodeを停止..."
        kill -INT "$CAPTURE_PID" 2>/dev/null || true
        wait "$CAPTURE_PID" 2>/dev/null || true
        CAPTURE_PID=""
    fi
    if [ -n "$BRIDGE_PID" ]; then
        echo "[3/4] unity_stereo_bridgeを停止..."
        kill -INT "$BRIDGE_PID" 2>/dev/null || true
        wait "$BRIDGE_PID" 2>/dev/null || true
        BRIDGE_PID=""
    fi
    restore_sam3_container
    show_result
    echo "================================================="
    echo "✓ すべての録画終了処理が完了しました。"
    echo "================================================="
    exit "$exit_status"
}
trap 'cleanup 130' INT TERM HUP

wait_for_process() {
    local pid="$1" label="$2" elapsed=0 state
    while kill -0 "$pid" 2>/dev/null; do
        state=$(ps -o stat= -p "$pid" 2>/dev/null | tr -d ' ')
        if [[ "$state" == Z* ]] || [ -z "$state" ]; then break; fi
        sleep 1; elapsed=$((elapsed + 1))
        if [ "$elapsed" -eq 1 ] || [ $((elapsed % 3)) -eq 0 ]; then
            echo "  … $label 処理中 (${elapsed}秒)"
        fi
    done
}

run_with_progress() {
    local label="$1"; shift
    "$@" & local pid=$!
    wait_for_process "$pid" "$label"
    wait "$pid" 2>/dev/null
}

recording_heartbeat() {
    local started_at elapsed size
    started_at=$(date +%s)
    while [ -n "$ROSBAG_PID" ] && kill -0 "$ROSBAG_PID" 2>/dev/null; do
        sleep 10
        kill -0 "$ROSBAG_PID" 2>/dev/null || break
        elapsed=$(($(date +%s) - started_at))
        size="準備中"
        [ -d "$CURRENT_BAG_PATH" ] && size=$(du -sh "$CURRENT_BAG_PATH" 2>/dev/null | cut -f1)
        echo "[録画中] ${elapsed}秒 / bag: ${size} / ZED=${HAD_ZED} THETA=${HAD_THETA} / 停止: Ctrl+Cを1回"
    done
}
start_recording_heartbeat(){ recording_heartbeat & HEARTBEAT_PID=$!; }
stop_recording_heartbeat(){ [ -n "$HEARTBEAT_PID" ] && { kill -TERM "$HEARTBEAT_PID" 2>/dev/null; wait "$HEARTBEAT_PID" 2>/dev/null; HEARTBEAT_PID=""; }; }

restore_sam3_container() {
    if [ "$SAM3_WAS_RUNNING" = true ]; then
        echo "SAM3 Dockerを録画前の状態へ復帰..."
        docker start "$SAM3_CONTAINER" >/dev/null && echo "  ✓ $SAM3_CONTAINER を再起動" || echo "  ✗ 再起動失敗"
        SAM3_WAS_RUNNING=false
    fi
}

# SIM: ZEDトピックが無いときだけ unity_stereo_bridge を起動（port競合はSAM3を一時停止）
maybe_start_zed_bridge() {
    ros2 topic list 2>/dev/null | grep -qx "$ZED_TOPIC" && return 0
    [ "$MODE" = "sim" ] || return 0
    echo "ZEDトピックが無いため unity_stereo_bridge (port ${UNITY_BRIDGE_PORT}) を起動します..."
    if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx "$SAM3_CONTAINER"; then
        if docker port "$SAM3_CONTAINER" 2>/dev/null | grep -q ":${UNITY_BRIDGE_PORT}$"; then
            echo "  ポート競合のため $SAM3_CONTAINER を録画中だけ停止します..."
            docker stop -t 10 "$SAM3_CONTAINER" >/dev/null && SAM3_WAS_RUNNING=true
        fi
    fi
    if ss -H -ltn "sport = :${UNITY_BRIDGE_PORT}" 2>/dev/null | grep -q .; then
        echo "  エラー: ポート${UNITY_BRIDGE_PORT}使用中。ZED bridgeを起動できません。"
        return 0
    fi
    ros2 run sirius_navigation unity_stereo_bridge --ros-args -p port:="$UNITY_BRIDGE_PORT" &
    BRIDGE_PID=$!
    sleep 2
    kill -0 "$BRIDGE_PID" 2>/dev/null || { echo "  ✗ bridge起動失敗"; BRIDGE_PID=""; }
}

# REAL: THETAトピックが無いときだけ theta_capture_node を起動
maybe_start_theta_capture() {
    ros2 topic list 2>/dev/null | grep -qx "$THETA_TOPIC" && return 0
    [ "$MODE" = "real" ] || return 0
    echo "THETAトピックが無いため theta_capture_node を起動します..."
    ros2 run sirius_navigation theta_capture_node --ros-args \
        -p device:=/dev/theta_capture -p fourcc:=YUYV -p fps:=5.0 &
    CAPTURE_PID=$!
    sleep 3
    kill -0 "$CAPTURE_PID" 2>/dev/null || { echo "  ✗ THETAキャプチャ起動失敗"; CAPTURE_PID=""; }
}

select_topics() {
    local have
    have=$(ros2 topic list 2>/dev/null)
    local candidates=(
        "$ZED_TOPIC" "$ZED_PARAMS_TOPIC" "$THETA_TOPIC"
        /tf /tf_static /odom /odom/filtered /scan3
        /imu /magnetometer /cmd_vel /cmd_vel_nav /experiment_metadata
    )
    if [ "$REQUIRE_CLOCK" = true ]; then candidates+=( /clock ); else candidates+=( /roboteq/odom ); fi

    RECORD_TOPICS=(); SKIPPED_TOPICS=()
    for t in "${candidates[@]}"; do
        if printf '%s\n' "$have" | grep -qx "$t"; then RECORD_TOPICS+=("$t"); else SKIPPED_TOPICS+=("$t"); fi
    done

    printf '%s\n' "$have" | grep -qx "$ZED_TOPIC" && HAD_ZED=true
    printf '%s\n' "$have" | grep -qx "$THETA_TOPIC" && HAD_THETA=true
}

preflight() {
    echo ""
    echo "========================================="
    echo "録画開始前チェック [$MODE_LABEL]"
    echo "========================================="
    select_topics
    echo "  記録するトピック (${#RECORD_TOPICS[@]}):"
    for t in "${RECORD_TOPICS[@]}"; do echo "    + $t"; done
    if [ "${#SKIPPED_TOPICS[@]}" -gt 0 ]; then
        echo "  無いためスキップ (どちらかが無くてもOK):"
        for t in "${SKIPPED_TOPICS[@]}"; do echo "    - $t"; done
    fi
    echo "  ZED:   $([ "$HAD_ZED" = true ] && echo あり || echo なし)"
    echo "  THETA: $([ "$HAD_THETA" = true ] && echo あり || echo なし)"
    echo "========================================="
    if [ "$HAD_ZED" = false ] && [ "$HAD_THETA" = false ]; then
        echo "✗ ZED/THETA どちらも配信がありません。映像源を起動してください。"
        return 1
    fi
    if ! ros2 topic list 2>/dev/null | grep -qx /tf; then
        echo "✗ /tf がありません（SLAM Toolbox等を起動してください）。"
        return 1
    fi
    echo "✓ 録画を開始できます。"
    return 0
}

show_result() {
    [ "$RESULT_SHOWN" = true ] && return
    RESULT_SHOWN=true
    if [ -z "$CURRENT_BAG_PATH" ]; then
        echo "録画データは作成されませんでした。"; return
    fi
    echo ""
    echo "========================================="
    echo "✓ 録画が完了しました: $CURRENT_BAG_PATH"
    if [ -d "$CURRENT_BAG_PATH" ]; then
        local mcap_file; mcap_file=$(find "$CURRENT_BAG_PATH" -name "*.mcap" | head -n 1)
        if [ -n "$mcap_file" ] && [ -f "${WS_DIR}/mcap" ]; then
            local tmp="${mcap_file}.tmp"
            run_with_progress "MCAPインデックス修復" "${WS_DIR}/mcap" recover "$mcap_file" -o "$tmp" \
                && mv "$tmp" "$mcap_file" || rm -f "$tmp"
        fi
        if [ ! -f "$CURRENT_BAG_PATH/metadata.yaml" ]; then
            run_with_progress "Rosbagメタデータ生成" ros2 bag reindex "$CURRENT_BAG_PATH" -s mcap
        fi
        echo "  データサイズ: $(du -sh "$CURRENT_BAG_PATH" | cut -f1)"
        echo "  録画トピック: ZED=$HAD_ZED THETA=$HAD_THETA"
        echo ""
        echo "録画内容を検証..."
        if [ "$HAD_THETA" = true ] && [ -f "$THETA_VALIDATOR" ]; then
            if [ "$REQUIRE_CLOCK" = true ]; then
                run_with_progress "THETA検証" python3 "$THETA_VALIDATOR" "$CURRENT_BAG_PATH" --require-clock || true
            else
                run_with_progress "THETA検証" python3 "$THETA_VALIDATOR" "$CURRENT_BAG_PATH" || true
            fi
        fi
        if [ "$HAD_ZED" = true ] && [ -f "$ZED_VALIDATOR" ]; then
            run_with_progress "ZED検証" python3 "$ZED_VALIDATOR" "$CURRENT_BAG_PATH" || true
        fi
    fi
    echo "========================================="
}

input_filename() {
    echo ""
    echo "========================================="
    echo -n "ファイル名を入力してください (例: ${MODE}_campus_01): "
    read bag_name
    [ -z "$bag_name" ] && { bag_name="$DEFAULT_BAG_NAME"; echo "デフォルト名: $bag_name"; }
    echo -n "実験内容のメモ (任意): "
    read input_memo
    [ -n "$input_memo" ] && experiment_memo="$input_memo"
    if [ -d "$ROSBAG_DIR/$bag_name" ]; then
        echo "警告: $bag_name は既に存在します"
        echo -n "上書きしますか? (y/N): "
        read overwrite
        [[ ! "$overwrite" =~ ^[Yy]$ ]] && { echo "キャンセル"; return 1; }
        rm -rf "$ROSBAG_DIR/$bag_name"
    fi
    selected_name="$bag_name"
}

start_recording() {
    CURRENT_BAG_PATH="$ROSBAG_DIR/$1"
    RESULT_SHOWN=false
    echo "========================================="
    echo "記録開始: $CURRENT_BAG_PATH"
    echo "  停止: Ctrl+C を1回"
    echo "========================================="
    ros2 topic pub --once /experiment_metadata std_msgs/msg/String "data: '$experiment_memo'" >/dev/null 2>&1 &
    ros2 bag record -s mcap -o "$CURRENT_BAG_PATH" "${RECORD_TOPICS[@]}" &
    ROSBAG_PID=$!
    start_recording_heartbeat
    wait $ROSBAG_PID
    stop_recording_heartbeat
}

echo "========================================="
echo "  統合オフライン録画 (ZED + THETA) [$MODE_LABEL]"
echo "========================================="
echo "※ SLAM Toolbox等は録画前に起動しておいてください。"

maybe_start_zed_bridge
maybe_start_theta_capture

if ! preflight; then
    cleanup 1
fi

input_filename
if [ $? -eq 0 ] && [ -n "$selected_name" ]; then
    start_recording "$selected_name"
fi
cleanup 0
