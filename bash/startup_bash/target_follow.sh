#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、終了します"' 2
cd ~/sirius_jazzy_ws
source install/setup.bash

LOG_FOLLOWER="/tmp/sirius_target_follower.log"
LOG_DETECTOR="/tmp/sirius_target_detector.log"

FOLLOWER_NODE="target_follower"
DETECTOR_NODE="target_detector"

# NPCの手動/自動モード状態管理
NPC_MANUAL_MODE=false

is_node_running() {
    local node_name=$1
    ros2 node list 2>/dev/null | grep "/$node_name" >/dev/null 2>&1
    return $?
}

start_follower() {
    local started_any=false
    
    # 1. 検出ノードの起動
    if ! is_node_running "$DETECTOR_NODE"; then
        echo "ターゲット検出ノード (LiDAR追跡) を起動しています... (ログ: $LOG_DETECTOR)"
        ros2 run sirius_navigation target_detector > "$LOG_DETECTOR" 2>&1 &
        started_any=true
    fi

    # 2. 追従ノードの起動
    if ! is_node_running "$FOLLOWER_NODE"; then
        echo "ターゲット追従ノードを起動しています... (ログ: $LOG_FOLLOWER)"
        ros2 run sirius_navigation target_follower > "$LOG_FOLLOWER" 2>&1 &
        started_any=true
    fi
    
    # 起動確認
    if [ "$started_any" = true ]; then
        echo -n "ノードの起動確認中"
        for i in {1..5}; do
            sleep 1
            echo -n "."
            if is_node_running "$DETECTOR_NODE" && is_node_running "$FOLLOWER_NODE"; then
                break
            fi
        done
        echo ""
    fi

    # 3. パラメータをenableにする
    if is_node_running "$DETECTOR_NODE" && is_node_running "$FOLLOWER_NODE"; then
        echo "追従を開始（有効化）しています..."
        ros2 param set /target_follower enable_following true > /dev/null 2>&1
        echo "✓ 追従が開始されました。LiDARでの人検出および追従を行います。"
    else
        echo "✗ ノードの起動確認に失敗しました。ログを確認してください:"
        if ! is_node_running "$DETECTOR_NODE"; then
            echo "--- 検出ノードログ ($LOG_DETECTOR) ---"
            tail -n 10 "$LOG_DETECTOR"
        fi
        if ! is_node_running "$FOLLOWER_NODE"; then
            echo "--- 追従ノードログ ($LOG_FOLLOWER) ---"
            tail -n 10 "$LOG_FOLLOWER"
        fi
    fi
}

pause_follower() {
    if ! is_node_running "$FOLLOWER_NODE"; then
        echo "追従ノードは起動していません。"
        return
    fi
    echo "追従を一時停止（無効化）しています..."
    ros2 param set /target_follower enable_following false > /dev/null 2>&1
    echo "✓ 追従を一時停止しました（ロボットは停止します）。"
}

stop_node_completely() {
    echo "追従システムを完全に停止しています..."
    if is_node_running "$FOLLOWER_NODE"; then
        pkill -f "target_follower"
    fi
    if is_node_running "$DETECTOR_NODE"; then
        pkill -f "target_detector"
    fi
    sleep 1
}

set_distance() {
    if ! is_node_running "$FOLLOWER_NODE"; then
        echo "エラー: 追従ノードが起動していません。まず [1] で開始してください。"
        return
    fi
    
    echo -n "追従距離を入力してください (メートル、例: 0.5): "
    read dist
    if [[ ! "$dist" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
        echo "エラー: 無効な数値です。"
        return
    fi
    ros2 param set /target_follower follow_distance "$dist" > /dev/null 2>&1
}

toggle_npc_mode() {
    if [ "$NPC_MANUAL_MODE" = "false" ]; then
        echo "NPCを手動目標モードに切り替えています... (自動徘徊を停止)"
        ros2 topic pub -1 /npc/manual_mode std_msgs/msg/Bool "{data: true}" > /dev/null 2>&1
        NPC_MANUAL_MODE=true
        echo "✓ NPCを手動目標モードに設定しました。RViz2の 'Publish Point' または '2D Goal Pose' を使用してNPC_0を目標地点へ動かせます。"
    else
        echo "NPCを自動徘徊モードに切り替えています..."
        ros2 topic pub -1 /npc/manual_mode std_msgs/msg/Bool "{data: false}" > /dev/null 2>&1
        NPC_MANUAL_MODE=false
        echo "✓ NPCの自動徘徊モードを再開しました。"
    fi
}

show_status() {
    echo "----------------------------------------"
    local f_run=false
    local d_run=false
    is_node_running "$FOLLOWER_NODE" && f_run=true
    is_node_running "$DETECTOR_NODE" && d_run=true

    if [ "$f_run" = true ] && [ "$d_run" = true ]; then
        local enable_val=$(ros2 param get /target_follower enable_following 2>/dev/null | awk '{print $NF}')
        local dist_val=$(ros2 param get /target_follower follow_distance 2>/dev/null | awk '{print $NF}')
        
        if [ "$enable_val" = "True" ]; then
            echo "  追従ステータス: 追従中 (FOLLOWING) - LiDAR追跡中"
        else
            echo "  追従ステータス: 一時停止中 (PAUSED) - ロボット停止中"
        fi
        echo "  維持距離: ${dist_val}m"
    elif [ "$f_run" = true ] || [ "$d_run" = true ]; then
        echo "  追従ステータス: エラー (一部のノードのみ起動中)"
        [ "$f_run" = true ] && echo "  - 追従ノード (target_follower): 起動中"
        [ "$f_run" = false ] && echo "  - 追従ノード (target_follower): 停止中"
        [ "$d_run" = true ] && echo "  - 検出ノード (target_detector): 起動中"
        [ "$d_run" = false ] && echo "  - 検出ノード (target_detector): 停止中"
    else
        echo "  追従ステータス: 停止中 (STOPPED)"
    fi

    if [ "$NPC_MANUAL_MODE" = "true" ]; then
        echo "  NPC移動モード : 手動目標モード (MANUAL) - RViz2で操作可能"
    else
        echo "  NPC移動モード : 自動徘徊モード (AUTO)"
    fi
    echo "----------------------------------------"
}

# 起動時に一度ステータスを確認
show_status

while :; do
    echo "=== ターゲット追従制御メニュー ==="
    echo " [1] 追従を開始する (Start Following)"
    echo " [2] 追従を一時停止・無効化する (Pause / Disable)"
    echo " [3] 追従距離の変更 (Set Distance)"
    echo " [4] 追従ノードログの表示 (Tail Follower Log)"
    echo " [5] 検出ノードログの表示 (Tail Detector Log)"
    echo " [6] NPC自動移動（徘徊）の有効/無効切り替え"
    echo " [q] メニュー終了 (ノードも停止)"
    echo "================================="
    echo -n "選択してください: "
    read selection

    case "$selection" in
        1) start_follower ;;
        2) pause_follower ;;
        3) set_distance ;;
        4) 
            echo "Ctrl+C でログ表示を抜けます..."
            tail -f "$LOG_FOLLOWER"
            ;;
        5) 
            echo "Ctrl+C でログ表示を抜けます..."
            tail -f "$LOG_DETECTOR"
            ;;
        6) toggle_npc_mode ;;
        q|Q) 
            stop_node_completely
            echo "終了します。"
            break 
            ;;
        *) echo "無効な選択です。" ;;
    esac
    echo ""
    show_status
done
