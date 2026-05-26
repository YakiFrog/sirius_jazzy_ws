#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、終了します"' 2
cd ~/sirius_jazzy_ws
source install/setup.bash

LOG_FILE="/tmp/sirius_target_follower.log"
FOLLOWER_NODE="target_follower"

is_node_running() {
    ros2 node list 2>/dev/null | grep "/$FOLLOWER_NODE" >/dev/null 2>&1
    return $?
}

start_follower() {
    # 1. 起動していない場合はバックグラウンドで起動
    if ! is_node_running; then
        echo "ターゲットトラッカーを起動しています... (ログ: $LOG_FILE)"
        ros2 run sirius_navigation target_follower > "$LOG_FILE" 2>&1 &
        
        # 起動を最大5秒間確認
        echo -n "起動確認中"
        for i in {1..5}; do
            sleep 1
            echo -n "."
            if is_node_running; then
                break
            fi
        done
        echo ""
    fi

    # 2. パラメータをenableにする
    if is_node_running; then
        echo "追従を開始（有効化）しています..."
        ros2 param set /target_follower enable_following true > /dev/null 2>&1
        echo "✓ 追従が開始されました。"
    else
        echo "✗ トラッカーノードの起動に失敗しました。ログを確認してください:"
        cat "$LOG_FILE"
    fi
}

pause_follower() {
    if ! is_node_running; then
        echo "トラッカーノードは起動していません。"
        return
    fi
    echo "追従を一時停止（無効化）しています..."
    ros2 param set /target_follower enable_following false > /dev/null 2>&1
    echo "✓ 追従を一時停止しました（ロボットは停止します）。"
}

stop_node_completely() {
    if is_node_running; then
        echo "トラッカーノードを完全に停止しています..."
        pkill -f "target_follower"
        sleep 1
    fi
}

set_distance() {
    if ! is_node_running; then
        echo "エラー: トラッカーノードが起動していません。まず [1] で開始してください。"
        return
    fi
    
    echo -n "追従距離を入力してください (メートル、例: 1.2): "
    read dist
    if [[ ! "$dist" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
        echo "エラー: 無効な数値です。"
        return
    fi
    ros2 param set /target_follower follow_distance "$dist" > /dev/null 2>&1
}

show_status() {
    echo "----------------------------------------"
    if is_node_running; then
        local enable_val=$(ros2 param get /target_follower enable_following 2>/dev/null | awk '{print $NF}')
        local dist_val=$(ros2 param get /target_follower follow_distance 2>/dev/null | awk '{print $NF}')
        
        if [ "$enable_val" = "True" ]; then
            echo "  ステータス: 追従中 (FOLLOWING) - 移動コマンド送信中"
        else
            echo "  ステータス: 一時停止中 (PAUSED) - ロボット停止中"
        fi
        echo "  維持距離: ${dist_val}m"
    else
        echo "  ステータス: 停止中 (STOPPED)"
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
    echo " [4] ログのリアルタイム表示 (Tail Log)"
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
            tail -f "$LOG_FILE"
            ;;
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
