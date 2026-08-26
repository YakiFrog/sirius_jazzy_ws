#!/bin/bash
# change_nav_mode.sh: Nav2 MPPI controller 走行モード動的切り替えスクリプト (一括設定高速版)
# 使い方: ./change_nav_mode.sh [normal|normal_active|safe|slow|wait_normal|strict_normal|strict_safe|strict_slow|strict] (未指定の場合はメニューから選択)

MODE=$1

# 引数がない場合は選択メニューを表示
if [ -z "$MODE" ]; then
    echo "========================================="
    echo " 走行モードを選択してください (Select Navigation Mode)"
    echo "========================================="
    echo "1) 通常走行モード (normal) - 0.9 m/s"
    echo "2) 通常走行・探索強化モード (normal_active) - 1.0 m/s (探索広め・加速強め・安全重視)"
    echo "3) ゆっくり安全歩行モード (safe) - 0.4 m/s"
    echo "4) 超低速安全歩行モード (slow) - 0.2 m/s"
    echo "5) 待機優先モード (wait_normal) - 0.9 m/s (障害物前で停止・再試行)"
    echo "6) パス追従優先・通常速度モード (strict_normal) - 0.9 m/s"
    echo "7) パス追従優先・ゆっくり速度モード (strict_safe) - 0.4 m/s"
    echo "8) パス追従優先・超低速速度モード (strict_slow) - 0.2 m/s"
    echo "-----------------------------------------"
    read -p "選択してください [1-8]: " CHOICE
    case "$CHOICE" in
        1) MODE="normal" ;;
        2) MODE="normal_active" ;;
        3) MODE="safe" ;;
        4) MODE="slow" ;;
        5) MODE="wait_normal" ;;
        6) MODE="strict_normal" ;;
        7) MODE="strict_safe" ;;
        8) MODE="strict_slow" ;;
        *) echo "無効な選択です。終了します。"; exit 1 ;;
    esac
fi

# 互換性のため、strict は strict_safe にリダイレクト
if [ "$MODE" = "strict" ]; then
    MODE="strict_safe"
fi

if [ "$MODE" != "normal" ] && [ "$MODE" != "normal_active" ] && [ "$MODE" != "safe" ] && [ "$MODE" != "slow" ] && [ "$MODE" != "wait_normal" ] && [ "$MODE" != "strict_normal" ] && [ "$MODE" != "strict_safe" ] && [ "$MODE" != "strict_slow" ]; then
    echo "エラー: 走行モードを正しく指定してください。"
    echo "使い方: $0 [normal|normal_active|safe|slow|wait_normal|strict_normal|strict_safe|strict_slow]"
    exit 1
fi

echo "========================================="
echo " Nav2 MPPI 走行モード切り替え: $MODE"
echo "========================================="

# ROS 2環境のセットアップ確認
if ! command -v ros2 &> /dev/null; then
    source /opt/ros/jazzy/setup.bash 2>/dev/null
    source ~/sirius_jazzy_ws/install/setup.bash 2>/dev/null
fi

# ノード起動確認
if ! ros2 node list 2>/dev/null | grep "/controller_server" >/dev/null 2>&1; then
    echo "[警告] /controller_server ノードが起動していません。Nav2を起動した状態で実行してください。"
    exit 1
fi

# Python一括設定スクリプトを呼び出し
python3 ~/sirius_jazzy_ws/bash/startup_bash/change_nav_mode_fast.py "$MODE"

echo "========================================="
