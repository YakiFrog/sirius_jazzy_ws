#!/bin/bash
# change_nav_mode.sh: Nav2 MPPI controller 走行モード動的切り替えスクリプト
# 使い方: ./change_nav_mode.sh [normal|safe] (未指定の場合はメニューから選択)

MODE=$1

# 引数がない場合は選択メニューを表示
if [ -z "$MODE" ]; then
    echo "========================================="
    echo " 走行モードを選択してください (Select Navigation Mode)"
    echo "========================================="
    echo "1) 通常走行モード (normal) - 0.9 m/s"
    echo "2) ゆっくり安全歩行モード (safe) - 0.35 m/s"
    echo "-----------------------------------------"
    read -p "選択してください [1-2]: " CHOICE
    case "$CHOICE" in
        1) MODE="normal" ;;
        2) MODE="safe" ;;
        *) echo "無効な選択です。終了します。"; exit 1 ;;
    esac
fi

if [ "$MODE" != "normal" ] && [ "$MODE" != "safe" ]; then
    echo "エラー: 走行モードを正しく指定してください。"
    echo "使い方: $0 [normal|safe]"
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

if [ "$MODE" = "normal" ]; then
    echo "通常走行モードに設定中..."
    
    # MPPIコントローラーの速度上限と探索分散の設定
    ros2 param set /controller_server FollowPath.vx_max 0.90
    ros2 param set /controller_server FollowPath.wz_max 0.90
    ros2 param set /controller_server FollowPath.vx_std 0.25
    ros2 param set /controller_server FollowPath.wz_std 0.30
    
    # MPPIコントローラーの加速度制限の設定
    ros2 param set /controller_server FollowPath.ax_max 0.90
    ros2 param set /controller_server FollowPath.ax_min -0.90
    ros2 param set /controller_server FollowPath.az_max 1.50
    
    # 障害物回避の重み（標準値）
    ros2 param set /controller_server FollowPath.CostCritic.cost_weight 10.0
    
    # 速度スムーサーの同期
    if ros2 node list 2>/dev/null | grep "/velocity_smoother" >/dev/null 2>&1; then
        ros2 param set /velocity_smoother max_velocity "[0.9, 0.0, 0.9]"
        ros2 param set /velocity_smoother min_velocity "[-0.9, 0.0, -0.9]"
        ros2 param set /velocity_smoother max_accel "[0.9, 0.0, 1.5]"
        ros2 param set /velocity_smoother max_decel "[-0.9, 0.0, -1.5]"
    fi
    
elif [ "$MODE" = "safe" ]; then
    echo "ゆっくり安全歩行モードに設定中..."
    
    # MPPIコントローラーの速度上限と探索分散の設定（低速・スムーズ）
    ros2 param set /controller_server FollowPath.vx_max 0.35
    ros2 param set /controller_server FollowPath.wz_max 0.40
    ros2 param set /controller_server FollowPath.vx_std 0.15
    ros2 param set /controller_server FollowPath.wz_std 0.15
    
    # MPPIコントローラーの加速度制限の設定（緩やかな加減速）
    ros2 param set /controller_server FollowPath.ax_max 0.40
    ros2 param set /controller_server FollowPath.ax_min -0.40
    ros2 param set /controller_server FollowPath.az_max 0.80
    
    # 障害物回避の重み（高めて安全性を強化）
    ros2 param set /controller_server FollowPath.CostCritic.cost_weight 18.0
    
    # 速度スムーサーの同期
    if ros2 node list 2>/dev/null | grep "/velocity_smoother" >/dev/null 2>&1; then
        ros2 param set /velocity_smoother max_velocity "[0.35, 0.0, 0.40]"
        ros2 param set /velocity_smoother min_velocity "[-0.35, 0.0, -0.40]"
        ros2 param set /velocity_smoother max_accel "[0.40, 0.0, 0.80]"
        ros2 param set /velocity_smoother max_decel "[-0.40, 0.0, -0.80]"
    fi
fi

echo "-----------------------------------------"
echo "✓ 走行モード切り替え完了"
echo "========================================="
