#!/bin/bash
# change_nav_mode.sh: Nav2 MPPI controller 走行モード動的切り替えスクリプト
# 使い方: ./change_nav_mode.sh [normal|safe|strict_normal|strict_safe|strict] (未指定の場合はメニューから選択)

MODE=$1

# 引数がない場合は選択メニューを表示
if [ -z "$MODE" ]; then
    echo "========================================="
    echo " 走行モードを選択してください (Select Navigation Mode)"
    echo "========================================="
    echo "1) 通常走行モード (normal) - 0.9 m/s"
    echo "2) ゆっくり安全歩行モード (safe) - 0.5 m/s"
    echo "3) パス追従優先・通常速度モード (strict_normal) - 0.9 m/s (回避せず待機)"
    echo "4) パス追従優先・ゆっくり速度モード (strict_safe) - 0.5 m/s (回避せず待機)"
    echo "-----------------------------------------"
    read -p "選択してください [1-4]: " CHOICE
    case "$CHOICE" in
        1) MODE="normal" ;;
        2) MODE="safe" ;;
        3) MODE="strict_normal" ;;
        4) MODE="strict_safe" ;;
        *) echo "無効な選択です。終了します。"; exit 1 ;;
    esac
fi

# 互換性のため、strict は strict_safe にリダイレクト
if [ "$MODE" = "strict" ]; then
    MODE="strict_safe"
fi

if [ "$MODE" != "normal" ] && [ "$MODE" != "safe" ] && [ "$MODE" != "strict_normal" ] && [ "$MODE" != "strict_safe" ]; then
    echo "エラー: 走行モードを正しく指定してください。"
    echo "使い方: $0 [normal|safe|strict_normal|strict_safe]"
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
    ros2 param set /controller_server FollowPath.vx_min -0.60
    ros2 param set /controller_server FollowPath.wz_max 0.90
    ros2 param set /controller_server FollowPath.vx_std 0.25
    ros2 param set /controller_server FollowPath.wz_std 0.30
    
    # MPPIコントローラーの加速度制限の設定
    ros2 param set /controller_server FollowPath.ax_max 0.90
    ros2 param set /controller_server FollowPath.ax_min -0.90
    ros2 param set /controller_server FollowPath.az_max 1.50
    
    # 障害物回避の重み（標準値）
    ros2 param set /controller_server FollowPath.CostCritic.cost_weight 10.0
    
    # パス追従の重み（標準値にリセット）
    ros2 param set /controller_server FollowPath.PathAlignCritic.cost_weight 8.0
    ros2 param set /controller_server FollowPath.PathFollowCritic.cost_weight 8.0
    ros2 param set /controller_server FollowPath.PathAngleCritic.cost_weight 2.0
    
    # 旋回抑制の重み（標準値にリセット）
    ros2 param set /controller_server FollowPath.TwirlingCritic.cost_weight 3.0
    
    # 前進優先の重み（標準値にリセット）
    ros2 param set /controller_server FollowPath.PreferForwardCritic.cost_weight 15.0
    
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
    ros2 param set /controller_server FollowPath.vx_max 0.50
    ros2 param set /controller_server FollowPath.vx_min -0.20
    ros2 param set /controller_server FollowPath.wz_max 0.50
    ros2 param set /controller_server FollowPath.vx_std 0.20
    ros2 param set /controller_server FollowPath.wz_std 0.20
    
    # MPPIコントローラーの加速度制限の設定（緩やかな加減速）
    ros2 param set /controller_server FollowPath.ax_max 0.50
    ros2 param set /controller_server FollowPath.ax_min -0.50
    ros2 param set /controller_server FollowPath.az_max 1.00
    
    # 障害物回避の重み（高めて安全性を強化）
    ros2 param set /controller_server FollowPath.CostCritic.cost_weight 15.0
    
    # パス追従の重み（標準値にリセット）
    ros2 param set /controller_server FollowPath.PathAlignCritic.cost_weight 8.0
    ros2 param set /controller_server FollowPath.PathFollowCritic.cost_weight 8.0
    ros2 param set /controller_server FollowPath.PathAngleCritic.cost_weight 2.0
    
    # 旋回抑制の重み（標準値にリセット）
    ros2 param set /controller_server FollowPath.TwirlingCritic.cost_weight 3.0
    
    # 前進優先の重み（標準値にリセット）
    ros2 param set /controller_server FollowPath.PreferForwardCritic.cost_weight 15.0
    
    # 速度スムーサーの同期
    if ros2 node list 2>/dev/null | grep "/velocity_smoother" >/dev/null 2>&1; then
        ros2 param set /velocity_smoother max_velocity "[0.50, 0.0, 0.50]"
        ros2 param set /velocity_smoother min_velocity "[-0.20, 0.0, -0.50]"
        ros2 param set /velocity_smoother max_accel "[0.50, 0.0, 1.00]"
        ros2 param set /velocity_smoother max_decel "[-0.50, 0.0, -1.00]"
    fi

elif [ "$MODE" = "strict_normal" ]; then
    echo "パス追従優先・通常速度モードに設定中..."
    
    # MPPIコントローラーの速度上限と探索分散の設定（通常速度・安全なバックを許可）
    ros2 param set /controller_server FollowPath.vx_max 0.90
    ros2 param set /controller_server FollowPath.vx_min -0.60
    ros2 param set /controller_server FollowPath.wz_max 0.90
    ros2 param set /controller_server FollowPath.vx_std 0.25
    ros2 param set /controller_server FollowPath.wz_std 0.30
    
    # MPPIコントローラーの加速度制限の設定
    ros2 param set /controller_server FollowPath.ax_max 0.90
    ros2 param set /controller_server FollowPath.ax_min -0.90
    ros2 param set /controller_server FollowPath.az_max 1.50
    
    # 障害物回避の重み（しっかり安全に止まる）
    ros2 param set /controller_server FollowPath.CostCritic.cost_weight 15.0
    
    # パス追従の重みを極限まで高め、パスからの逸脱（回避行動）を抑制
    ros2 param set /controller_server FollowPath.PathAlignCritic.cost_weight 35.0
    ros2 param set /controller_server FollowPath.PathFollowCritic.cost_weight 25.0
    ros2 param set /controller_server FollowPath.PathAngleCritic.cost_weight 1.5
    
    # 旋回抑制の重みを高めて無駄な方向転換を抑制
    ros2 param set /controller_server FollowPath.TwirlingCritic.cost_weight 5.0
    
    # 前進優先の重みを大幅に高め、バックに非常に重いペナルティを課す（基本は前進）
    ros2 param set /controller_server FollowPath.PreferForwardCritic.cost_weight 25.0
    
    # 速度スムーサーの同期
    if ros2 node list 2>/dev/null | grep "/velocity_smoother" >/dev/null 2>&1; then
        ros2 param set /velocity_smoother max_velocity "[0.90, 0.0, 0.90]"
        ros2 param set /velocity_smoother min_velocity "[-0.60, 0.0, -0.90]"
        ros2 param set /velocity_smoother max_accel "[0.90, 0.0, 1.50]"
        ros2 param set /velocity_smoother max_decel "[-0.90, 0.0, -1.50]"
    fi

elif [ "$MODE" = "strict_safe" ]; then
    echo "パス追従優先・ゆっくり速度モードに設定中..."
    
    # MPPIコントローラーの速度上限と探索分散の設定（低速・安全なバックを許可）
    ros2 param set /controller_server FollowPath.vx_max 0.50
    ros2 param set /controller_server FollowPath.vx_min -0.20
    ros2 param set /controller_server FollowPath.wz_max 0.50
    ros2 param set /controller_server FollowPath.vx_std 0.20
    ros2 param set /controller_server FollowPath.wz_std 0.20
    
    # MPPIコントローラーの加速度制限の設定（緩やかな加減速）
    ros2 param set /controller_server FollowPath.ax_max 0.50
    ros2 param set /controller_server FollowPath.ax_min -0.50
    ros2 param set /controller_server FollowPath.az_max 1.00
    
    # 障害物回避の重み（しっかり安全に止まる）
    ros2 param set /controller_server FollowPath.CostCritic.cost_weight 15.0
    
    # パス追従の重みを極限まで高め、パスからの逸脱（回避行動）を抑制
    ros2 param set /controller_server FollowPath.PathAlignCritic.cost_weight 35.0
    ros2 param set /controller_server FollowPath.PathFollowCritic.cost_weight 25.0
    ros2 param set /controller_server FollowPath.PathAngleCritic.cost_weight 1.5
    
    # 旋回抑制の重みを高めて無駄な方向転換を抑制
    ros2 param set /controller_server FollowPath.TwirlingCritic.cost_weight 5.0
    
    # 前進優先の重みを大幅に高め、バックに非常に重いペナルティを課す（基本は前進）
    ros2 param set /controller_server FollowPath.PreferForwardCritic.cost_weight 25.0
    
    # 速度スムーサーの同期
    if ros2 node list 2>/dev/null | grep "/velocity_smoother" >/dev/null 2>&1; then
        ros2 param set /velocity_smoother max_velocity "[0.50, 0.0, 0.50]"
        ros2 param set /velocity_smoother min_velocity "[-0.20, 0.0, -0.50]"
        ros2 param set /velocity_smoother max_accel "[0.50, 0.0, 1.00]"
        ros2 param set /velocity_smoother max_decel "[-0.50, 0.0, -1.00]"
    fi
fi

echo "-----------------------------------------"
echo "✓ 走行モード切り替え完了"
echo "========================================="
