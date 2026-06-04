#!/bin/bash
# change_nav_mode.sh: Nav2 MPPI controller 走行モード動的切り替えスクリプト
# 使い方: ./change_nav_mode.sh [normal|normal_active|safe|slow|strict_normal|strict_safe|strict_slow|strict] (未指定の場合はメニューから選択)

MODE=$1

# 引数がない場合は選択メニューを表示
if [ -z "$MODE" ]; then
    echo "========================================="
    echo " 走行モードを選択してください (Select Navigation Mode)"
    echo "========================================="
    echo "1) 通常走行モード (normal) - 0.9 m/s"
    echo "2) 通常走行・探索強化モード (normal_active) - 0.9 m/s (探索広め・安全重視)"
    echo "3) ゆっくり安全歩行モード (safe) - 0.4 m/s"
    echo "4) 超低速安全歩行モード (slow) - 0.2 m/s"
    echo "5) パス追従優先・通常速度モード (strict_normal) - 0.9 m/s (回避せず待機)"
    echo "6) パス追従優先・ゆっくり速度モード (strict_safe) - 0.4 m/s (回避せず待機)"
    echo "7) パス追従優先・超低速速度モード (strict_slow) - 0.2 m/s (回避せず待機)"
    echo "-----------------------------------------"
    read -p "選択してください [1-7]: " CHOICE
    case "$CHOICE" in
        1) MODE="normal" ;;
        2) MODE="normal_active" ;;
        3) MODE="safe" ;;
        4) MODE="slow" ;;
        5) MODE="strict_normal" ;;
        6) MODE="strict_safe" ;;
        7) MODE="strict_slow" ;;
        *) echo "無効な選択です。終了します。"; exit 1 ;;
    esac
fi

# 互換性のため、strict は strict_safe にリダイレクト
if [ "$MODE" = "strict" ]; then
    MODE="strict_safe"
fi

if [ "$MODE" != "normal" ] && [ "$MODE" != "normal_active" ] && [ "$MODE" != "safe" ] && [ "$MODE" != "slow" ] && [ "$MODE" != "strict_normal" ] && [ "$MODE" != "strict_safe" ] && [ "$MODE" != "strict_slow" ]; then
    echo "エラー: 走行モードを正しく指定してください。"
    echo "使い方: $0 [normal|normal_active|safe|slow|strict_normal|strict_safe|strict_slow]"
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
    
    # ゴールへの進行・向きの重み（標準値にリセット）
    ros2 param set /controller_server FollowPath.GoalCritic.cost_weight 3.0
    ros2 param set /controller_server FollowPath.GoalAngleCritic.cost_weight 1.0
    
    # 速度スムーサーの同期
    if ros2 node list 2>/dev/null | grep "/velocity_smoother" >/dev/null 2>&1; then
        ros2 param set /velocity_smoother max_velocity "[0.9, 0.0, 0.9]"
        ros2 param set /velocity_smoother min_velocity "[-0.9, 0.0, -0.9]"
        ros2 param set /velocity_smoother max_accel "[0.9, 0.0, 1.5]"
        ros2 param set /velocity_smoother max_decel "[-0.9, 0.0, -1.5]"
    fi
    
    # グローバルコストマップの障害物レイヤーを有効化 (回避ルート算出を許可)
    if ros2 node list 2>/dev/null | grep "/global_costmap" >/dev/null 2>&1; then
        ros2 param set /global_costmap/global_costmap obstacle_layer.enabled True
    fi

elif [ "$MODE" = "normal_active" ]; then
    echo "通常走行・探索強化モードに設定中..."
    
    # MPPIコントローラーの速度上限と探索分散の設定 (探索ノイズを広げて経路開拓能力を強化)
    ros2 param set /controller_server FollowPath.vx_max 0.90
    ros2 param set /controller_server FollowPath.vx_min -0.60
    ros2 param set /controller_server FollowPath.wz_max 0.90
    ros2 param set /controller_server FollowPath.vx_std 0.38
    ros2 param set /controller_server FollowPath.wz_std 0.45
    
    # MPPIコントローラーの加速度制限の設定
    ros2 param set /controller_server FollowPath.ax_max 0.90
    ros2 param set /controller_server FollowPath.ax_min -0.90
    ros2 param set /controller_server FollowPath.az_max 1.50
    
    # 障害物回避の重み (ノイズが大きくても安全領域を通るように重みを少し高めに設定)
    ros2 param set /controller_server FollowPath.CostCritic.cost_weight 15.0
    
    # パス追従の重み
    ros2 param set /controller_server FollowPath.PathAlignCritic.cost_weight 8.0
    ros2 param set /controller_server FollowPath.PathFollowCritic.cost_weight 8.0
    ros2 param set /controller_server FollowPath.PathAngleCritic.cost_weight 2.0
    
    # 旋回抑制の重み
    ros2 param set /controller_server FollowPath.TwirlingCritic.cost_weight 3.0
    
    # 前進優先の重み
    ros2 param set /controller_server FollowPath.PreferForwardCritic.cost_weight 15.0
    
    # ゴールへの進行・向きの重み
    ros2 param set /controller_server FollowPath.GoalCritic.cost_weight 3.0
    ros2 param set /controller_server FollowPath.GoalAngleCritic.cost_weight 1.0
    
    # 速度スムーサーの同期
    if ros2 node list 2>/dev/null | grep "/velocity_smoother" >/dev/null 2>&1; then
        ros2 param set /velocity_smoother max_velocity "[0.9, 0.0, 0.9]"
        ros2 param set /velocity_smoother min_velocity "[-0.9, 0.0, -0.9]"
        ros2 param set /velocity_smoother max_accel "[0.9, 0.0, 1.5]"
        ros2 param set /velocity_smoother max_decel "[-0.9, 0.0, -1.5]"
    fi
    
    # グローバルコストマップの障害物レイヤーを有効化
    if ros2 node list 2>/dev/null | grep "/global_costmap" >/dev/null 2>&1; then
        ros2 param set /global_costmap/global_costmap obstacle_layer.enabled True
    fi
    
elif [ "$MODE" = "safe" ]; then
    echo "ゆっくり安全歩行モードに設定中..."
    
    # MPPIコントローラーの速度上限と探索分散の設定（低速・スムーズ）
    ros2 param set /controller_server FollowPath.vx_max 0.40
    ros2 param set /controller_server FollowPath.vx_min -0.20
    ros2 param set /controller_server FollowPath.wz_max 0.40
    ros2 param set /controller_server FollowPath.vx_std 0.20
    ros2 param set /controller_server FollowPath.wz_std 0.20
    
    # MPPIコントローラーの加速度制限の設定（緩やかな加減速）
    ros2 param set /controller_server FollowPath.ax_max 0.40
    ros2 param set /controller_server FollowPath.ax_min -0.40
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
    
    # ゴールへの進行・向きの重み（標準値にリセット）
    ros2 param set /controller_server FollowPath.GoalCritic.cost_weight 3.0
    ros2 param set /controller_server FollowPath.GoalAngleCritic.cost_weight 1.0
    
    # 速度スムーサーの同期
    if ros2 node list 2>/dev/null | grep "/velocity_smoother" >/dev/null 2>&1; then
        ros2 param set /velocity_smoother max_velocity "[0.40, 0.0, 0.40]"
        ros2 param set /velocity_smoother min_velocity "[-0.20, 0.0, -0.40]"
        ros2 param set /velocity_smoother max_accel "[0.40, 0.0, 1.00]"
        ros2 param set /velocity_smoother max_decel "[-0.40, 0.0, -1.00]"
    fi
    
    # グローバルコストマップの障害物レイヤーを有効化 (回避ルート算出を許可)
    if ros2 node list 2>/dev/null | grep "/global_costmap" >/dev/null 2>&1; then
        ros2 param set /global_costmap/global_costmap obstacle_layer.enabled True
    fi

elif [ "$MODE" = "slow" ]; then
    echo "超低速安全歩行モードに設定中..."
    
    # MPPIコントローラーの速度上限と探索分散の設定（極めて低速・高精度）
    ros2 param set /controller_server FollowPath.vx_max 0.20
    ros2 param set /controller_server FollowPath.vx_min -0.10
    ros2 param set /controller_server FollowPath.wz_max 0.20
    ros2 param set /controller_server FollowPath.vx_std 0.05
    ros2 param set /controller_server FollowPath.wz_std 0.05
    
    # MPPIコントローラーの加速度制限の設定（極めて緩やかな加減速）
    ros2 param set /controller_server FollowPath.ax_max 0.20
    ros2 param set /controller_server FollowPath.ax_min -0.20
    ros2 param set /controller_server FollowPath.az_max 0.50
    
    # 障害物回避の重み（高めて安全性を最大化）
    ros2 param set /controller_server FollowPath.CostCritic.cost_weight 20.0
    
    # パス追従の重み（標準値にリセット）
    ros2 param set /controller_server FollowPath.PathAlignCritic.cost_weight 8.0
    ros2 param set /controller_server FollowPath.PathFollowCritic.cost_weight 8.0
    ros2 param set /controller_server FollowPath.PathAngleCritic.cost_weight 2.0
    
    # 旋回抑制の重み（標準値にリセット）
    ros2 param set /controller_server FollowPath.TwirlingCritic.cost_weight 3.0
    
    # 前進優先の重み（標準値にリセット）
    ros2 param set /controller_server FollowPath.PreferForwardCritic.cost_weight 15.0
    
    # ゴールへの進行・向きの重み（標準値にリセット）
    ros2 param set /controller_server FollowPath.GoalCritic.cost_weight 3.0
    ros2 param set /controller_server FollowPath.GoalAngleCritic.cost_weight 1.0
    
    # 速度スムーサーの同期
    if ros2 node list 2>/dev/null | grep "/velocity_smoother" >/dev/null 2>&1; then
        ros2 param set /velocity_smoother max_velocity "[0.20, 0.0, 0.20]"
        ros2 param set /velocity_smoother min_velocity "[-0.10, 0.0, -0.20]"
        ros2 param set /velocity_smoother max_accel "[0.20, 0.0, 0.50]"
        ros2 param set /velocity_smoother max_decel "[-0.20, 0.0, -0.50]"
    fi
    
    # グローバルコストマップの障害物レイヤーを有効化 (回避ルート算出を許可)
    if ros2 node list 2>/dev/null | grep "/global_costmap" >/dev/null 2>&1; then
        ros2 param set /global_costmap/global_costmap obstacle_layer.enabled True
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
    
    # パス追従の重みを極限まで高め、かつ前進の引っ張り（PathFollow）を抑えて回避行動を抑制
    ros2 param set /controller_server FollowPath.PathAlignCritic.cost_weight 60.0
    ros2 param set /controller_server FollowPath.PathFollowCritic.cost_weight 3.0
    ros2 param set /controller_server FollowPath.PathAngleCritic.cost_weight 1.5
    
    # 旋回抑制の重みを高めて無駄な方向転換を抑制
    ros2 param set /controller_server FollowPath.TwirlingCritic.cost_weight 5.0
    
    # ゴールへの進行・向きの重みを下げて障害物前での立ち往生/待機を優先
    ros2 param set /controller_server FollowPath.GoalCritic.cost_weight 0.5
    ros2 param set /controller_server FollowPath.GoalAngleCritic.cost_weight 0.5
    
    # 前進優先の重みを大幅に高め、バックに非常に重いペナルティを課す（基本は前進）
    ros2 param set /controller_server FollowPath.PreferForwardCritic.cost_weight 25.0
    
    # 速度スムーサーの同期
    if ros2 node list 2>/dev/null | grep "/velocity_smoother" >/dev/null 2>&1; then
        ros2 param set /velocity_smoother max_velocity "[0.90, 0.0, 0.90]"
        ros2 param set /velocity_smoother min_velocity "[-0.60, 0.0, -0.90]"
        ros2 param set /velocity_smoother max_accel "[0.90, 0.0, 1.50]"
        ros2 param set /velocity_smoother max_decel "[-0.90, 0.0, -1.50]"
    fi
    
    # グローバルコストマップの障害物レイヤーを無効化 (回避ルートの算出を禁止し立ち往生/待機させる)
    if ros2 node list 2>/dev/null | grep "/global_costmap" >/dev/null 2>&1; then
        ros2 param set /global_costmap/global_costmap obstacle_layer.enabled False
    fi

elif [ "$MODE" = "strict_safe" ]; then
    echo "パス追従優先・ゆっくり速度モードに設定中..."
    
    # MPPIコントローラーの速度上限と探索分散の設定（低速・安全なバックを許可）
    ros2 param set /controller_server FollowPath.vx_max 0.40
    ros2 param set /controller_server FollowPath.vx_min -0.20
    ros2 param set /controller_server FollowPath.wz_max 0.40
    ros2 param set /controller_server FollowPath.vx_std 0.20
    ros2 param set /controller_server FollowPath.wz_std 0.20
    
    # MPPIコントローラーの加速度制限の設定（緩やかな加減速）
    ros2 param set /controller_server FollowPath.ax_max 0.40
    ros2 param set /controller_server FollowPath.ax_min -0.40
    ros2 param set /controller_server FollowPath.az_max 1.00
    
    # 障害物回避の重み（しっかり安全に止まる）
    ros2 param set /controller_server FollowPath.CostCritic.cost_weight 15.0
    
    # パス追従の重みを極限まで高め、かつ前進の引っ張り（PathFollow）を抑えて回避行動を抑制
    ros2 param set /controller_server FollowPath.PathAlignCritic.cost_weight 60.0
    ros2 param set /controller_server FollowPath.PathFollowCritic.cost_weight 3.0
    ros2 param set /controller_server FollowPath.PathAngleCritic.cost_weight 1.5
    
    # 旋回抑制の重みを高めて無駄な方向転換を抑制
    ros2 param set /controller_server FollowPath.TwirlingCritic.cost_weight 5.0
    
    # ゴールへの進行・向きの重みを下げて障害物前での立ち往生/待機を優先
    ros2 param set /controller_server FollowPath.GoalCritic.cost_weight 0.5
    ros2 param set /controller_server FollowPath.GoalAngleCritic.cost_weight 0.5
    
    # 前進優先の重みを大幅に高め、バックに非常に重いペナルティを課す（基本は前進）
    ros2 param set /controller_server FollowPath.PreferForwardCritic.cost_weight 25.0
    
    # 速度スムーサーの同期
    if ros2 node list 2>/dev/null | grep "/velocity_smoother" >/dev/null 2>&1; then
        ros2 param set /velocity_smoother max_velocity "[0.40, 0.0, 0.40]"
        ros2 param set /velocity_smoother min_velocity "[-0.20, 0.0, -0.40]"
        ros2 param set /velocity_smoother max_accel "[0.40, 0.0, 1.00]"
        ros2 param set /velocity_smoother max_decel "[-0.40, 0.0, -1.00]"
    fi
    
    # グローバルコストマップの障害物レイヤーを無効化 (回避ルートの算出を禁止し立ち往生/待機させる)
    if ros2 node list 2>/dev/null | grep "/global_costmap" >/dev/null 2>&1; then
        ros2 param set /global_costmap/global_costmap obstacle_layer.enabled False
    fi

elif [ "$MODE" = "strict_slow" ]; then
    echo "パス追従優先・超低速速度モードに設定中..."
    
    # MPPIコントローラーの速度上限と探索分散の設定（極めて低速・高精度・安全なバックを許可）
    ros2 param set /controller_server FollowPath.vx_max 0.20
    ros2 param set /controller_server FollowPath.vx_min -0.10
    ros2 param set /controller_server FollowPath.wz_max 0.20
    ros2 param set /controller_server FollowPath.vx_std 0.05
    ros2 param set /controller_server FollowPath.wz_std 0.05
    
    # MPPIコントローラーの加速度制限の設定（極めて緩やかな加減速）
    ros2 param set /controller_server FollowPath.ax_max 0.20
    ros2 param set /controller_server FollowPath.ax_min -0.20
    ros2 param set /controller_server FollowPath.az_max 0.50
    
    # 障害物回避の重み（しっかり安全に止まる）
    ros2 param set /controller_server FollowPath.CostCritic.cost_weight 20.0
    
    # パス追従の重みを極限まで高め、かつ前進の引っ張り（PathFollow）を抑えて回避行動を抑制
    ros2 param set /controller_server FollowPath.PathAlignCritic.cost_weight 60.0
    ros2 param set /controller_server FollowPath.PathFollowCritic.cost_weight 3.0
    ros2 param set /controller_server FollowPath.PathAngleCritic.cost_weight 1.5
    
    # 旋回抑制の重みを高めて無駄な方向転換を抑制
    ros2 param set /controller_server FollowPath.TwirlingCritic.cost_weight 5.0
    
    # ゴールへの進行・向きの重みを下げて障害物前での立ち往生/待機を優先
    ros2 param set /controller_server FollowPath.GoalCritic.cost_weight 0.5
    ros2 param set /controller_server FollowPath.GoalAngleCritic.cost_weight 0.5
    
    # 前進優先の重みを大幅に高め、バックに非常に重いペナルティを課す（基本は前進）
    ros2 param set /controller_server FollowPath.PreferForwardCritic.cost_weight 25.0
    
    # 速度スムーサーの同期
    if ros2 node list 2>/dev/null | grep "/velocity_smoother" >/dev/null 2>&1; then
        ros2 param set /velocity_smoother max_velocity "[0.20, 0.0, 0.20]"
        ros2 param set /velocity_smoother min_velocity "[-0.10, 0.0, -0.20]"
        ros2 param set /velocity_smoother max_accel "[0.20, 0.0, 0.50]"
        ros2 param set /velocity_smoother max_decel "[-0.20, 0.0, -0.50]"
    fi
    
    # グローバルコストマップの障害物レイヤーを無効化 (回避ルートの算出を禁止し立ち往生/待機させる)
    if ros2 node list 2>/dev/null | grep "/global_costmap" >/dev/null 2>&1; then
        ros2 param set /global_costmap/global_costmap obstacle_layer.enabled False
    fi
fi

echo "-----------------------------------------"
echo "✓ 走行モード切り替え完了"
echo "========================================="
