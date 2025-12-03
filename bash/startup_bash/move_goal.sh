#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
cd ~/sirius_jazzy_ws

WAYPOINTS_DIR="${HOME}/sirius_jazzy_ws/maps_waypoints/waypoints"

# ウェイポイントファイル一覧を取得
get_waypoints_list() {
    find "$WAYPOINTS_DIR" -name "*.yaml" -type f | sort
}

# ウェイポイントファイルを選択
select_waypoints() {
    echo "========================================="
    echo "  利用可能なウェイポイントファイル一覧"
    echo "========================================="
    
    wp_files=($(get_waypoints_list))
    
    if [ ${#wp_files[@]} -eq 0 ]; then
        echo "エラー: $WAYPOINTS_DIR にウェイポイントファイルが見つかりません"
        return 1
    fi
    
    for i in "${!wp_files[@]}"; do
        wp_name=$(basename "${wp_files[$i]}" .yaml)
        echo "  [$((i+1))] $wp_name"
    done
    
    echo "========================================="
    echo -n "番号を入力 (デフォルト: waypoints): "
    read selection
    
    # 空入力の場合はデフォルト
    if [ -z "$selection" ]; then
        selected_waypoints="waypoints"
        return 0
    fi
    
    if [[ ! "$selection" =~ ^[0-9]+$ ]] || [ "$selection" -lt 1 ] || [ "$selection" -gt ${#wp_files[@]} ]; then
        echo "エラー: 無効な選択です"
        return 1
    fi
    
    selected_waypoints=$(basename "${wp_files[$((selection-1))]}" .yaml)
    echo "選択されたウェイポイント: $selected_waypoints"
    echo ""
}

while : ;do
    read -p "Press [Enter] key to start waypoint_nav..."
    source install/setup.bash
    
    # ウェイポイントファイルを選択
    selected_waypoints=""
    select_waypoints
    
    if [ $? -ne 0 ]; then
        echo "ウェイポイントの選択がキャンセルされました"
        continue
    fi
    
    echo -n "開始するウェイポイント番号 (デフォルト: 1): "
    read waypoint_number
    count="${waypoint_number:-1}"
    
    echo ""
    echo "ウェイポイントファイル: $selected_waypoints.yaml"
    echo "開始インデックス: $count"
    echo ""
    
    ros2 run sirius_navigation move_goal --waypoints "$selected_waypoints" --count "$count"
done