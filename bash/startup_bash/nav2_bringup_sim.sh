#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
cd ~/sirius_jazzy_ws

MAPS_DIR="${HOME}/sirius_jazzy_ws/maps_waypoints/maps"

# 地図ファイル一覧を取得
get_map_list() {
    find "$MAPS_DIR" -name "*.yaml" -type f | sort
}

# 地図を選択
select_map() {
    echo "========================================="
    echo "  利用可能な地図一覧"
    echo "========================================="
    
    maps=($(get_map_list))
    
    if [ ${#maps[@]} -eq 0 ]; then
        echo "エラー: $MAPS_DIR に地図ファイルが見つかりません"
        return 1
    fi
    
    for i in "${!maps[@]}"; do
        map_name=$(basename "${maps[$i]}" .yaml)
        echo "  [$((i+1))] $map_name"
    done
    
    echo "========================================="
    echo -n "番号を入力してください (1-${#maps[@]}): "
    read selection
    
    if [[ ! "$selection" =~ ^[0-9]+$ ]] || [ "$selection" -lt 1 ] || [ "$selection" -gt ${#maps[@]} ]; then
        echo "エラー: 無効な選択です"
        return 1
    fi
    
    selected_map="${maps[$((selection-1))]}"
    echo "選択された地図: $(basename "$selected_map" .yaml)"
    echo ""
}

while : ;do
    read -p "Press [Enter] key to start nav2 bringup..."
    source install/setup.bash
    
    selected_map=""
    select_map
    
    if [ $? -ne 0 ] || [ -z "$selected_map" ]; then
        echo "地図の選択がキャンセルされました"
        continue
    fi
    
    ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=True \
    map:=$selected_map \
    params_file:=${HOME}/sirius_jazzy_ws/params/nav2_params_sim.yaml \
    use_composition:=False
done