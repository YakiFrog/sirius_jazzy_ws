#!/bin/bash
# 地図切り替えスクリプト
# 使い方: ./change_map.sh
#         一覧から地図を選択して切り替え

trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2

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

# 地図を切り替え
change_map() {
    local map_path="$1"
    
    echo "地図を切り替え中..."
    echo "  パス: $map_path"
    echo ""
    
    # map_server サービスを呼び出し（出力を簡潔に）
    result=$(ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: '$map_path'}" 2>&1)
    exit_code=$?
    
    # 成功判定: exit_code が 0 かつ result に "result:" が含まれる
    if [ $exit_code -eq 0 ]; then
        echo "✓ 地図の切り替えが完了しました"
    else
        echo "✗ 地図の切り替えに失敗しました"
        echo "  Nav2が起動しているか確認してください"
        echo "  エラー: $result"
    fi
}

# メイン処理（ループ）
cd ~/sirius_jazzy_ws
source install/setup.bash

while : ; do
    echo ""
    read -p "Press [Enter] key to change map..."
    
    selected_map=""
    select_map
    
    if [ $? -eq 0 ] && [ -n "$selected_map" ]; then
        change_map "$selected_map"
    fi
    
    echo ""
    echo "-----------------------------------------"
done
