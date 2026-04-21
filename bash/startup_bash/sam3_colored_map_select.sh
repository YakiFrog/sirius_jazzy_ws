#!/bin/bash

# ROS環境の読み込み
if [ -f "$HOME/sirius_jazzy_ws/install/setup.bash" ]; then
    source "$HOME/sirius_jazzy_ws/install/setup.bash"
else
    source /opt/ros/jazzy/setup.bash
fi
export ROS_DOMAIN_ID=56

MAP_DIR="$HOME/sirius_jazzy_ws/maps_waypoints/maps"

echo "------------------------------------------------"
echo "  SAM3 Color Map Selector"
echo "------------------------------------------------"

# マップファイルのリストを取得
cd "$MAP_DIR" || exit
maps=($(ls *.pgm 2>/dev/null | sed 's/\.pgm$//'))

if [ ${#maps[@]} -eq 0 ]; then
    echo "エラー: $MAP_DIR にマップが見つかりません。"
    exit 1
fi

echo "ロードするマップを選択してください:"
for i in "${!maps[@]}"; do
    echo "  [$i] ${maps[$i]}"
done

read -p "選択 (番号を入力): " choice

if [[ ! "$choice" =~ ^[0-9]+$ ]] || [ "$choice" -ge "${#maps[@]}" ]; then
    echo "エラー: 無効な選択です。"
    exit 1
fi

SELECTED_MAP="${maps[$choice]}"
SELECTED_PATH="$MAP_DIR/$SELECTED_MAP"

echo "------------------------------------------------"
echo "次のマップをロード中: $SELECTED_MAP"
echo "トピック: /sam3/static_colored_map_cloud"
echo "------------------------------------------------"

ros2 run sirius_navigation sam3_colored_map_loader --ros-args -p map_path:="$SELECTED_PATH"
