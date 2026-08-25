#!/bin/bash

# ROS環境の読み込み
if [ -f "$HOME/sirius_jazzy_ws/install/setup.bash" ]; then
    source "$HOME/sirius_jazzy_ws/install/setup.bash"
else
    source /opt/ros/jazzy/setup.bash
fi

MAP_DIR="$HOME/sirius_jazzy_ws/maps_waypoints/maps"

echo "------------------------------------------------"
echo "  SAM3 Color Map Selector"
echo "------------------------------------------------"

# マップファイルのリストを取得
cd "$MAP_DIR" || exit
maps=($(find . -maxdepth 2 -name "*.pgm" ! -name "*.colored.pgm" 2>/dev/null | sed 's|^\./||' | sed 's/\.pgm$//' | sort))

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

echo ""
echo "RVizに表示する内容を選択してください:"
echo "  [1] 路面RGBとセマンティックを別トピックで両方 [推奨]"
echo "  [2] 路面RGBテクスチャのみ (.texture.png)"
echo "  [3] SAM3セマンティッククラス色のみ (.colored.pgm)"
read -p "表示 [1]: " display_choice

case "${display_choice:-1}" in
    1)
        DISPLAY_MODE="both"
        Z_OFFSET="0.03"
        CLOUD_TOPIC="/sam3/static_texture_map_cloud と /sam3/static_colored_map_cloud"
        ;;
    2)
        DISPLAY_MODE="texture"
        Z_OFFSET="0.03"
        CLOUD_TOPIC="/sam3/static_texture_map_cloud"
        ;;
    3)
        DISPLAY_MODE="semantic"
        Z_OFFSET="-0.20"
        CLOUD_TOPIC="/sam3/static_colored_map_cloud"
        ;;
    *)
        echo "エラー: 1、2、3 のいずれかを選択してください。"
        exit 1
        ;;
esac

echo ""
echo "配信方法を選択してください:"
echo "  [1] 初回のみ配信 [推奨・Rosbag容量を節約]"
echo "  [2] 定期配信（周波数を指定）"
read -p "配信 [1]: " publish_choice

case "${publish_choice:-1}" in
    1)
        PUBLISH_ONCE="true"
        PUBLISH_RATE="1.0"
        PUBLISH_DESCRIPTION="初回のみ"
        ;;
    2)
        PUBLISH_ONCE="false"
        read -p "配信周波数 Hz [0.5]: " publish_rate_input
        PUBLISH_RATE="${publish_rate_input:-0.5}"
        if ! awk -v rate="$PUBLISH_RATE" \
            'BEGIN { exit !(rate ~ /^[0-9]+([.][0-9]+)?$/ && rate > 0 && rate <= 2.0) }'; then
            echo "エラー: 配信周波数は 0より大きく2.0以下で指定してください。"
            exit 1
        fi
        PUBLISH_DESCRIPTION="定期 ${PUBLISH_RATE} Hz"
        ;;
    *)
        echo "エラー: 1 または 2 を選択してください。"
        exit 1
        ;;
esac

echo "------------------------------------------------"
echo "次のマップをロード中: $SELECTED_MAP"
echo "表示モード: $DISPLAY_MODE"
echo "配信方法: $PUBLISH_DESCRIPTION"
echo "トピック: $CLOUD_TOPIC"
echo "------------------------------------------------"

ros2 run sirius_navigation sam3_colored_map_loader --ros-args \
    -p map_path:="$SELECTED_PATH" \
    -p visualization_mode:="$DISPLAY_MODE" \
    -p z_offset:="$Z_OFFSET" \
    -p publish_once:="$PUBLISH_ONCE" \
    -p publish_rate:="$PUBLISH_RATE"
