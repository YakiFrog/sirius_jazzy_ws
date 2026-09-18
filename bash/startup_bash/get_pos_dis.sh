#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
cd ~/sirius_jazzy_ws
DEFAULT_INTERVAL=4.0
while : ;do
    read -p "Press [Enter] key to start get_position_distance..."
    source install/setup.bash

    # ウェイポイントを置く間隔[m]を対話で指定。無入力/不正ならデフォルトを使用。
    read -p "何m間隔でウェイポイントを置きますか？ [${DEFAULT_INTERVAL}]: " interval
    if ! [[ "$interval" =~ ^[0-9]+([.][0-9]+)?$ ]] || ! awk -v v="$interval" 'BEGIN{exit !(v>0)}'; then
        interval="$DEFAULT_INTERVAL"
        echo "入力なし/不正のため、デフォルト (${DEFAULT_INTERVAL} m) を使用します。"
    else
        # 整数入力でも double パラメータとして渡せるよう小数表記に正規化
        interval=$(awk -v v="$interval" 'BEGIN{printf "%.3f", v}')
        echo "間隔: ${interval} m で記録します。"
    fi

    ros2 run sirius_navigation get_position_distance --ros-args -p distance_threshold:="${interval}"
done
