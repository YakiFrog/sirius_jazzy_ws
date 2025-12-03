#!/bin/bash
# 地図切り替えスクリプト（プログラム呼び出し用）
# 使い方: ./change_map_simple.sh <地図名>
#         地図名は .yaml 拡張子なしで指定
# 
# 例: ./change_map_simple.sh 1202-15f
#     ./change_map_simple.sh atc_11_29_1F
#
# 終了コード:
#   0 - 成功
#   1 - 引数エラー
#   2 - 地図ファイルが見つからない
#   3 - サービス呼び出し失敗

MAPS_DIR="${HOME}/sirius_jazzy_ws/maps_waypoints/maps"

# 引数チェック
if [ -z "$1" ]; then
    echo "エラー: 地図名を指定してください"
    echo "使い方: $0 <地図名>"
    echo ""
    echo "利用可能な地図:"
    find "$MAPS_DIR" -name "*.yaml" -type f | sort | while read f; do
        echo "  - $(basename "$f" .yaml)"
    done
    exit 1
fi

map_name="$1"
map_path="$MAPS_DIR/$map_name.yaml"

# 地図ファイルの存在確認
if [ ! -f "$map_path" ]; then
    echo "エラー: 地図ファイルが見つかりません: $map_path"
    exit 2
fi

# 地図を切り替え
result=$(ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: '$map_path'}" 2>&1)
exit_code=$?

if [ $exit_code -eq 0 ]; then
    echo "OK: $map_name"
    exit 0
else
    echo "FAILED: $map_name"
    exit 3
fi
