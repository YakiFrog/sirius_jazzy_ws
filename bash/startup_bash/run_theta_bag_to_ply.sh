#!/bin/bash
# ==============================================================================
# 録画済みbagから色付き3D地図(PLY)を生成する対話ラッパー（SiriusLauncherから実行可）。
#   引数にbagパスを渡せば対話スキップ: run_theta_bag_to_ply.sh <bag_dir>
# ==============================================================================
set -u
WS_DIR="${HOME}/sirius_jazzy_ws"
ROSBAG_DIR="${HOME}/rosbag2_data"
CONVERTER="${WS_DIR}/other_programs/theta_3d_map/theta_bag_to_ply.py"

set +u
if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
else
    source /opt/ros/jazzy/setup.bash
fi
set -u

BAG="${1:-}"

list_bags() {
    find "$ROSBAG_DIR" -maxdepth 2 -name metadata.yaml -printf '%h\n' 2>/dev/null | sort -r
    # metadata.yaml が無い(未reindex)bagも候補に
    find "$ROSBAG_DIR" -maxdepth 2 -name '*.mcap' -printf '%h\n' 2>/dev/null | sort -u -r
}

if [ -z "$BAG" ]; then
    mapfile -t BAGS < <(list_bags | awk '!seen[$0]++')
    if [ ${#BAGS[@]} -eq 0 ]; then
        echo "エラー: ${ROSBAG_DIR} にbagが見つかりません。"
        read -r -p "Enterで閉じます..." _
        exit 1
    fi
    echo "================================================="
    echo " 色付き3D地図を生成するbagを選択"
    echo "================================================="
    for i in "${!BAGS[@]}"; do
        printf "  [%d] %s\n" "$((i + 1))" "$(basename "${BAGS[$i]}")"
    done
    read -r -p "番号 (既定1): " SEL
    SEL=${SEL:-1}
    if ! [[ "$SEL" =~ ^[0-9]+$ ]] || [ "$SEL" -lt 1 ] || [ "$SEL" -gt "${#BAGS[@]}" ]; then
        echo "エラー: 不正な選択です。"
        read -r -p "Enterで閉じます..." _
        exit 1
    fi
    BAG="${BAGS[$((SEL - 1))]}"
fi

if [ ! -d "$BAG" ]; then
    echo "エラー: bagディレクトリがありません: $BAG"
    read -r -p "Enterで閉じます..." _
    exit 1
fi

read -r -p "ボクセルサイズ[m] (既定0.05): " VOXEL
VOXEL=${VOXEL:-0.05}
read -r -p "固定フレーム (既定 sirius3/odom, SLAM使用時は map): " FRAME
FRAME=${FRAME:-sirius3/odom}
OUT="${BAG}/colored_map.ply"

echo ""
echo "生成中: bag=$(basename "$BAG") voxel=$VOXEL frame=$FRAME"
python3 "$CONVERTER" "$BAG" -o "$OUT" --voxel "$VOXEL" --frame "$FRAME"
STATUS=$?

echo ""
if [ $STATUS -eq 0 ] && [ -f "$OUT" ]; then
    echo "✓ 完成: $OUT ($(du -h "$OUT" | cut -f1))"
    echo "  表示例(open3dがある端末): python3 -c \"import open3d as o3d; o3d.visualization.draw_geometries([o3d.io.read_point_cloud('$OUT')])\""
else
    echo "✗ 生成に失敗しました (exit=$STATUS)"
fi
read -r -p "Enterで閉じます..." _
