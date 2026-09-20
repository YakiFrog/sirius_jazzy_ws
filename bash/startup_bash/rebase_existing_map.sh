#!/bin/bash
# ==============================================================================
# 既存の生成済み路面/セマンティック地図を SLAM Toolbox の構造地図に重ね合わせる
# Rosbag再生やマッピングは行わず、保存済み地図のみを対象にする。
# ==============================================================================

WS_DIR="${HOME}/sirius_jazzy_ws"
MAPS_DIR="$WS_DIR/maps_waypoints/maps"
REBASE_SCRIPT="$WS_DIR/bash/startup_bash/rebase_semantic_map_to_slam.py"

echo "================================================="
echo "  既存地図 × SLAM Toolbox 重ね合わせ"
echo "================================================="

if [ ! -f "$REBASE_SCRIPT" ]; then
    echo "エラー: 変換スクリプトがありません: $REBASE_SCRIPT"
    exit 1
fi

# 1. 重ね合わせ可能な生成済み地図（必須ファイルが揃ったディレクトリ）を新しい順に一覧
SOURCE_MAPS=()
while IFS= read -r dir; do
    base="$dir/$(basename "$dir")"
    if [ -f "${base}.yaml" ] && [ -f "${base}.pgm" ] && \
        [ -f "${base}.colored.pgm" ] && [ -f "${base}.colored.json" ] && \
        [ -f "${base}.color.png" ]; then
        SOURCE_MAPS+=("$dir")
    fi
done < <(find "$MAPS_DIR" -maxdepth 1 -mindepth 1 -type d -printf '%T@ %p\n' 2>/dev/null | sort -rn | cut -d' ' -f2-)

if [ ${#SOURCE_MAPS[@]} -eq 0 ]; then
    echo ""
    echo "エラー: 重ね合わせ可能な生成済み地図がありません。"
    echo "先にマッピングを実行して地図を保存してください。"
    exit 1
fi

echo ""
echo "重ね合わせる既存地図を選択してください:"
for i in "${!SOURCE_MAPS[@]}"; do
    echo "  [$((i+1))] $(basename "${SOURCE_MAPS[$i]}")"
done
read -p "選択 [1]: " source_choice
source_choice=${source_choice:-1}
source_index=$((source_choice-1))
if [ "$source_index" -lt 0 ] || [ "$source_index" -ge ${#SOURCE_MAPS[@]} ]; then
    echo "無効な選択です。"
    exit 1
fi
SOURCE_DIR="${SOURCE_MAPS[$source_index]}"
SOURCE_BASE="$SOURCE_DIR/$(basename "$SOURCE_DIR")"
echo "選択された地図: $SOURCE_DIR"

# 2. 構造ベースにする SLAM Toolbox 地図YAMLを選択
mapfile -t SLAM_MAP_YAMLS < <(
    find "$MAPS_DIR" -maxdepth 1 -type f -name '*.yaml' | sort
)
if [ ${#SLAM_MAP_YAMLS[@]} -eq 0 ]; then
    echo ""
    echo "エラー: SLAM Toolbox地図YAMLがありません: $MAPS_DIR"
    exit 1
fi

echo ""
echo "構造ベースにするSLAM Toolbox地図を選択してください:"
for i in "${!SLAM_MAP_YAMLS[@]}"; do
    echo "  [$((i+1))] $(basename "${SLAM_MAP_YAMLS[$i]}")"
done
read -p "選択 [1]: " slam_choice
slam_choice=${slam_choice:-1}
slam_index=$((slam_choice-1))
if [ "$slam_index" -lt 0 ] || [ "$slam_index" -ge ${#SLAM_MAP_YAMLS[@]} ]; then
    echo "無効な選択です。"
    exit 1
fi
SLAM_BASE_YAML="${SLAM_MAP_YAMLS[$slam_index]}"
echo "SLAM Toolbox構造地図: $SLAM_BASE_YAML"

echo ""
echo "重ね合わせを実行中..."
if python3 "$REBASE_SCRIPT" "$SOURCE_BASE" "$SLAM_BASE_YAML" ${REBASE_ARGS:-}; then
    echo ""
    echo "✓ 重ね合わせ地図を生成しました。"
else
    echo ""
    echo "エラー: SLAM Toolboxベース版の生成に失敗しました。"
    exit 1
fi
