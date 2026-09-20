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

# 1. 重ね合わせ可能な生成済み地図（必須ファイルが揃ったもの）を新しい順に一覧
#    maps/ 直下だけでなく 0920/theta のような日付サブフォルダにも保存されるため、
#    ディレクトリ名ではなく *.colored.json を起点に地図ベースを探索する。
map_base_has_requirements() {
    local base="$1"
    [ -f "${base}.yaml" ] && [ -f "${base}.pgm" ] && \
        [ -f "${base}.colored.pgm" ] && [ -f "${base}.colored.json" ] && \
        [ -f "${base}.color.png" ]
}

map_label() {
    local rel="${1#"$MAPS_DIR"/}"
    local dir_rel
    dir_rel="$(dirname "$rel")"
    if [ "$dir_rel" = "." ]; then
        printf '%s' "$(basename "$rel")"
    elif [ "$(basename "$dir_rel")" = "$(basename "$rel")" ]; then
        printf '%s' "$dir_rel"
    else
        printf '%s' "$rel"
    fi
}

SOURCE_MAPS=()
while IFS= read -r json_file; do
    base="${json_file%.colored.json}"
    if map_base_has_requirements "$base"; then
        SOURCE_MAPS+=("$base")
    fi
done < <(find "$MAPS_DIR" -type f -name '*.colored.json' -printf '%T@ %p\n' 2>/dev/null | sort -rn | cut -d' ' -f2-)

if [ ${#SOURCE_MAPS[@]} -eq 0 ]; then
    echo ""
    echo "エラー: 重ね合わせ可能な生成済み地図がありません。"
    echo "先にマッピングを実行して地図を保存してください。"
    exit 1
fi

echo ""
echo "重ね合わせる既存地図を選択してください（パスを直接入力しても可）:"
for i in "${!SOURCE_MAPS[@]}"; do
    echo "  [$((i+1))] $(map_label "${SOURCE_MAPS[$i]}")"
done
read -p "選択 [1]: " source_choice
source_choice=${source_choice:-1}

SOURCE_BASE=""
if [[ "$source_choice" =~ ^[0-9]+$ ]]; then
    source_index=$((source_choice-1))
    if [ "$source_index" -lt 0 ] || [ "$source_index" -ge ${#SOURCE_MAPS[@]} ]; then
        echo "無効な選択です。"
        exit 1
    fi
    SOURCE_BASE="${SOURCE_MAPS[$source_index]}"
else
    # 数字以外はパス入力として扱う（例: ~/sirius_jazzy_ws/maps_waypoints/maps/0920/theta）
    input_path="${source_choice/#\~/$HOME}"
    if [ -d "$input_path" ]; then
        json_file="$(find "$input_path" -maxdepth 1 -type f -name '*.colored.json' | sort | head -n1)"
        if [ -z "$json_file" ]; then
            echo "エラー: 指定フォルダに地図ファイル (*.colored.json) がありません: $input_path"
            exit 1
        fi
        SOURCE_BASE="${json_file%.colored.json}"
    elif [ -f "$input_path" ]; then
        SOURCE_BASE="$input_path"
        SOURCE_BASE="${SOURCE_BASE%.colored.json}"
        SOURCE_BASE="${SOURCE_BASE%.yaml}"
        SOURCE_BASE="${SOURCE_BASE%.pgm}"
    else
        echo "エラー: フォルダ/ファイルが見当たりません: $input_path"
        exit 1
    fi
    if ! map_base_has_requirements "$SOURCE_BASE"; then
        echo "エラー: 地図に必要なファイルが揃っていません: $SOURCE_BASE"
        exit 1
    fi
fi
SOURCE_DIR="$(dirname "$SOURCE_BASE")"
echo "選択された地図: $SOURCE_BASE"

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
