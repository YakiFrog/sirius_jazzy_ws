#!/bin/bash
# 3D地図ディレクトリ選択の共通関数群。
# 3D地図は maps_waypoints/maps3d/<name>/ に <name>.npy(.pcd) と 2D投影 <name>.yaml(.pgm) を置く。

SIRIUS_3D_MAPS_DIR="${HOME}/sirius_jazzy_ws/maps_waypoints/maps3d"

sirius3d_list_dirs() {
    find "$SIRIUS_3D_MAPS_DIR" -maxdepth 1 -mindepth 1 -type d 2>/dev/null | sort
}

sirius3d_pick_dir() {
    local dirs
    mapfile -t dirs < <(sirius3d_list_dirs)
    if [ ${#dirs[@]} -eq 0 ]; then
        echo "3D地図がありません: $SIRIUS_3D_MAPS_DIR" >&2
        echo "先に 3D地図保存(save3d) を実行してください。" >&2
        return 1
    fi
    {
        echo "========================================="
        echo "  利用可能な3D地図"
        echo "========================================="
    } >&2
    local i
    for i in "${!dirs[@]}"; do
        echo "  [$((i+1))] $(basename "${dirs[$i]}")" >&2
    done
    echo "=========================================" >&2
    local sel
    read -p "番号を入力してください (1-${#dirs[@]}): " sel
    if [[ ! "$sel" =~ ^[0-9]+$ ]] || [ "$sel" -lt 1 ] || [ "$sel" -gt ${#dirs[@]} ]; then
        echo "無効な選択です" >&2
        return 1
    fi
    echo "${dirs[$((sel-1))]}"
}

sirius3d_find_file() {
    local dir="$1"
    shift
    local base
    base=$(basename "$dir")
    local ext
    for ext in "$@"; do
        if [ -f "$dir/$base.$ext" ]; then
            echo "$dir/$base.$ext"
            return 0
        fi
    done
    for ext in "$@"; do
        local f
        f=$(find "$dir" -maxdepth 1 -name "*.$ext" 2>/dev/null | sort | head -1)
        if [ -n "$f" ]; then
            echo "$f"
            return 0
        fi
    done
    return 1
}
