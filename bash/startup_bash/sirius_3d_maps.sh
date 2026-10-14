#!/bin/bash
# 3D地図の選択ヘルパー。
#   - maps_waypoints/maps3d/<name>/ : save3d が作る 3D(.npy/.pcd/.ply) と 2D投影(.yaml)
#   - 既存の3D点群(.ply/.pcd/.npy) : RTAB-Map等の出力。同名の .yaml を2D地図として使用
#   - 環境変数 SIRIUS_3D_MAP を指定するとメニューを出さずにそれを使用
#   - 環境変数 SIRIUS_3D_MAP_LIST_ALL=1 で backup も含めて全件表示

SIRIUS_3D_MAPS_DIR="${HOME}/sirius_jazzy_ws/maps_waypoints/maps3d"
SIRIUS_3D_MAPS_ROOT="${HOME}/sirius_jazzy_ws/maps_waypoints"

sirius3d_find_in_dir() {
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

sirius3d_list_entries() {
    local d f y
    if [ -d "$SIRIUS_3D_MAPS_DIR" ]; then
        while IFS= read -r d; do
            f=$(sirius3d_find_in_dir "$d" npy pcd ply) || continue
            y=$(sirius3d_find_in_dir "$d" yaml) || y=""
            echo "$f|$y"
        done < <(find "$SIRIUS_3D_MAPS_DIR" -maxdepth 1 -mindepth 1 -type d 2>/dev/null | sort)
    fi
    local candidates
    if [ "${SIRIUS_3D_MAP_LIST_ALL:-0}" = "1" ]; then
        candidates=$(find "$SIRIUS_3D_MAPS_ROOT" \( -name "*.ply" -o -name "*.pcd" -o -name "*.npy" \) 2>/dev/null)
    else
        candidates=$(find "$SIRIUS_3D_MAPS_ROOT" \( -name "*.ply" -o -name "*.pcd" -o -name "*.npy" \) 2>/dev/null | grep -v "/backup/")
    fi
    while IFS= read -r f; do
        [ -z "$f" ] && continue
        case "$f" in "$SIRIUS_3D_MAPS_DIR"/*) continue;; esac
        y="${f%.*}.yaml"
        [ -f "$y" ] || y=""
        echo "$f|$y"
    done < <(echo "$candidates" | sort)
}

sirius3d_pick() {
    if [ -n "${SIRIUS_3D_MAP:-}" ]; then
        local p
        p=$(eval echo "$SIRIUS_3D_MAP")
        if [ -f "$p" ]; then
            local y="${p%.*}.yaml"
            [ -f "$y" ] || y=""
            echo "$p|$y"
            return 0
        fi
        echo "SIRIUS_3D_MAP が見つかりません: $p" >&2
        return 1
    fi

    local entries=()
    mapfile -t entries < <(sirius3d_list_entries)
    if [ ${#entries[@]} -eq 0 ]; then
        {
            echo "3D地図が見つかりません。"
            echo "  - 新規作成: 3D地図生成セットで走行後に save3d を実行"
            echo "  - 既存利用: maps_waypoints 配下の .ply/.pcd/.npy を配置"
        } >&2
        return 1
    fi

    {
        echo "========================================="
        echo "  3D地図を選択 (3D | 2D投影)"
        echo "========================================="
    } >&2
    local i rel y2d
    for i in "${!entries[@]}"; do
        rel="${entries[$i]%%|*}"
        rel="${rel#$HOME/sirius_jazzy_ws/}"
        y2d="${entries[$i]##*|}"
        if [ -n "$y2d" ]; then
            printf "  [%d] %s\n" "$((i + 1))" "$rel" >&2
        else
            printf "  [%d] %s  (2D投影なし)\n" "$((i + 1))" "$rel" >&2
        fi
    done
    echo "=========================================" >&2

    local sel
    read -p "番号を入力してください (1-${#entries[@]}): " sel
    if [[ ! "$sel" =~ ^[0-9]+$ ]] || [ "$sel" -lt 1 ] || [ "$sel" -gt ${#entries[@]} ]; then
        echo "無効な選択です" >&2
        return 1
    fi
    echo "${entries[$((sel - 1))]}"
}
