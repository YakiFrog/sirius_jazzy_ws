#!/bin/bash
# ==============================================================================
# THETA SAM3 事前確認（rosbag代表フレーム -> HTMLレポート）
# 選択したrosbagから等間隔にフレームを抽出し、本番と同じ経路でSAM3推論して
# セマンティック対象・尤度統計・閾値スイープをHTMLでブラウザ表示します。
# 使い方: run_theta_sam3_preview.sh [rosbagディレクトリ] [追加オプション]
# ==============================================================================

WS_DIR="${HOME}/sirius_jazzy_ws"
ROSBAG_DIR="${HOME}/rosbag2_data"
SERVER_URL="http://localhost:8080"
PREVIEW_PY="$WS_DIR/src/sirius/sirius_navigation/sirius_navigation/theta_sam3_preview.py"

source "$WS_DIR/install/setup.bash" 2>/dev/null || source /opt/ros/jazzy/setup.bash

echo "================================================="
echo "  THETA SAM3 事前確認"
echo "================================================="

# 1. rosbagの選択（引数でディレクトリ指定も可）
SELECTED_BAG=""
if [ $# -gt 0 ] && [ -d "$1" ]; then
    SELECTED_BAG="$(readlink -f "$1")"
    shift
fi

if [ -n "$SELECTED_BAG" ] && [ ! -f "$SELECTED_BAG/metadata.yaml" ]; then
    if compgen -G "$SELECTED_BAG/*/*.mcap" >/dev/null; then
        INNER=()
        while IFS= read -r d; do
            compgen -G "$d/*.mcap" >/dev/null && INNER+=("$d")
        done < <(find "$SELECTED_BAG" -maxdepth 1 -mindepth 1 -type d | sort)
        if [ ${#INNER[@]} -gt 0 ]; then
            echo "$(basename "$SELECTED_BAG") 内の Rosbag 一覧:"
            for i in "${!INNER[@]}"; do
                echo "  [$((i+1))] $(basename "${INNER[$i]}")"
            done
            read -p "使用する Rosbag 番号 [1]: " sel
            sel=${sel:-1}
            SELECTED_BAG="${INNER[$((sel-1))]}"
        fi
    fi
fi

if [ -z "$SELECTED_BAG" ]; then
    BAG_LIST=($(find "$ROSBAG_DIR" -maxdepth 1 -mindepth 1 -type d | sort -r))
    if [ ${#BAG_LIST[@]} -eq 0 ]; then
        echo "エラー: $ROSBAG_DIR に Rosbag がありません。"
        exit 1
    fi
    echo "利用可能な Rosbag 一覧:"
    for i in "${!BAG_LIST[@]}"; do
        echo "  [$((i+1))] $(basename "${BAG_LIST[$i]}")"
    done
    read -p "使用する Rosbag 番号 [1]: " choice
    choice=${choice:-1}
    SELECTED_BAG="${BAG_LIST[$((choice-1))]}"
fi

if [ ! -f "$SELECTED_BAG/metadata.yaml" ]; then
    echo "エラー: rosbagではありません（metadata.yaml無し）: $SELECTED_BAG"
    exit 1
fi
BAG_NAME=$(basename "$SELECTED_BAG")
echo "選択された Rosbag: $SELECTED_BAG"

# 2. 校正YAMLの自動選択（実機 real_* は実機校正）
CALIB="$WS_DIR/src/sirius/sirius_navigation/config/theta_calibration.yaml"
case "$BAG_NAME" in
    real_*) CALIB="$WS_DIR/src/sirius/sirius_navigation/config/theta_calibration_real.yaml" ;;
esac

# 3. SAM3サーバの起動確認
if ! docker ps --format '{{.Names}}' 2>/dev/null | grep -qx sam3_zed_container; then
    if [ -d "${HOME}/sam3_zed_server" ]; then
        echo "SAM3サーバーを起動しています..."
        (cd "${HOME}/sam3_zed_server" && docker compose up -d sam3-zed-merged)
    fi
fi
echo -n "SAM3サーバーの応答を待機中"
for _attempt in $(seq 1 120); do
    if curl -fsS -m 1 "$SERVER_URL/debug_state" >/dev/null 2>&1; then
        echo " OK"
        break
    fi
    echo -n "."
    sleep 1
done

# 4. 事前確認（HTMLレポート）
exec python3 "$PREVIEW_PY" \
    --bag "$SELECTED_BAG" \
    --calibration "$CALIB" \
    --server "$SERVER_URL" "$@"
