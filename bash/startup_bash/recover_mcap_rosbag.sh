#!/usr/bin/env bash
# Recover an unreadable ROS 2 MCAP bag into a new sibling directory.
# The source bag is never modified.

set -uo pipefail

WS_DIR="${SIRIUS_WS_DIR:-${HOME}/sirius_jazzy_ws}"
MCAP_TOOL="${WS_DIR}/mcap"
SOURCE_BAG="${1:-}"
RECOVERY_TMP=""

log() {
    printf '%s\n' "$*" >&2
}

cleanup_partial() {
    if [ -n "$RECOVERY_TMP" ] && [ -d "$RECOVERY_TMP" ]; then
        case "$RECOVERY_TMP" in
            "${SOURCE_PARENT}"/.*_recovering_*) rm -rf -- "$RECOVERY_TMP" ;;
        esac
    fi
}

if [ -z "$SOURCE_BAG" ] || [ ! -d "$SOURCE_BAG" ]; then
    log "エラー: 復旧対象のRosbagディレクトリがありません: ${SOURCE_BAG:-未指定}"
    exit 2
fi

SOURCE_BAG=$(realpath "$SOURCE_BAG")
SOURCE_PARENT=$(dirname "$SOURCE_BAG")
SOURCE_NAME=$(basename "$SOURCE_BAG")

if [ ! -x "$MCAP_TOOL" ]; then
    log "エラー: MCAP復旧ツールがありません: $MCAP_TOOL"
    exit 2
fi

mapfile -d '' MCAP_FILES < <(
    find "$SOURCE_BAG" -maxdepth 1 -type f -name '*.mcap' -print0 | sort -z
)
if [ "${#MCAP_FILES[@]}" -eq 0 ]; then
    log "エラー: $SOURCE_BAG にMCAPファイルがありません。"
    exit 2
fi

# Recovery writes another copy of the recorded payload. Refuse to start when
# there is clearly not enough room, so the healthy source is never endangered.
SOURCE_BYTES=$(du -sb "$SOURCE_BAG" | awk '{print $1}')
AVAILABLE_BYTES=$(df -PB1 "$SOURCE_PARENT" | awk 'NR == 2 {print $4}')
SAFETY_BYTES=$((256 * 1024 * 1024))
REQUIRED_BYTES=$((SOURCE_BYTES + SAFETY_BYTES))
if [ "$AVAILABLE_BYTES" -lt "$REQUIRED_BYTES" ]; then
    log "エラー: MCAP復旧用の空き容量が不足しています。"
    log "  必要: 約 $((REQUIRED_BYTES / 1024 / 1024)) MiB"
    log "  空き: 約 $((AVAILABLE_BYTES / 1024 / 1024)) MiB"
    exit 2
fi

RECOVERY_TMP=$(mktemp -d "${SOURCE_PARENT}/.${SOURCE_NAME}_recovering_XXXXXX") || exit 2
trap 'cleanup_partial; exit 130' INT TERM HUP

log "MCAP破損を検出したため、安全な別ディレクトリへ自動復旧します。"
log "  元データ: $SOURCE_BAG"

for input_file in "${MCAP_FILES[@]}"; do
    output_file="${RECOVERY_TMP}/$(basename "$input_file")"
    log "  復旧中: $(basename "$input_file")"
    if ! "$MCAP_TOOL" recover "$input_file" -o "$output_file" >&2; then
        # A recording cut in the middle of a compressed chunk makes the MCAP
        # CLI return non-zero after it has already written every complete chunk
        # preceding the damage. Accept that partial output only when MCAP's own
        # structural checker confirms that the recovered file is valid.
        if [ -s "$output_file" ] && \
            "$MCAP_TOOL" doctor "$output_file" >/dev/null 2>&1; then
            log "  ⚠ 末尾の不完全チャンクを除外し、救出できた範囲を使用します。"
        else
            log "エラー: MCAPデータの復旧に失敗しました。元データは変更していません。"
            cleanup_partial
            exit 2
        fi
    fi
    if ! "$MCAP_TOOL" doctor "$output_file" >/dev/null 2>&1; then
        log "エラー: 復旧出力のMCAP構造が不正です。元データは変更していません。"
        cleanup_partial
        exit 2
    fi
done

if ! ros2 bag reindex "$RECOVERY_TMP" -s mcap >&2; then
    log "エラー: 復旧後Rosbagのmetadata生成に失敗しました。元データは変更していません。"
    cleanup_partial
    exit 2
fi

RECOVERED_BAG="${SOURCE_BAG}_recovered"
suffix=2
while [ -e "$RECOVERED_BAG" ]; do
    RECOVERED_BAG="${SOURCE_BAG}_recovered_${suffix}"
    suffix=$((suffix + 1))
done

if ! mv "$RECOVERY_TMP" "$RECOVERED_BAG"; then
    log "エラー: 復旧済みRosbagの配置に失敗しました。"
    cleanup_partial
    exit 2
fi
RECOVERY_TMP=""
trap - INT TERM HUP

log "✓ MCAPを復旧しました: $RECOVERED_BAG"
printf '%s\n' "$RECOVERED_BAG"
