#!/bin/bash
# ==============================================================================
# NUC(ssh先)のリポジトリ版を監視し、origin(master)に揃える。
#   - 本体(sirius_jazzy_ws) と サブモジュール(sirius_navigation) の HEAD を比較
#   - sync で fetch + reset --hard origin/master、必要なら sirius_navigation を再ビルド
#
# 使い方:
#   sync_nuc_repo.sh [check|sync|watch] [host] [interval_sec]
#     既定: check, sirius-jazzy-nuc, interval 30
#   watch は interval ごとに確認し、ズレていれば自動 sync する。
# ==============================================================================
set -u
MODE="${1:-check}"
HOST="${2:-sirius-jazzy-nuc}"
INTERVAL="${3:-30}"
WS="$HOME/sirius_jazzy_ws"
SUBREL="src/sirius/sirius_navigation"
SSHOPT="-o BatchMode=yes -o ConnectTimeout=8"

cd "$WS" || { echo "ワークスペースがありません: $WS"; exit 1; }
git fetch origin -q 2>/dev/null || true
ORIGIN_MAIN=$(git rev-parse --short origin/master 2>/dev/null || echo '?')
LOCAL_MAIN=$(git rev-parse --short HEAD)
LOCAL_SUB=$(git -C "$SUBREL" rev-parse --short HEAD 2>/dev/null || echo '?')

remote_heads() {
    ssh $SSHOPT "$HOST" "cd ~/sirius_jazzy_ws 2>/dev/null && git fetch origin -q 2>/dev/null; \
        echo \$(git rev-parse --short HEAD 2>/dev/null) \$(git -C $SUBREL rev-parse --short HEAD 2>/dev/null)" 2>/dev/null
}

sync_remote() {
    echo "[sync] $HOST を origin/master に同期..."
    ssh $SSHOPT "$HOST" "cd ~/sirius_jazzy_ws && \
        git fetch origin -q && git reset --hard origin/master -q && \
        git -C $SUBREL fetch origin -q && git -C $SUBREL reset --hard origin/master -q && \
        source /opt/ros/jazzy/setup.bash && source install/setup.bash 2>/dev/null; \
        colcon build --packages-select sirius_navigation --symlink-install 2>&1 | tail -2"
}

check_once() {
    local rmain rsub
    read -r rmain rsub < <(remote_heads)
    rmain="${rmain:-?}"; rsub="${rsub:-?}"
    echo "desktop: main=$LOCAL_MAIN (origin=$ORIGIN_MAIN) sub=$LOCAL_SUB"
    echo "nuc    : main=$rmain sub=$rsub"
    if [ "$LOCAL_MAIN" != "$ORIGIN_MAIN" ]; then
        echo "⚠ desktop に未pushのコミットがあります（先に push が必要）"
    fi
    if [ "$rmain" = "$ORIGIN_MAIN" ] && [ "$rsub" = "$LOCAL_SUB" ]; then
        echo "✓ 同期済み"
        return 0
    fi
    echo "✗ 未同期（nuc が origin と不一致）"
    return 1
}

case "$MODE" in
    check)
        check_once
        ;;
    sync)
        sync_remote
        check_once
        ;;
    watch)
        echo "監視開始: interval=${INTERVAL}s (Ctrl+Cで停止)"
        while true; do
            if ! check_once; then
                sync_remote
            fi
            sleep "$INTERVAL"
        done
        ;;
    *)
        echo "使い方: $0 [check|sync|watch] [host] [interval_sec]"
        exit 2
        ;;
esac
