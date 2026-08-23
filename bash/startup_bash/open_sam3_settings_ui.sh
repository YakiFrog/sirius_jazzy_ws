#!/usr/bin/env bash
# Start the canonical SAM3 server if necessary and open its settings UI.

set -eu

SAM3_SERVER_DIR="${HOME}/sam3_zed_server"
SAM3_URL="http://localhost:8080/"

if [ ! -d "$SAM3_SERVER_DIR" ]; then
    echo "エラー: SAM3サーバーディレクトリがありません: $SAM3_SERVER_DIR"
    exit 1
fi

if ! docker ps --format '{{.Names}}' | grep -qx sam3_zed_container; then
    echo "SAM3サーバーを起動しています..."
    (cd "$SAM3_SERVER_DIR" && docker compose up -d sam3-zed-merged)
fi

echo -n "SAM3設定UIの準備を待機中"
READY=false
for _attempt in $(seq 1 90); do
    if curl -fsS -m 1 "$SAM3_URL" >/dev/null 2>&1; then
        READY=true
        break
    fi
    echo -n "."
    sleep 1
done
echo ""

if [ "$READY" != true ]; then
    echo "エラー: SAM3サーバーが90秒以内に応答しませんでした。"
    echo "確認: docker logs --tail 100 sam3_zed_container"
    exit 1
fi

echo "SAM3設定UIを開きます: $SAM3_URL"
xdg-open "$SAM3_URL" >/dev/null 2>&1 &
