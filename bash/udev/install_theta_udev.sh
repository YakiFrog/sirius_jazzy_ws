#!/bin/bash
# THETA S HDMIキャプチャのudevルールをインストールする。
# /dev/theta_capture への安定シンボリックリンクを作成する。
set -e
RULES_SRC="$(cd "$(dirname "$0")" && pwd)/99-theta-capture.rules"
RULES_DST="/etc/udev/rules.d/99-theta-capture.rules"

if [ ! -f "$RULES_SRC" ]; then
    echo "エラー: $RULES_SRC がありません"
    exit 1
fi
sudo cp "$RULES_SRC" "$RULES_DST"
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "✓ インストール完了: $RULES_DST"
echo "確認: ls -l /dev/theta_capture"
ls -l /dev/theta_capture 2>/dev/null || echo "（デバイスを再接続してください）"
