#!/bin/bash
# USBオートサスペンド無効化のudevルールをインストールする。
# Hokuyo/Roboteq/WiFi/ハブがUSBバスから落ちる問題（error -71 / -110）への対策。
set -e
RULES_SRC="$(cd "$(dirname "$0")/../.." && pwd)/udevrules/99-usb-autosuspend.rules"
RULES_DST="/etc/udev/rules.d/99-usb-autosuspend.rules"

if [ ! -f "$RULES_SRC" ]; then
    echo "エラー: $RULES_SRC がありません"
    exit 1
fi
sudo cp "$RULES_SRC" "$RULES_DST"
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "✓ インストール完了: $RULES_DST"
echo "既に接続中のデバイスへ即時反映:"
for dev in /sys/bus/usb/devices/*/power/control; do
    if [ -w "$dev" ]; then
        echo on | sudo tee "$dev" >/dev/null 2>&1 || true
    fi
done
echo "確認: cat /sys/bus/usb/devices/3-7/power/control"
