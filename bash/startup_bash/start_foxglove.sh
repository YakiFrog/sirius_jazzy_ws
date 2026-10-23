#!/bin/bash
# Foxglove Bridge を起動し、接続方法（IP・URL）をコンソールに表示する。
# Usage: start_foxglove.sh [port]   # 既定 8766（8765はDartsRobot monitor_dashboardが使用中）
set -eo pipefail
PORT="${1:-${FOXGLOVE_PORT:-8766}}"
WS="$HOME/sirius_jazzy_ws"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

IP_LIST="$(hostname -I 2>/dev/null || true)"
TS_IP="$(tailscale ip -4 2>/dev/null | head -1 || true)"

echo "================================================="
echo " Foxglove Bridge 起動 (port ${PORT})"
echo "-------------------------------------------------"
echo " 接続先URL（Foxglove WebSocket）:"
for ip in $IP_LIST; do
    case "$ip" in
        *:*) echo "   ws://[${ip}]:${PORT}" ;;
        *) echo "   ws://${ip}:${PORT}" ;;
    esac
done
if [ -n "$TS_IP" ]; then
    echo "   ws://${TS_IP}:${PORT}   (Tailscale)"
fi
echo "-------------------------------------------------"
echo " Foxglove Studio: Open connection -> Foxglove WebSocket -> 上記URLを入力"
echo " Web版でも可: https://studio.foxglove.dev"
echo " 3D地図表示: 3Dパネルを追加 -> Frame: offline_map / Topic: /offline_map/colored_points"
echo " 削除対象の比較: /offline_map/removed_points をRGBで重ねる（赤色）"
echo "   （先に view_3dmap で地図を配信しておく）"
echo "================================================="

# The preview relay itself keeps one newest scan. Keep normal bridge history so
# /tf and transient-local /tf_static are not globally limited to one message.
exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:="$PORT" max_qos_depth:=10 send_buffer_limit:=1000000
