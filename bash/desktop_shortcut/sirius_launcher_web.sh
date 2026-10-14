#!/bin/bash
# ブラウザ版SiriusLauncherを起動（未起動なら）してブラウザで開く。
set -u
WS="$HOME/sirius_jazzy_ws"
cd "$WS/other_programs/sirius_launcher" || exit 1
PORT="${SIRIUS_WEB_PORT:-8090}"
DOMAIN="${ROS_DOMAIN_ID:-53}"
if ! ss -ltn 2>/dev/null | grep -q ":$PORT "; then
    nohup python3 web_server.py --port "$PORT" --ros-domain "$DOMAIN" >/tmp/sirius_web.log 2>&1 &
    for _ in $(seq 1 30); do ss -ltn 2>/dev/null | grep -q ":$PORT " && break; sleep 0.3; done
fi
xdg-open "http://localhost:$PORT" >/dev/null 2>&1 || true
