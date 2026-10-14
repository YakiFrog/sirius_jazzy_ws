#!/bin/bash
# ブラウザ版SiriusLauncherをTerminatorのタブで起動し、ブラウザで開く。
# Terminatorを閉じるとWebサーバーも終了する。
set -u
WS="$HOME/sirius_jazzy_ws"
DIR="$WS/other_programs/sirius_launcher"
PORT="${SIRIUS_WEB_PORT:-8090}"
DOMAIN="${ROS_DOMAIN_ID:-53}"

open_browser() {
    # 遠隔再起動時などはブラウザを開かない
    if [ "${SIRIUS_WEB_NO_BROWSER:-0}" = "1" ]; then
        return 0
    fi
    local url="http://localhost:$PORT"
    # 既にアプリウィンドウが開いていればそれを前面に出す（多重起動を防ぐ）
    if wmctrl -l 2>/dev/null | grep -q "SiriusLauncher Web"; then
        wmctrl -a "SiriusLauncher Web" >/dev/null 2>&1
        return
    fi
    # Chromeのアプリモードで開く（タブ・アドレスバー・戻るボタンなしのネイティブ風表示）
    for bin in google-chrome google-chrome-stable chromium chromium-browser; do
        if command -v "$bin" >/dev/null 2>&1; then
            "$bin" --app="$url" >/dev/null 2>&1 &
            return
        fi
    done
    xdg-open "$url" >/dev/null 2>&1 || true
}

# すでに起動している場合はブラウザを開くだけ
if ss -ltn 2>/dev/null | grep -q ":$PORT "; then
    open_browser
    exit 0
fi

# Terminatorのタブ内で実行する起動スクリプト（毎回作り直す）
# スーパーバイザとして動作し、再起動要求(exit 99)で web_server を再起動する。
RUN_SCRIPT="/tmp/sirius_launcher_web_run.sh"
cat > "$RUN_SCRIPT" <<EOF
#!/bin/bash
export ROS_DOMAIN_ID="$DOMAIN"
export SIRIUS_WEB_SUPERVISED=1
if [ -f "$WS/bash/startup_bash/detect_gui_env.sh" ]; then
    source "$WS/bash/startup_bash/detect_gui_env.sh"
    sirius_detect_gui_env || true
fi
cd "$DIR" || exit 1
echo -ne "\\033]0;[SiriusWebServer]\\007"
while true; do
    python3 web_server.py --port "$PORT" --ros-domain "$DOMAIN"
    code=\$?
    if [ "\$code" -eq 99 ]; then
        echo "[supervisor] 再起動要求を受信しました。Webサーバーを再起動します..."
        sleep 0.5
        continue
    fi
    echo "[supervisor] web_server.py が終了しました (exit=\$code)"
    break
done
EOF
chmod +x "$RUN_SCRIPT"

# Terminatorの新規タブで起動（既存ウィンドウがあればそのタブ、無ければ新規ウィンドウ）。
# このタブを閉じる/ウィンドウを閉じるとWebサーバーも終了する。
if pgrep -x terminator >/dev/null 2>&1; then
    terminator --new-tab -e "bash $RUN_SCRIPT" >/dev/null 2>&1 &
else
    terminator -e "bash $RUN_SCRIPT" >/dev/null 2>&1 &
fi

# サーバーが立ち上がるのを待ってブラウザを開く
for _ in $(seq 1 40); do
    ss -ltn 2>/dev/null | grep -q ":$PORT " && break
    sleep 0.3
done
open_browser
