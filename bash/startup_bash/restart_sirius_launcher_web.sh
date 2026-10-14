#!/bin/bash
# 遠隔（SSHなど）から SiriusLauncher Web を再起動する。
#   使い方: bash ~/sirius_jazzy_ws/bash/startup_bash/restart_sirius_launcher_web.sh
# GUIセッションが検出できれば Terminator のタブで起動、無ければヘッドレスで起動する。
set -u
USER_NAME="${USER:-$(id -un)}"
LAUNCH="$HOME/sirius_jazzy_ws/bash/desktop_shortcut/sirius_launcher_web.sh"
PORT="${SIRIUS_WEB_PORT:-8090}"
DOMAIN="${ROS_DOMAIN_ID:-53}"

echo "[1/3] 起動中のWebサーバーを停止"
if pkill -f "web_server.py" 2>/dev/null; then
    echo "  停止しました"
else
    echo "  起動していません"
fi
for _ in $(seq 1 20); do
    ss -ltn 2>/dev/null | grep -q ":$PORT " || break
    sleep 0.3
done

echo "[2/3] GUIセッション環境を検出"
source "$HOME/sirius_jazzy_ws/bash/startup_bash/detect_gui_env.sh"
if sirius_detect_gui_env; then
    echo "  DISPLAY=$DISPLAY を使用"
    echo "[3/3] Terminatorタブで起動"
    SIRIUS_WEB_NO_BROWSER=1 bash "$LAUNCH"
else
    echo "  GUIセッション未検出のためヘッドレスで起動"
    SUPERVISOR="/tmp/sirius_launcher_web_run.sh"
    cat > "$SUPERVISOR" <<EOF
#!/bin/bash
export ROS_DOMAIN_ID="$DOMAIN"
export SIRIUS_WEB_SUPERVISED=1
cd "$HOME/sirius_jazzy_ws/other_programs/sirius_launcher" || exit 1
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
    chmod +x "$SUPERVISOR"
    setsid nohup bash "$SUPERVISOR" </dev/null >/tmp/sirius_web.log 2>&1 &
    disown 2>/dev/null || true
    for _ in $(seq 1 40); do
        ss -ltn 2>/dev/null | grep -q ":$PORT " && break
        sleep 0.3
    done
fi

sleep 1
if ss -ltn 2>/dev/null | grep -q ":$PORT "; then
    echo "再起動しました: http://localhost:$PORT"
else
    echo "起動に失敗した可能性があります。ログ: /tmp/sirius_web.log"
    exit 1
fi
