#!/bin/bash

# ==============================================================================
# Sirius ROSBridge + WSS Tunnel Launcher
# Port 9090 の rosbridge_server と ngrok (または Cloudflare Tunnel) を同時起動し、
# HTTPS/Vercel (タブレット環境) から接続可能な WSS URL を自動生成・表示します。
# ==============================================================================

WS_DIR=$(cd "$(dirname "$0")/../.." && pwd)
export PATH="$HOME/.local/bin:$PATH"

echo "======================================================"
echo " 🚀 Sirius ROSBridge + WSS Launcher"
echo "======================================================"

# Workspace Sourcing
if [ -f "$WS_DIR/install/setup.bash" ]; then
    echo "[1/3] Sourcing workspace: $WS_DIR"
    # shellcheck disable=SC1090
    source "$WS_DIR/install/setup.bash"
fi

ROSBRIDGE_PID=""
TUNNEL_PID=""
LOG_DIR="/tmp"

cleanup() {
    echo ""
    echo "[ROSBridge] 終了処理を実行中..."
    if [ -n "$TUNNEL_PID" ]; then
        kill "$TUNNEL_PID" 2>/dev/null
    fi
    if [ -n "$ROSBRIDGE_PID" ]; then
        kill "$ROSBRIDGE_PID" 2>/dev/null
    fi
    rm -f /tmp/rosbridge_wss_url.txt
    echo "[ROSBridge] 正常に停止しました。"
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# ------------------------------------------------------------------------------
# 1. rosbridge_server 起動確認・起動
# ------------------------------------------------------------------------------
if ss -tulpn 2>/dev/null | grep -q ":9090 "; then
    echo "[2/3] ポート 9090 で既に rosbridge または別プロセスが動作中です。"
    echo "      既存の rosbridge にトンネルを接続します。"
else
    echo "[2/3] rosbridge_server をポート 9090 で起動中..."
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 > "$LOG_DIR/rosbridge_internal.log" 2>&1 &
    ROSBRIDGE_PID=$!
    sleep 2
fi

# ------------------------------------------------------------------------------
# 2. WSS トンネル (ngrok または Cloudflare Tunnel) の起動
# ------------------------------------------------------------------------------
echo "[3/3] WSS トンネルを開設中..."

# ngrok authtoken の設定有無をチェック
HAS_NGROK_TOKEN=false
if [ -n "$NGROK_AUTHTOKEN" ]; then
    HAS_NGROK_TOKEN=true
elif [ -f "$HOME/.config/ngrok/ngrok.yml" ] && grep -q "authtoken:" "$HOME/.config/ngrok/ngrok.yml"; then
    HAS_NGROK_TOKEN=true
fi

TUNNEL_TYPE=""
PUBLIC_WSS_URL=""

if [ "$HAS_NGROK_TOKEN" = true ]; then
    TUNNEL_TYPE="ngrok"
    echo "  -> ngrok を使用してトンネルを作成します..."
    ngrok http 9090 --log=stdout > "$LOG_DIR/ngrok_rosbridge.log" 2>&1 &
    TUNNEL_PID=$!

    # ngrok の API から URL を取得 (最大12秒待機)
    for i in {1..12}; do
        RAW_URL=$(curl -s http://127.0.0.1:4040/api/tunnels 2>/dev/null | grep -o 'https://[^" ]*' | head -n 1)
        if [ -n "$RAW_URL" ]; then
            PUBLIC_WSS_URL="${RAW_URL/https:\/\//wss:\/\/}"
            break
        fi
        sleep 1
    done
else
    # ngrok のトークンが未設定の場合は Cloudflare Tunnel (アカウント不要) をフォールバック起動
    TUNNEL_TYPE="Cloudflare Tunnel (即時利用可)"
    echo "  --------------------------------------------------------"
    echo "  ℹ️  ngrok の authtoken がまだ設定されていません。"
    echo "     ngrok を利用する場合は、以下のコマンドで登録してください:"
    echo "       ngrok config add-authtoken <YOUR_TOKEN>"
    echo "  ⚡ 今回は登録不要の Cloudflare Tunnel で WSS を自動開設します！"
    echo "  --------------------------------------------------------"

    cloudflared tunnel --url http://localhost:9090 > "$LOG_DIR/cloudflared_rosbridge.log" 2>&1 &
    TUNNEL_PID=$!

    # ログから trycloudflare.com URL を抽出 (最大15秒待機)
    for i in {1..15}; do
        RAW_URL=$(grep -o 'https://[-a-zA-Z0-9.]*trycloudflare\.com' "$LOG_DIR/cloudflared_rosbridge.log" 2>/dev/null | head -n 1)
        if [ -n "$RAW_URL" ]; then
            PUBLIC_WSS_URL="${RAW_URL/https:\/\//wss:\/\/}"
            break
        fi
        sleep 1
    done
fi

LOCAL_IP=$(hostname -I 2>/dev/null | awk '{print $1}')

# ------------------------------------------------------------------------------
# 3. 接続情報バナーの表示
# ------------------------------------------------------------------------------
echo ""
echo -e "\033[1;32m====================================================================\033[0m"
echo -e "\033[1;32m  ✨ ROSBridge + WSS トンネル接続準備完了！\033[0m"
echo -e "\033[1;32m====================================================================\033[0m"
echo -e "  [モード] : $TUNNEL_TYPE"
echo -e "  [ローカル接続 (PC / 同一LAN内)]:"
echo -e "    ws://localhost:9090"
if [ -n "$LOCAL_IP" ]; then
    echo -e "    ws://${LOCAL_IP}:9090"
fi
echo ""
echo -e "\033[1;36m  [外部・Vercel・タブレット接続用 (WSS / HTTPS必須)]:\033[0m"
if [ -n "$PUBLIC_WSS_URL" ]; then
    echo -e "\033[1;33;40m   👉  ${PUBLIC_WSS_URL}  👈 \033[0m"
    echo "$PUBLIC_WSS_URL" > /tmp/rosbridge_wss_url.txt

    # Vercel アプリのベースURLが設定されていればパラメータ付きURLを生成
    QR_PAYLOAD="$PUBLIC_WSS_URL"
    if [ -f "$HOME/.vercel_app_url" ]; then
        VERCEL_BASE=$(cat "$HOME/.vercel_app_url" | tr -d '\r\n ')
        if [ -n "$VERCEL_BASE" ]; then
            # 末尾スラッシュを削除
            VERCEL_BASE="${VERCEL_BASE%/}"
            QR_PAYLOAD="${VERCEL_BASE}/?ros=${PUBLIC_WSS_URL}"
        fi
    fi

    # 端末内に QR コードを表示 (カメラで読み取り可能)
    python3 -c "
import qrcode, sys
try:
    qr = qrcode.QRCode(border=1)
    qr.add_data('$QR_PAYLOAD')
    print('\n【QRコード (アプリの「📷 QR読取」またはタブレットのカメラでスキャン)】')
    qr.print_ascii(invert=True)
except Exception:
    pass
" 2>/dev/null

else
    echo -e "\033[1;31m   ⚠️ トンネルURLの自動取得に失敗しました。ログを確認してください。\033[0m"
fi

echo -e "\033[1;32m--------------------------------------------------------------------\033[0m"
echo -e "  📱 タブレット側の接続手順:"
echo -e "    1. Androidタブレットで Vercel の操作画面を開く"
echo -e "    2. 接続URLの横にある \033[1;36m[📷 QR読取]\033[0m ボタンをタップ"
echo -e "    3. カメラで上記のQRコードを映すと、URLが自動入力され \033[1;32m即座に接続\033[0m されます！"
echo -e "\033[1;32m====================================================================\033[0m"
echo -e "  ※ 終了するには SiriusLauncher の「停止」ボタンまたは Ctrl+C を押してください。"
echo ""

# プロセスを維持
if [ -n "$ROSBRIDGE_PID" ]; then
    wait "$ROSBRIDGE_PID"
else
    # 既存の rosbridge をトンネルしている場合はトンネルプロセスの終了を監視
    wait "$TUNNEL_PID"
fi
