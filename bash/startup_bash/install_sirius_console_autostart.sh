#!/bin/bash
# Start only the console UI at desktop login; robot programs remain user-controlled.
set -eu
WS="$HOME/sirius_jazzy_ws"
TARGET="${XDG_CONFIG_HOME:-$HOME/.config}/autostart/sirius-console.desktop"
if [ "${1:-}" = "--remove" ]; then
    if [ -f "$TARGET" ]; then
        rm -- "$TARGET"
    fi
    echo "Console autostart disabled."
    exit 0
fi
if [ ! -f "$WS/bash/startup_bash/start_sirius_console.sh" ]; then
    echo "Console startup script was not found in $WS" >&2
    exit 1
fi
mkdir -p "$(dirname "$TARGET")"
cat > "$TARGET" <<EOF
[Desktop Entry]
Type=Application
Name=Sirius Control Console
Comment=Sirius full-screen launcher
Exec=env ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-53} SIRIUS_WEB_KIOSK=1 bash "$WS/bash/startup_bash/start_sirius_console.sh"
Terminal=false
X-GNOME-Autostart-enabled=true
X-GNOME-Autostart-Delay=3
StartupNotify=false
EOF
chmod 644 "$TARGET"
echo "Console autostart enabled: $TARGET"
