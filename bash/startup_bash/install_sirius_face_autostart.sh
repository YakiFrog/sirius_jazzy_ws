#!/bin/bash
# Register the user's existing SiriusFace shortcut for desktop login.
set -eu
WS="$HOME/sirius_jazzy_ws"
STARTER="$WS/other_programs/sirius_launcher/startup_face.py"
TARGET="${XDG_CONFIG_HOME:-$HOME/.config}/autostart/sirius-face.desktop"
if [ "${1:-}" = "--remove" ]; then
    rm -f -- "$TARGET"
    echo "SiriusFace autostart disabled."
    exit 0
fi
[ -f "$STARTER" ] || { echo "SiriusFace startup script not found: $STARTER" >&2; exit 1; }
mkdir -p "$(dirname "$TARGET")"
cat > "$TARGET" <<EOF
[Desktop Entry]
Type=Application
Name=SiriusFace
Comment=SiriusFace full-screen display
Exec=/usr/bin/python3 "$STARTER"
Terminal=false
X-GNOME-Autostart-enabled=true
X-GNOME-Autostart-Delay=5
StartupNotify=false
EOF
chmod 644 "$TARGET"
echo "SiriusFace autostart enabled: $TARGET"
