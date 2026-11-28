#!/bin/bash
# Dedicated Sirius console. The server survives browser/terminal closure.
set -u
WS="$HOME/sirius_jazzy_ws"
PORT="${SIRIUS_WEB_PORT:-8090}"
DOMAIN="${ROS_DOMAIN_ID:-53}"
MANAGED_PROFILE="$HOME/.local/share/sirius/console-browser"
BROWSER_PROFILE="${SIRIUS_WEB_BROWSER_PROFILE:-$MANAGED_PROFILE}"
STATE="${XDG_STATE_HOME:-$HOME/.local/state}/sirius_console"
mkdir -p "$STATE"
if [ "${1:-}" = "--serve" ]; then
    export SIRIUS_WEB_SUPERVISED=1 ROS_DOMAIN_ID="$DOMAIN"
    cd "$WS/other_programs/sirius_launcher" || exit 1
    while true; do
        python3 web_server.py --host 0.0.0.0 --port "$PORT" --ros-domain "$DOMAIN"
        result=$?
        [ "$result" -eq 99 ] || exit "$result"
        sleep 0.5
    done
fi
URL="http://127.0.0.1:$PORT"
APP_URL="$URL/console"
WINDOW_CLASS="SiriusLauncher-$PORT"
WINDOW_FINDER="$(dirname "${BASH_SOURCE[0]}")/../../other_programs/sirius_launcher/launcher_windows.py"
healthy() { curl --silent --fail --max-time 2 "$URL/api/health" | python3 -c 'import json,sys; sys.exit(json.load(sys.stdin).get("service") != "sirius-launcher")' >/dev/null 2>&1; }
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/sirius_console_${UID}_${PORT}.lock"
flock 9
if ! healthy; then
    if ss -ltn 2>/dev/null | grep -q ":$PORT "; then
        echo "Port $PORT is occupied by another service or an older launcher. Update/restart that launcher first." >&2
        exit 1
    fi
    setsid nohup bash "$0" --serve </dev/null >"$STATE/server.log" 2>&1 9>&- &
    for _ in $(seq 1 40); do
        healthy && break
        sleep 0.3
    done
    if ! healthy; then
        echo "Console failed to start. See $STATE/server.log" >&2
        exit 1
    fi
fi
# Keep the startup lock until the browser window is mapped as well. Browser
# children close fd 9 so the lock is not retained for their entire lifetime.
[ "${SIRIUS_WEB_NO_BROWSER:-0}" = "1" ] && exit 0
if [ -f "$WS/bash/startup_bash/detect_gui_env.sh" ]; then
    source "$WS/bash/startup_bash/detect_gui_env.sh"
fi
gui_ready=0
for _ in $(seq 1 60); do
    if declare -F sirius_detect_gui_env >/dev/null; then sirius_detect_gui_env || true; fi
    if wmctrl -m >/dev/null 2>&1; then gui_ready=1; break; fi
    sleep 1
done
if [ "$gui_ready" != 1 ]; then
    echo "Desktop window manager did not become ready." >&2
    exit 1
fi
focus_console() {
    local windows wid fullscreen_args
    windows=$(python3 "$WINDOW_FINDER" "$PORT") || return 1
    wid=${windows%%$'\n'*}
    [ -n "$wid" ] || return 1
    fullscreen_args=("$wid")
    if [ -n "${SIRIUS_WEB_WORKSPACE:-}" ]; then
        fullscreen_args+=("$SIRIUS_WEB_WORKSPACE")
    else
        # An explicit launch brings the existing app forward. Login leaves the
        # current workspace active and prepares Launcher on its own workspace.
        wmctrl -i -a "$wid" || true
    fi
    if [ "${SIRIUS_WEB_KIOSK:-1}" = "1" ]; then
        python3 "$(dirname "$WINDOW_FINDER")/fullscreen_workspace.py" "${fullscreen_args[@]}" || return 2
    fi
    return 0
}
focus_console
existing_result=$?
if [ "$existing_result" -eq 0 ]; then exit 0; fi
if [ "$existing_result" -eq 2 ]; then
    for _ in $(seq 1 100); do
        sleep 0.3
        if focus_console; then exit 0; fi
    done
    echo "Existing Launcher window did not reach fullscreen." >&2
    exit 1
fi
for browser in google-chrome google-chrome-stable chromium chromium-browser; do
    if command -v "$browser" >/dev/null 2>&1; then
        BROWSER_ARGS=("--app=$APP_URL" "--user-data-dir=$BROWSER_PROFILE" --no-first-run --no-default-browser-check --disable-session-crashed-bubble "--class=$WINDOW_CLASS")
        if [ "$BROWSER_PROFILE" = "$MANAGED_PROFILE" ]; then
            # Autologin does not unlock GNOME Keyring. This local-only profile
            # does not save passwords and must start without a keyring prompt.
            if [ ! -L "$BROWSER_PROFILE/SingletonLock" ]; then
                python3 - "$BROWSER_PROFILE" <<'PY' || exit 1
import json
from pathlib import Path
import sys
import tempfile

profile = Path(sys.argv[1])
profile.mkdir(mode=0o700, parents=True, exist_ok=True)
path = profile / "Default" / "Preferences"
path.parent.mkdir(mode=0o700, exist_ok=True)
prefs = json.loads(path.read_text()) if path.exists() else {}
settings = prefs.setdefault("profile", {})
if prefs.get("credentials_enable_service") is not False or settings.get("password_manager_enabled") is not False:
    prefs["credentials_enable_service"] = False
    settings["password_manager_enabled"] = False
    with tempfile.NamedTemporaryFile(mode="w", dir=path.parent, delete=False) as tmp:
        json.dump(prefs, tmp, ensure_ascii=False)
    Path(tmp.name).replace(path)
PY
            fi
            BROWSER_ARGS+=(--password-store=basic)
        fi
        printf '%s Opening %s\n' "$(date -Is)" "$APP_URL" >>"$STATE/browser.log"
        "$browser" "${BROWSER_ARGS[@]}" >>"$STATE/browser.log" 2>&1 9>&- &
        # Wait for the dedicated app window and for X11 to confirm fullscreen.
        for _ in $(seq 1 100); do
            if focus_console; then exit 0; fi
            sleep 0.3
        done
        echo "Launcher window did not reach fullscreen. See $STATE/browser.log" >&2
        exit 1
    fi
done
echo "No supported browser found for the Launcher window." >&2
exit 1
