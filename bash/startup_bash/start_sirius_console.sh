#!/bin/bash
# Dedicated Sirius console. The server survives browser/terminal closure.
set -u
WS="$HOME/sirius_jazzy_ws"
PORT="${SIRIUS_WEB_PORT:-8090}"
DOMAIN="${ROS_DOMAIN_ID:-53}"
BROWSER_PROFILE="$HOME/.local/share/sirius/console-browser"
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
    sirius_detect_gui_env || true
fi
focus_console() {
    command -v wmctrl >/dev/null 2>&1 || return 1
    # Titles change between the shell and individual pages. Identify only our
    # dedicated Chrome profile and URL, without focusing other browser windows.
    local wid desktop pid host rest command_line
    while read -r wid desktop pid host rest; do
        [ -r "/proc/$pid/cmdline" ] || continue
        # Chrome can rewrite argv into a single space-separated string.
        # Normalize both forms and require complete option/value boundaries.
        command_line=" $(tr '\0' ' ' < "/proc/$pid/cmdline") "
        if [[ "$command_line" == *" --user-data-dir=$BROWSER_PROFILE "* ]] &&
           [[ "$command_line" == *" --app=$URL "* ]]; then
            wmctrl -i -a "$wid" || true
            [ "${SIRIUS_WEB_KIOSK:-1}" != "1" ] || wmctrl -i -r "$wid" -b add,fullscreen
            return 0
        fi
    done < <(wmctrl -lp)
    return 1
}
if focus_console; then
    exit 0
fi
for browser in google-chrome google-chrome-stable chromium chromium-browser; do
    if command -v "$browser" >/dev/null 2>&1; then
        BROWSER_ARGS=("--app=$URL" "--user-data-dir=$BROWSER_PROFILE" --no-first-run --no-default-browser-check --disable-session-crashed-bubble --class=SiriusLauncher)
        [ "${SIRIUS_WEB_KIOSK:-1}" = "1" ] && BROWSER_ARGS+=(--kiosk --start-fullscreen)
        "$browser" "${BROWSER_ARGS[@]}" >/dev/null 2>&1 9>&- &
        # App mode may initially ignore kiosk size; apply the window state once mapped.
        for _ in $(seq 1 30); do
            focus_console && break
            sleep 0.3
        done
        exit 0
    fi
done
xdg-open "$URL" >/dev/null 2>&1 9>&- &
