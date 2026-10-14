#!/bin/bash
# ローカルGUIセッション(X/Wayland)の環境変数を検出してエクスポートする。
# SSH/X11転送(localhost:10.0等)から起動されたプロセスでも、
# NUC実ディスプレイ(:0等)で Terminator / RViz2 を開けるようにする。
#
# 使い方: source detect_gui_env.sh; sirius_detect_gui_env || true

sirius_detect_gui_env() {
    # X11転送先など、現在のDISPLAYを意図的に使いたい場合は SIRIUS_KEEP_DISPLAY=1
    if [ "${SIRIUS_KEEP_DISPLAY:-0}" = "1" ]; then
        return 0
    fi

    # 既にローカルディスプレイへ接続できるならそのまま使う
    if [ -n "${DISPLAY:-}" ]; then
        case "$DISPLAY" in
            :0|:0.*|:1|:1.*|unix:*)
                if timeout 3 xdpyinfo >/dev/null 2>&1; then
                    return 0
                fi
                ;;
        esac
    fi

    local user pid envsrc=""
    user="${USER:-$(id -un)}"
    for pid in \
        $(pgrep -u "$user" -x gnome-shell 2>/dev/null) \
        $(pgrep -u "$user" -x Xorg 2>/dev/null) \
        $(pgrep -u "$user" -x gnome-session 2>/dev/null) \
        $(pgrep -u "$user" -x terminator 2>/dev/null); do
        if [ -r "/proc/$pid/environ" ]; then
            envsrc="/proc/$pid/environ"
            break
        fi
    done
    if [ -z "$envsrc" ]; then
        return 1
    fi

    while IFS= read -r line; do
        case "$line" in
            DISPLAY=*|XAUTHORITY=*|XDG_RUNTIME_DIR=*|DBUS_SESSION_BUS_ADDRESS=*|WAYLAND_DISPLAY=*)
                export "$line"
                ;;
        esac
    done < <(tr '\0' '\n' < "$envsrc")

    # X11セッションで XAUTHORITY が無い場合は gdm のデフォルトを補完
    if [ -z "${XAUTHORITY:-}" ] && [ -r "/run/user/$(id -u)/gdm/Xauthority" ]; then
        export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
    fi

    if [ -n "${DISPLAY:-}" ] && timeout 3 xdpyinfo >/dev/null 2>&1; then
        return 0
    fi
    return 1
}
