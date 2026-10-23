#!/usr/bin/env python3
"""Switch existing Sirius windows, with a small return button on the Face screen."""
import argparse
import ast
import fcntl
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import time

from face_bridge import FaceBridge, FaceError


def active_window(bridge):
    result = subprocess.run(['xprop', '-root', '_NET_ACTIVE_WINDOW'], env=bridge.gui_env(),
                            capture_output=True, text=True, timeout=1)
    match = re.search(r'0x[0-9a-fA-F]+', result.stdout)
    return int(match[0], 16) if match else 0


def toggle(bridge):
    faces = bridge.windows(bridge.shortcut())
    target = 'launcher' if any(int(w['id'], 16) == active_window(bridge) for w in faces) else 'face'
    return bridge.focus_screen(target)


def install(bridge):
    """Install only our own GNOME bindings, preserving all other custom shortcuts."""
    schema = 'org.gnome.settings-daemon.plugins.media-keys'
    env = bridge.gui_env()
    def gsettings(*args):
        return subprocess.run(['gsettings', *args], env=env, check=True, capture_output=True,
                              text=True, timeout=3).stdout.strip()
    raw = gsettings('get', schema, 'custom-keybindings')
    paths = ast.literal_eval(raw.removeprefix('@as '))
    script = Path(__file__).resolve()
    for name, mode, binding in [('toggle', 'toggle', '<Control><Alt>f'),
                                ('launcher', 'launcher', '<Control><Alt>w')]:
        path = '/org/gnome/settings-daemon/plugins/media-keys/custom-keybindings/sirius-'+name+'/'
        target = 'org.gnome.settings-daemon.plugins.media-keys.custom-keybinding:'+path
        gsettings('set', target, 'name', 'Sirius '+name)
        gsettings('set', target, 'command', shlex.join(['/usr/bin/python3', str(script), mode]))
        gsettings('set', target, 'binding', binding)
        if path not in paths:
            paths.append(path)
    gsettings('set', schema, 'custom-keybindings', repr(paths))
    autostart = bridge.home/'.config/autostart/sirius-screen-switch.desktop'
    autostart.parent.mkdir(parents=True, exist_ok=True)
    # Desktop-entry Exec quoting differs from shell quoting.
    quoted = str(script).replace('\\', '\\\\').replace('"', '\\"').replace('`', '\\`').replace('$', '\\$').replace('%', '%%')
    autostart.write_text('[Desktop Entry]\nType=Application\nName=Sirius screen switch\n'
                         f'Exec=/usr/bin/python3 "{quoted}" overlay\nTerminal=false\n'
                         'X-GNOME-Autostart-enabled=true\nX-GNOME-Autostart-Delay=4\n')


def overlay(bridge):
    runtime = Path(os.environ.get('XDG_RUNTIME_DIR', '/tmp'))
    # Keep the file descriptor open for the entire GTK lifetime.
    lock = (runtime/f'sirius-screen-switch-{os.getuid()}.lock').open('w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        return
    import gi
    gi.require_version('Gtk', '3.0')
    from gi.repository import Gtk, Gdk, GLib
    window = Gtk.Window(title='Sirius screen switch')
    window.set_decorated(False)
    window.set_resizable(False)
    window.set_accept_focus(False)
    window.set_focus_on_map(False)
    window.set_skip_taskbar_hint(True)
    window.set_skip_pager_hint(True)
    window.set_keep_above(True)
    window.stick()
    button = Gtk.Button(label='← Launcherへ戻る')
    button.set_size_request(205, 52)
    button.set_tooltip_text('Ctrl + Alt + F でも切り替えられます')
    style = Gtk.CssProvider()
    style.load_from_data(b'button { background-image: none; background-color: #172237; color: #ffffff; border: 2px solid #83b5ff; border-radius: 10px; font-size: 15px; padding: 8px 14px; } button:hover { background-color: #2b4270; }')
    button.get_style_context().add_provider(style, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)
    window.add(button)
    def back(_):
        try:
            bridge.focus_screen('launcher')
            window.hide()
        except (FaceError, OSError, subprocess.SubprocessError) as error:
            button.set_tooltip_text(str(error))
    button.connect('clicked', back)
    hover_until = 0
    def tick():
        nonlocal hover_until
        try:
            faces = bridge.windows(bridge.shortcut())
            active = active_window(bridge)
            visible = any(int(w['id'], 16) == active for w in faces)
            # In case a WM temporarily activates the button, keep it available.
            own = window.get_window()
            if own and active == own.get_xid():
                visible = True
            if visible:
                display = Gdk.Display.get_default()
                face = next((w for w in faces if int(w['id'], 16) == active), None)
                monitor = display.get_primary_monitor() or display.get_monitor(0)
                if face:
                    # Match the Face monitor instead of assuming the primary screen.
                    foreign = GdkX11.X11Window.foreign_new_for_display(display, int(face['id'], 16))
                    if foreign:
                        monitor = display.get_monitor_at_window(foreign)
                area = monitor.get_workarea()
                window.move(area.x+area.width-225, area.y+16)
                _, px, py = display.get_default_seat().get_pointer().get_position()
                in_corner = (area.x+area.width-225 <= px < area.x+area.width
                             and area.y <= py < area.y+90)
                if in_corner:
                    hover_until = time.monotonic()+1.5
                if time.monotonic() < hover_until:
                    window.show_all()
                else:
                    window.hide()
            elif window.get_visible():
                window.hide()
                hover_until = 0
        except (OSError, subprocess.SubprocessError, ValueError) as error:
            print(str(error), file=sys.stderr, flush=True)
            window.hide()
        return True
    # The robot desktop runs X11; these functions are not available on Wayland.
    gi.require_version('GdkX11', '3.0')
    from gi.repository import GdkX11
    GLib.timeout_add(700, tick)
    tick()
    Gtk.main()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['face', 'launcher', 'toggle', 'overlay', 'install'])
    args = parser.parse_args()
    bridge = FaceBridge()
    os.environ.update({k: v for k, v in bridge.gui_env().items()
                       if k in {'DISPLAY', 'XAUTHORITY', 'XDG_RUNTIME_DIR', 'DBUS_SESSION_BUS_ADDRESS'}})
    try:
        if args.mode == 'install':
            install(bridge)
        elif args.mode == 'overlay':
            overlay(bridge)
        elif args.mode == 'toggle':
            toggle(bridge)
        else:
            bridge.focus_screen(args.mode)
    except (FaceError, OSError, subprocess.SubprocessError) as error:
        print(error, file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
