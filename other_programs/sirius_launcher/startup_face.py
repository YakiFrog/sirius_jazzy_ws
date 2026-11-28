#!/usr/bin/env python3
"""Start the desktop Face shortcut once and verify its X11 fullscreen state."""
import fcntl
import os
from pathlib import Path
import subprocess
import sys
import time

from face_bridge import FaceBridge
from fullscreen_workspace import ensure_fullscreen


def main():
    workspace = os.environ.get('SIRIUS_FACE_WORKSPACE')
    workspace = int(workspace) if workspace else None
    runtime = Path(os.environ.get('XDG_RUNTIME_DIR', '/tmp'))
    with (runtime / f'sirius_face_autostart_{os.getuid()}.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        bridge = FaceBridge()
        shortcut = bridge.shortcut()
        if not shortcut:
            raise RuntimeError('SiriusFace desktop shortcut not found')
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            # Do not cache an incomplete GUI environment during login.
            bridge.gui = None
            env = bridge.gui_env()
            try:
                result = subprocess.run(['wmctrl', '-m'], env=env, capture_output=True, timeout=2)
                if result.returncode == 0:
                    break
            except (OSError, subprocess.TimeoutExpired):
                pass
            time.sleep(1)
        else:
            raise RuntimeError('Desktop window manager did not become ready')

        windows = bridge.windows(shortcut)
        if not windows and not bridge.processes(shortcut):
            subprocess.Popen(['gio', 'launch', str(shortcut['path'])], env=env,
                             stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL, close_fds=True)
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            windows = bridge.windows(shortcut)
            if windows:
                window = windows[0]['id']
                if ensure_fullscreen(window, workspace, env):
                    print(f'SiriusFace fullscreen: {window}', flush=True)
                    return
            time.sleep(.3)
        raise RuntimeError('SiriusFace window did not reach fullscreen')


if __name__ == '__main__':
    try:
        main()
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
        print(f'SiriusFace autostart: {error}', file=sys.stderr)
        sys.exit(1)
