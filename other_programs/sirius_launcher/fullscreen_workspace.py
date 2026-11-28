"""Place a Sirius window fullscreen on its X11 workspace."""

import re
import subprocess
import sys
import time


def ensure_fullscreen(window, workspace=None, env=None):
    def run(*args):
        try:
            result = subprocess.run(args, env=env, capture_output=True, text=True, timeout=2)
        except (OSError, subprocess.TimeoutExpired):
            return None
        return result.stdout if result.returncode == 0 else None

    def wait_for(condition):
        deadline = time.monotonic() + 1
        while not condition():
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.05)
        return True

    def on_workspace():
        output = run("xprop", "-id", window, "_NET_WM_DESKTOP") or ""
        match = re.search(r"=\s*(\d+)\s*$", output)
        return match is not None and int(match[1]) == workspace

    if workspace is not None and not on_workspace():
        if run("wmctrl", "-i", "-r", window, "-t", str(workspace)) is None:
            return False
        if not wait_for(on_workspace):
            return False

    def is_fullscreen():
        output = run("xprop", "-id", window, "_NET_WM_STATE") or ""
        return "_NET_WM_STATE_FULLSCREEN" in output

    return is_fullscreen() or (
        run("wmctrl", "-i", "-r", window, "-b", "add,fullscreen") is not None
        and wait_for(is_fullscreen)
    )


if __name__ == "__main__":
    workspace = int(sys.argv[2]) if len(sys.argv) > 2 else None
    sys.exit(0 if ensure_fullscreen(sys.argv[1], workspace) else 1)
