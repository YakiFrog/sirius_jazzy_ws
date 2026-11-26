"""Find the X11 windows that belong to the local Launcher app."""

import os
from pathlib import Path
import subprocess
import sys


def launcher_windows(port, env=None):
    try:
        result = subprocess.run(
            ["wmctrl", "-lpx"], env=env, capture_output=True, text=True, timeout=1,
        )
    except (OSError, subprocess.TimeoutExpired):
        return []
    hosts = ("127.0.0.1", "localhost")
    instances = {host + suffix for host in hosts for suffix in ("", "__console")}
    urls = {f"--app=http://{host}:{port}{path}" for host in hosts for path in ("", "/", "/console")}
    dedicated = []
    legacy = []
    for line in result.stdout.splitlines():
        fields = line.split(None, 5)
        if len(fields) < 5 or not fields[2].isdigit():
            continue
        instance, _, window_class = fields[3].lower().rpartition(".")
        if instance not in instances:
            continue
        window = {"id": fields[0], "pid": int(fields[2])}
        if window_class == f"siriuslauncher-{port}":
            dedicated.append(window)
            continue
        try:
            command = Path("/proc", fields[2], "cmdline").read_bytes()
        except OSError:
            continue
        options = set(command.replace(b"\0", b" ").decode(errors="replace").split())
        if window_class in ("siriuslauncher", "google-chrome", "chromium", "chromium-browser") and options & urls:
            legacy.append(window)
    return dedicated + legacy


if __name__ == "__main__":
    for window in launcher_windows(sys.argv[1] if len(sys.argv) > 1 else os.environ.get("SIRIUS_WEB_PORT", "8090")):
        print(window["id"])
