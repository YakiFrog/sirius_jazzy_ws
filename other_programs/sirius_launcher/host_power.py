"""Guarded power actions for the PC running SiriusLauncher."""

from pathlib import PurePath
import socket
import subprocess

import psutil
from flask import Blueprint, g, jsonify, request

from launcher_access import direct_local_request


ACTIONS = {"reboot": "Reboot", "poweroff": "PowerOff"}


def login1(method, *arguments):
    return subprocess.run(
        ["busctl", "--timeout=35s", "call", "org.freedesktop.login1",
         "/org/freedesktop/login1", "org.freedesktop.login1.Manager", method, *arguments],
        capture_output=True, check=True, text=True, timeout=38,
    ).stdout.strip()


def recording_bag():
    for process in psutil.process_iter(["cmdline", "status"]):
        if process.info["status"] == psutil.STATUS_ZOMBIE:
            continue
        argv = process.info["cmdline"] or ()
        names = [PurePath(arg).name for arg in argv]
        if any(names[i:i + 3] == ["ros2", "bag", "record"] for i in range(len(names))):
            return True
        if "rosbag2_recorder" in names or any(
            name.startswith("record_rosbag") and name.endswith(".sh") for name in names
        ):
            return True
    return False


def register_host_power(app, password, lock):
    routes = Blueprint("host_power", __name__)

    @routes.route("/api/host-power", methods=["GET", "POST"])
    def host_power():
        secret = password()
        local = direct_local_request()
        if not (local or g.get("launcher_tailnet_user")
                or (secret and request.cookies.get("pw") == secret)):
            return jsonify(error="power_access"), 403

        host = socket.gethostname()
        if request.method == "GET":
            available = {}
            for action, method in ACTIONS.items():
                try:
                    result = login1("Can" + method)
                    available[action] = result.removeprefix('s ').strip('"')
                except (OSError, subprocess.SubprocessError):
                    available[action] = "unknown"
            response = jsonify(host=host, recording=recording_bag(), available=available)
            response.headers["Cache-Control"] = "no-store"
            return response

        if (
            request.headers.get("Origin") != request.host_url.rstrip("/")
            or request.headers.get("Sec-Fetch-Site") not in (None, "same-origin")
        ):
            return jsonify(error="power_access"), 403
        body = request.get_json(silent=True)
        if not isinstance(body, dict) or body.get("host") != host:
            return jsonify(error="power_target"), 400
        action = body.get("action")
        if not isinstance(action, str) or action not in ACTIONS:
            return jsonify(error="power_action"), 400
        with lock:
            try:
                capability = login1("Can" + ACTIONS[action])
                if capability not in ('s "yes"', 's "challenge"'):
                    return jsonify(error="power_unavailable"), 503
                if recording_bag():
                    return jsonify(error="recording_bag"), 409
                login1(ACTIONS[action], "b", str(capability == 's "challenge"').lower())
            except (OSError, subprocess.SubprocessError):
                return jsonify(error="power_unavailable"), 503
        return jsonify(accepted=True)

    app.register_blueprint(routes)
