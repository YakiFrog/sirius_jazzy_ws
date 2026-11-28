"""Read and change the PC's operating-system power profile."""

import json
import os
import re
import subprocess
from threading import Lock
from urllib.parse import urlsplit

from flask import Blueprint, g, jsonify, request

from launcher_access import direct_local_request, loopback_peer


PROFILES = ("performance", "balanced", "power-saver")
PROFILE_LINE = re.compile(r"^\s*(\*)?\s*(performance|balanced|power-saver):\s*$")


def read_profiles():
    output = subprocess.run(
        ["powerprofilesctl", "list"], env={**os.environ, "LC_ALL": "C"},
        capture_output=True, check=True, text=True, timeout=3,
    ).stdout
    entries = [match.groups() for line in output.splitlines() if (match := PROFILE_LINE.fullmatch(line))]
    available = [profile for _, profile in entries]
    active = next((profile for marker, profile in entries if marker), None)
    if not active:
        raise ValueError("Could not determine the active power profile")
    return dict(active=active, available=available)


def apply_profile(profile):
    # Run in the desktop user's manager even if Launcher was started from SSH.
    # The OS still authorizes the request through power-profiles-daemon.
    env = {**os.environ, "XDG_RUNTIME_DIR": f"/run/user/{os.getuid()}"}
    subprocess.run(
        ["systemd-run", "--user", "--wait", "--collect", "--quiet", "--pipe",
         "--property=Type=exec", "--property=RuntimeMaxSec=4s",
         "--", "/usr/bin/powerprofilesctl", "set", profile],
        env=env, stdin=subprocess.DEVNULL, capture_output=True, check=True, text=True, timeout=6,
    )


def can_change_profile():
    if direct_local_request() or g.get("launcher_tailnet_user"):
        return True
    origin = urlsplit(request.host_url)
    login = request.headers.get("Tailscale-User-Login")
    if not (loopback_peer() and origin.scheme == "https" and login and
            (origin.hostname or "").endswith(".ts.net")):
        return False
    # A Desktop install can expose Launcher through Serve without a Launcher
    # tailnet configuration. In that case accept the device owner's login.
    try:
        status = json.loads(subprocess.run(
            ["tailscale", "status", "--json", "--peers=false"], capture_output=True,
            check=True, text=True, timeout=2,
        ).stdout)
        host = status["Self"]
        owner = status["User"][str(host["UserID"])]["LoginName"]
        return str(host["DNSName"]).rstrip(".") == origin.hostname and owner == login
    except (KeyError, OSError, subprocess.SubprocessError, TypeError, ValueError):
        return False


def register_host_profile(app, auth_ok):
    routes = Blueprint("host_profile", __name__)
    lock = Lock()

    @routes.route("/api/host-profile", methods=["GET", "POST"])
    def host_profile():
        if not auth_ok():
            return jsonify(error="auth"), 401
        write = request.method == "POST"
        if write and (
            request.headers.get("Origin") != request.host_url.rstrip("/")
            or request.headers.get("Sec-Fetch-Site") not in (None, "same-origin")
        ):
            return jsonify(error="profile_access"), 403
        allowed = can_change_profile()
        if write and not allowed:
            return jsonify(error="profile_access"), 403
        if write:
            body = request.get_json(silent=True)
            profile = body.get("profile") if isinstance(body, dict) else None
            if profile not in PROFILES:
                return jsonify(error="invalid_profile"), 400
        try:
            with lock:
                state = read_profiles()
                if write and profile != state["active"]:
                    if profile not in state["available"]:
                        return jsonify(error="invalid_profile"), 400
                    apply_profile(profile)
                    state = read_profiles()
                    if state["active"] != profile:
                        return jsonify(error="profile_not_applied", state=state), 409
        except (OSError, subprocess.SubprocessError, ValueError) as failure:
            if write and isinstance(failure, subprocess.CalledProcessError):
                detail = failure.stderr or ""
                if "energy_performance_preference" in detail and ("Code26:" in detail or "Device or resource busy" in detail):
                    return jsonify(error="profile_cpu_locked"), 409
            error = "profile_change_failed" if write else "profile_unavailable"
            return jsonify(error=error), 503
        response = jsonify(**state, can_change=allowed)
        response.headers["Cache-Control"] = "no-store"
        return response

    app.register_blueprint(routes)
