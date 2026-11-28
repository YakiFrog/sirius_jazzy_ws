"""Access policy for a Launcher reached through Tailscale Serve."""

from ipaddress import ip_address
import json
from pathlib import Path
from urllib.parse import urlsplit

from flask import g, jsonify, request


CONFIG_PATH = Path.home() / ".config/sirius/launcher-tailnet.json"


def load_tailnet_access(path=CONFIG_PATH):
    try:
        data = json.loads(path.read_text())
    except FileNotFoundError:
        return None
    origin, login = data["origin"], data["login"]
    url = urlsplit(origin)
    if (url.scheme != "https" or not url.hostname or not url.hostname.endswith(".ts.net")
            or origin != f"https://{url.hostname}" or not isinstance(login, str)
            or not login.strip()):
        raise ValueError("Invalid Launcher Tailscale origin or login")
    return origin, login


def loopback_peer():
    original = request.environ.get("werkzeug.proxy_fix.orig", {})
    try:
        peer = ip_address(original.get("REMOTE_ADDR") or request.remote_addr or "")
    except ValueError:
        return False
    if getattr(peer, "ipv4_mapped", None) is not None:
        peer = peer.ipv4_mapped
    return peer.is_loopback


def direct_local_request():
    return (
        loopback_peer()
        and urlsplit(request.host_url).hostname in {"127.0.0.1", "::1", "localhost"}
        and not any(request.headers.get(header) for header in (
            "Forwarded", "X-Forwarded-Host", "X-Forwarded-Proto", "X-Forwarded-For",
            "Tailscale-User-Login",
        ))
    )


def enforce_tailnet_access(settings):
    if not settings:
        return None
    origin, login = settings
    g.launcher_tailnet_user = (
        loopback_peer() and request.host_url.rstrip("/") == origin
        and request.headers.get("Tailscale-User-Login") == login
    )
    local = direct_local_request()
    if not (local or g.launcher_tailnet_user):
        if request.path.startswith("/api/"):
            return jsonify(error="launcher_access"), 403
        return "このLauncherへのアクセスには、登録済みのTailscaleアカウントが必要です。", 403

    if request.method not in ("GET", "HEAD", "OPTIONS"):
        sender = request.headers.get("Origin")
        if (request.headers.get("Sec-Fetch-Site") in ("cross-site", "same-site")
                or (sender is not None and sender != request.host_url.rstrip("/"))
                or (g.launcher_tailnet_user and sender is None)):
            return jsonify(error="launcher_access"), 403
    return None
